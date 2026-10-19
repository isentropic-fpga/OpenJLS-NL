-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- nl_top: near-lossless JPEG-LS encoder, 4 vertical stripes coded as 4 images in parallel
--
-- Lane l codes stripe l (W/4 x H) onto slice l of the output buses. iNear, iImageWidth and
-- iImageHeight are sampled while iRst = '1'; W is a multiple of 4 in [32, MAX_IMAGE_WIDTH].
-- nl_config -> nl_frontend -> nl_loop -> nl_backend -> per-lane skid FIFO -> byte_stuffer_nl ->
-- nl_framer -> per-lane output FIFO. The stuffer and framer serve the lanes in turn (lane sSlot),
-- so the core emits at most 4 payload bytes per clock.

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use work.nl_pkg.all;

library work;
  use work.olo_base_pkg_math.log2ceil;

entity nl_top is
  generic (
    BITNESS          : positive range 8 to 16    := 12;
    MAX_IMAGE_WIDTH  : positive range 32 to 65532 := 4096;
    MAX_IMAGE_HEIGHT : positive range 1 to 65535 := 4096;
    OUT_WIDTH        : positive range 48 to 1024 := 64
  );
  port (
    iClk         : in    std_logic;
    iRst         : in    std_logic;
    iNear        : in    std_logic_vector(7 downto 0);
    iImageWidth  : in    std_logic_vector(15 downto 0);
    iImageHeight : in    std_logic_vector(15 downto 0);
    iValid       : in    std_logic;
    iPixel       : in    std_logic_vector(BITNESS - 1 downto 0);
    oReady       : out   std_logic;
    -- 4 output streams, lane l = slice l
    oData        : out   std_logic_vector(4 * OUT_WIDTH - 1 downto 0);
    oValid       : out   std_logic_vector(3 downto 0);
    oKeep        : out   std_logic_vector(4 * OUT_WIDTH / 8 - 1 downto 0);
    oLast        : out   std_logic_vector(3 downto 0);
    iReady       : in    std_logic_vector(3 downto 0)
  );
end entity nl_top;

architecture rtl of nl_top is

  constant RESET      : natural := 64;
  constant LIMIT      : natural := 2 * (BITNESS + maximum(8, BITNESS));
  constant LENW       : natural := log2ceil(LIMIT + 1);
  constant SKID_W     : natural := 1 + LENW + LIMIT;
  constant SKID_DEPTH : natural := 32;
  constant SKID_ALM   : natural := SKID_DEPTH - 12;                       -- words in flight after iHold
  constant BS_OUT_W   : natural := 32;
  constant FW_W       : natural := log2ceil(MAX_IMAGE_WIDTH / 4 + 1);
  constant FH_W       : natural := log2ceil(MAX_IMAGE_HEIGHT + 1);

  signal sWidth     : unsigned(15 downto 0);                            -- [32, MAX_IMAGE_WIDTH], multiple of 4
  signal sHeight    : unsigned(15 downto 0);                            -- [1, MAX_IMAGE_HEIGHT]
  signal sHold      : std_logic;                                      -- any skid FIFO almost full

  -- Config
  signal sCfgDone   : std_logic;
  signal sNear      : unsigned(7 downto 0);                             -- [0, NMAX] (nl_config)
  signal sRecip     : unsigned(BITNESS + 10 downto 0);                  -- 1/(2*NEAR+1) (nl_config)
  signal sRange     : unsigned(BITNESS downto 0);                       -- A.2 RANGE (nl_config)
  signal sRangeD    : unsigned(BITNESS + 1 downto 0);                   -- [MAXVAL+1, MAXVAL+4*NEAR+1] (nl_config)
  signal sQbpp      : unsigned(4 downto 0);                             -- [1, BITNESS]
  signal sAinit     : unsigned(BITNESS - 1 downto 0);                   -- [2, 2^(BITNESS-5)]
  signal sT1        : unsigned(BITNESS - 1 downto 0);                   -- C.2.4.1.1 thresholds (nl_config)
  signal sT2        : unsigned(BITNESS - 1 downto 0);                   -- C.2.4.1.1 thresholds (nl_config)
  signal sT3        : unsigned(BITNESS - 1 downto 0);                   -- C.2.4.1.1 thresholds (nl_config)

  -- Front end -> loop
  signal sClr       : std_logic;
  signal sClrAddr   : unsigned(8 downto 0);                             -- [0, 511]

  -- Kept by name in the netlist: the post-synth OSVVM TB probes it.
  attribute keep : string;
  attribute keep of sClr : signal is "true";

  signal fValid   : std_logic;
  signal fFirstPix  : std_logic;
  signal fFirstCol  : std_logic;
  signal fEol       : std_logic;
  signal fIx        : unsigned(BITNESS - 1 downto 0);                   -- [0, MAXVAL]
  signal fRb        : unsigned(BITNESS - 1 downto 0);                   -- [0, MAXVAL]
  signal fRc        : unsigned(BITNESS - 1 downto 0);                   -- [0, MAXVAL]
  signal fQ12Zero   : std_logic;
  signal fQ3Bound   : t_signed_array(0 to 7)(BITNESS + 1 downto 0);     -- nl_frontend bound ranges
  signal fCandCtx   : t_unsigned_array(0 to 8)(CO_CTX_WIDTH - 1 downto 0);-- [0, 368]
  signal fCandSign  : std_logic_vector(0 to 8);
  signal fCandC     : t_signed_array(0 to 8)(CO_C_WIDTH - 1 downto 0);  -- [MIN_C, MAX_C]
  signal fCandB     : t_signed_array(0 to 8)(6 downto 0);               -- [1-N, 0]
  signal fCandN     : t_unsigned_array(0 to 8)(6 downto 0);             -- [1, RESET]

  -- Loop exit
  signal xLoopValid : std_logic;                                     -- includes exits from before iRst
  signal xValid     : std_logic;
  signal xMode      : std_logic_vector(1 downto 0);
  signal xRiType    : std_logic;
  signal xRx        : unsigned(BITNESS - 1 downto 0);                   -- [0, MAXVAL]
  signal xErrval    : signed(BITNESS downto 0);                         -- A.9 range (nl_loop)
  signal xCtx       : unsigned(CO_CTX_WIDTH - 1 downto 0);              -- [0, 368]
  signal xCq        : signed(CO_C_WIDTH - 1 downto 0);                  -- [MIN_C, MAX_C]
  signal xBq        : signed(6 downto 0);                               -- [1-N, 0]
  signal xNq        : unsigned(6 downto 0);                             -- [1, RESET]
  signal xBpre      : signed(6 downto 0);                               -- [1-N, 0]
  signal xNpre      : unsigned(6 downto 0);                             -- [1, RESET]
  signal xNRes      : std_logic;
  signal xLane      : unsigned(1 downto 0);
  signal xFirstPix  : std_logic;
  signal xLastPix   : std_logic;
  signal xEol       : std_logic;

  -- Back end
  signal bWord      : std_logic_vector(LIMIT - 1 downto 0);
  signal bValid     : std_logic;
  signal bLen       : unsigned(LENW - 1 downto 0);                      -- [0, LIMIT]
  signal bLane      : unsigned(1 downto 0);
  signal bLast      : std_logic;

  signal sSkidIn    : std_logic_vector(SKID_W - 1 downto 0);
  signal sAlm       : std_logic_vector(3 downto 0);

  -- Shared stuffer + framer, lane sSlot this cycle
  constant OB_W     : natural := log2ceil(OUT_WIDTH / 8 + 1);
  constant OUTF_W   : natural := 1 + OB_W + OUT_WIDTH;
  signal sSlot      : unsigned(1 downto 0);                           -- [0, 3], lane served this cycle
  signal sSkidOut   : t_slv_array(0 to 3)(SKID_W - 1 downto 0);
  signal sSkidValid : std_logic_vector(3 downto 0);
  signal sPop       : std_logic_vector(3 downto 0);
  signal sBsAlm     : std_logic_vector(3 downto 0);
  signal sBsAlmReg  : std_logic_vector(3 downto 0);
  signal sStartPend : std_logic_vector(3 downto 0);                  -- per lane: image start not yet framed
  signal sBsIn      : std_logic_vector(SKID_W - 1 downto 0);
  signal sBsWord    : std_logic_vector(BS_OUT_W - 1 downto 0);
  signal sBsValid   : std_logic;
  signal sBsBytes   : unsigned(2 downto 0);                             -- [0, 4]
  signal sBsDone    : std_logic;
  signal sFrReady   : std_logic;
  signal sFrWord    : std_logic_vector(OUT_WIDTH - 1 downto 0);
  signal sFrValid   : std_logic;
  signal sFrBytes   : unsigned(OB_W - 1 downto 0);                      -- [0, OUT_WIDTH/8]
  signal sFrLast    : std_logic;
  signal sOutRdy    : std_logic_vector(3 downto 0);
  signal sImgOpen   : std_logic_vector(3 downto 0) := (others => '0');  -- simulation only: lane between start and last word

  function bool2sl (
    b : boolean
  ) return std_logic is
  begin

    if (b) then
      return '1';
    else
      return '0';
    end if;

  end function bool2sl;

begin

  p_input_reg : process (iClk) is
  begin

    if rising_edge(iClk) then
      if (iRst = '1') then
        sWidth  <= unsigned(iImageWidth);
        sHeight <= unsigned(iImageHeight);
        sHold   <= '0';
        assert unsigned(iImageWidth(1 downto 0)) = 0 and unsigned(iImageWidth) >= 32 and
               unsigned(iImageWidth) <= MAX_IMAGE_WIDTH
          report "nl_top: iImageWidth must be a multiple of 4, >= 32 and <= MAX_IMAGE_WIDTH"
          severity warning;
      else
        sHold <= bool2sl(sAlm /= "0000");
      end if;
    end if;

  end process p_input_reg;

  u_cfg : entity work.nl_config
    generic map (
      BITNESS => BITNESS
    )
    port map (
      iClk    => iClk,
      iRst    => iRst,
      iNear   => unsigned(iNear),
      oDone   => sCfgDone,
      oNear   => sNear,
      oRecip  => sRecip,
      oRange  => sRange,
      oRangeD => sRangeD,
      oQbpp   => sQbpp,
      oAinit  => sAinit,
      oT1     => sT1,
      oT2     => sT2,
      oT3     => sT3
    );

  u_fe : entity work.nl_frontend
    generic map (
      BITNESS         => BITNESS,
      MAX_IMAGE_WIDTH => MAX_IMAGE_WIDTH
    )
    port map (
      iClk        => iClk,
      iRst        => iRst,
      -- Ports, not sWidth / sHeight: the front end samples them in reset itself, so a
      -- one-cycle reset takes the new size (sWidth would still hold the old one).
      iWidth      => unsigned(iImageWidth),
      iHeight     => unsigned(iImageHeight),
      iCfgDone    => sCfgDone,
      iNear       => sNear,
      iT1         => sT1,
      iT2         => sT2,
      iT3         => sT3,
      iValid      => iValid,
      iPixel      => iPixel,
      oReady      => oReady,
      iHold       => sHold,
      oClr        => sClr,
      oClrAddr    => sClrAddr,
      oValid      => fValid,
      oLane       => open,
      oFirstPix   => fFirstPix,
      oFirstCol   => fFirstCol,
      oEol        => fEol,
      oIx         => fIx,
      oRb         => fRb,
      oRc         => fRc,
      oQ12Zero    => fQ12Zero,
      oQ3Bound    => fQ3Bound,
      oCandCtx    => fCandCtx,
      oCandSign   => fCandSign,
      oCandC      => fCandC,
      oCandB      => fCandB,
      oCandN      => fCandN,
      iExValid    => xLoopValid,
      iExMode     => xMode,
      iExRx       => xRx,
      iExCtx      => xCtx,
      iExCq       => xCq,
      iExBq       => xBq,
      iExNq       => xNq,
      oExLane     => xLane,
      oExValid    => xValid,
      oExFirstPix => xFirstPix,
      oExLastPix  => xLastPix,
      oExEol      => xEol
    );

  u_loop : entity work.nl_loop
    generic map (
      BITNESS => BITNESS,
      RESET   => RESET
    )
    port map (
      iClk      => iClk,
      iNear     => sNear,
      iMaxVal   => to_unsigned(2 ** BITNESS - 1, BITNESS),
      iRecip    => sRecip,
      iRange    => sRange,
      iRangeD   => sRangeD,
      iValid    => fValid,
      iFirstPix => fFirstPix,
      iFirstCol => fFirstCol,
      iEol      => fEol,
      iIx       => fIx,
      iRb       => fRb,
      iRc       => fRc,
      iQ12Zero  => fQ12Zero,
      iQ3Bound  => fQ3Bound,
      iCandCtx  => fCandCtx,
      iCandSign => fCandSign,
      iCandC    => fCandC,
      iCandB    => fCandB,
      iCandN    => fCandN,
      oValid    => xLoopValid,
      oMode     => xMode,
      oRiType   => xRiType,
      oRx       => xRx,
      oErrval   => xErrval,
      oCtx      => xCtx,
      oCq       => xCq,
      oBq       => xBq,
      oNq       => xNq,
      oBpre     => xBpre,
      oNpre     => xNpre,
      oNRes     => xNRes,
      oPx       => open
    );

  u_be : entity work.nl_backend
    generic map (
      BITNESS => BITNESS,
      RESET   => RESET
    )
    port map (
      iClk      => iClk,
      iRst      => iRst,
      iNear     => sNear,
      iQbpp     => sQbpp,
      iAinit    => sAinit,
      iClr      => sClr,
      iClrAddr  => sClrAddr,
      iValid    => xValid,
      iMode     => xMode,
      iRiType   => xRiType,
      iErrval   => xErrval,
      iCtx      => xCtx,
      iBpre     => xBpre,
      iNpre     => xNpre,
      iLane     => xLane,
      iLastPix  => xLastPix,
      iEol      => xEol,
      oWord     => bWord,
      oValid    => bValid,
      oLen      => bLen,
      oLane     => bLane,
      oLast     => bLast
    );

  sSkidIn <= bLast & std_logic_vector(bLen) & bWord;

  -- Lane turn counter and per-lane start flags
  p_slot : process (iClk) is
  begin

    if rising_edge(iClk) then
      sBsAlmReg <= sBsAlm;
      if (iRst = '1') then
        sSlot      <= (others => '0');
        sStartPend <= (others => '0');
      else
        sSlot <= sSlot + 1;
        for l in 0 to 3 loop
          -- Held until lane l's turn, then handed to the framer
          if (xValid = '1' and xFirstPix = '1' and xLane = l) then
            sStartPend(l) <= '1';
          elsif (sSlot = l) then
            sStartPend(l) <= '0';
          end if;
        end loop;
      end if;
    end if;

  end process p_slot;

  -- Per lane skid FIFO, popped on the lane's turn
  gen_skid : for l in 0 to 3 generate

    signal sSkidWe  : std_logic;
    signal sSkidRdy : std_logic;

  begin

    sSkidWe <= bValid and bool2sl(bLane = l);

    -- psl default clock is rising_edge(iClk);
    -- psl assert always (sSkidWe = '1' -> sSkidRdy = '1') report "nl_top: skid FIFO overflow";

    u_skid : entity work.olo_base_fifo_sync
      generic map (
        Width_g        => SKID_W,
        Depth_g        => SKID_DEPTH,
        AlmFullOn_g    => true,
        AlmFullLevel_g => SKID_ALM
      )
      port map (
        Clk       => iClk,
        Rst       => iRst,
        In_Data   => sSkidIn,
        In_Valid  => sSkidWe,
        In_Ready  => sSkidRdy,
        Out_Data  => sSkidOut(l),
        Out_Valid => sSkidValid(l),
        Out_Ready => sPop(l),
        AlmFull   => sAlm(l)
      );

    sPop(l) <= sSkidValid(l) and not sBsAlmReg(l) and bool2sl(sSlot = l);

  end generate gen_skid;

  sBsIn <= sSkidOut(to_integer(sSlot));

  -- Shared stuffer + framer
  u_bs : entity work.byte_stuffer_nl
    generic map (
      IN_WIDTH            => LIMIT,
      OUT_BYTES_PER_CYCLE => 4,
      BURST_DEPTH         => 16
    )
    port map (
      iClk          => iClk,
      iRst          => iRst,
      iSlot         => sSlot,
      iStall        => '0',
      iWord         => sBsIn(LIMIT - 1 downto 0),
      iWordValid    => sPop(to_integer(sSlot)),
      iWordValidLen => unsigned(sBsIn(LIMIT + LENW - 1 downto LIMIT)),
      iFlush        => sPop(to_integer(sSlot)) and sBsIn(SKID_W - 1),
      oWord         => sBsWord,
      oWordValid    => sBsValid,
      oValidBytes   => sBsBytes,
      iReady        => sFrReady,
      oAlmostFull   => sBsAlm,
      oFlushDone    => sBsDone
    );

  u_framer : entity work.nl_framer
    generic map (
      BITNESS          => BITNESS,
      IN_WIDTH         => BS_OUT_W,
      OUT_WIDTH        => OUT_WIDTH,
      MAX_IMAGE_WIDTH  => MAX_IMAGE_WIDTH / 4,
      MAX_IMAGE_HEIGHT => MAX_IMAGE_HEIGHT
    )
    port map (
      iClk         => iClk,
      iRst         => iRst,
      iStart       => sStartPend(to_integer(sSlot)),
      iNear        => sNear,
      iImageWidth  => resize(shift_right(sWidth, 2), FW_W),
      iImageHeight => resize(sHeight, FH_W),
      iEoi         => sBsDone,
      iWord        => sBsWord,
      iValid       => sBsValid,
      iByteEnable  => sBsBytes,
      oReady       => sFrReady,
      oWord        => sFrWord,
      oValid       => sFrValid,
      oByteEnable  => sFrBytes,
      oLast        => sFrLast,
      iReady       => sOutRdy(to_integer(sSlot))
    );

  -- Per lane output FIFO, written on the lane's turn
  gen_out : for l in 0 to 3 generate

    signal sOutData : std_logic_vector(OUTF_W - 1 downto 0);
    signal sOutBytes : unsigned(OB_W - 1 downto 0);

  begin

    u_out : entity work.olo_base_fifo_sync
      generic map (
        Width_g  => OUTF_W,
        Depth_g  => 4
      )
      port map (
        Clk       => iClk,
        Rst       => iRst,
        In_Data   => sFrLast & std_logic_vector(sFrBytes) & sFrWord,
        In_Valid  => sFrValid and bool2sl(sSlot = l),
        In_Ready  => sOutRdy(l),
        Out_Data  => sOutData,
        Out_Valid => oValid(l),
        Out_Ready => iReady(l)
      );

    oData((l + 1) * OUT_WIDTH - 1 downto l * OUT_WIDTH) <= sOutData(OUT_WIDTH - 1 downto 0);
    sOutBytes <= unsigned(sOutData(OUT_WIDTH + OB_W - 1 downto OUT_WIDTH));
    oLast(l) <= sOutData(OUTF_W - 1);

    gen_keep : for i in 0 to OUT_WIDTH / 8 - 1 generate
      oKeep(l * OUT_WIDTH / 8 + OUT_WIDTH / 8 - 1 - i) <= '1' when sOutBytes > i else
                                                         '0';
    end generate gen_keep;

  end generate gen_out;

  -- Lane hand-off: one start per lane image, framed before the next; the back end
  -- emits words of a lane only inside its image, and the last one once
  assert_proc : process (iClk) is
  begin

    if rising_edge(iClk) then
      if (iRst = '1') then
        sImgOpen <= (others => '0');
      else
        for l in 0 to 3 loop
          if (xValid = '1' and xFirstPix = '1' and xLane = l) then
            assert sStartPend(l) = '0'
              report "nl_top: lane " & integer'image(l) & " restarted before its start was framed" severity failure;
            assert sImgOpen(l) = '0'
              report "nl_top: lane " & integer'image(l) & " restarted before its last word" severity failure;
            sImgOpen(l) <= '1';
          end if;
          if (bValid = '1' and bLane = l) then
            assert sImgOpen(l) = '1' or (xValid = '1' and xFirstPix = '1' and xLane = l)
              report "nl_top: lane " & integer'image(l) & " word outside an image" severity failure;
            if (bLast = '1') then
              sImgOpen(l) <= '0';
            end if;
          end if;
        end loop;
      end if;
    end if;

  end process assert_proc;

  -- Wiring between blocks
  postponed assert bValid /= '1' or is_x(bLen) or bLen <= LIMIT
    report "nl_top: back-end word length past LIMIT" severity error;
  postponed assert sBsValid /= '1' or is_x(sBsBytes) or sBsBytes <= BS_OUT_W / 8
    report "nl_top: stuffer byte count past 4" severity error;
  postponed assert sFrValid /= '1' or is_x(sFrBytes) or sFrBytes <= OUT_WIDTH / 8
    report "nl_top: framer byte count past OUT_WIDTH/8" severity error;

  gen_psl : for l in 0 to 3 generate
  begin

    -- psl default clock is rising_edge(iClk);
    -- psl assert always (((iRst = '0' and oValid(l) = '1' and iReady(l) = '0') -> next (oValid(l) = '1')) abort (iRst = '1')) report "nl_top: oValid must hold until the beat is accepted (AXIS)";
    -- psl assert always (iRst = '1' -> next (oValid(l) = '0')) report "nl_top: reset must clear the output streams";

  end generate gen_psl;

end architecture rtl;
