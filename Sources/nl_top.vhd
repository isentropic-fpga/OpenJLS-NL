-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--

----------------------------------------------------------------------------------
-- Engineer:    Vitor Mendes Camilo
--
-- Module Name: nl_top - Behavioral
--
-- Notes:
--              Near-lossless JPEG-LS encoder, 4 vertical stripes in parallel.
--              The W x H raster input is split into 4 stripes of W/4 x H, each
--              coded as its own JPEG-LS image on its own output stream (lane l
--              is slice l of the output buses, OpenJLS-style AXI-Stream).
--              iNear, iImageWidth and iImageHeight are sampled while iRst = '1'.
--              W must be a multiple of 4 and W/4 >= 8.
--
--              nl_config -> nl_frontend -> nl_loop -> nl_backend -> per lane:
--              skid FIFO -> byte_stuffer -> nl_framer
--
----------------------------------------------------------------------------------

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

  signal sWidth     : unsigned(15 downto 0);
  signal sHeight    : unsigned(15 downto 0);
  signal sHold      : std_logic;

  -- Config
  signal sCfgDone   : std_logic;
  signal sNear      : unsigned(7 downto 0);
  signal sRecip     : unsigned(BITNESS + 10 downto 0);
  signal sRange     : unsigned(BITNESS downto 0);
  signal sRangeD    : unsigned(BITNESS + 1 downto 0);
  signal sQbpp      : unsigned(4 downto 0);
  signal sAinit     : unsigned(BITNESS - 1 downto 0);
  signal sT1        : unsigned(BITNESS - 1 downto 0);
  signal sT2        : unsigned(BITNESS - 1 downto 0);
  signal sT3        : unsigned(BITNESS - 1 downto 0);

  -- Front end -> loop
  signal sClr       : std_logic;
  signal sClrAddr   : unsigned(8 downto 0);
  signal fValid     : std_logic;
  signal fFirstPix  : std_logic;
  signal fFirstCol  : std_logic;
  signal fEol       : std_logic;
  signal fIx        : unsigned(BITNESS - 1 downto 0);
  signal fRb        : unsigned(BITNESS - 1 downto 0);
  signal fRc        : unsigned(BITNESS - 1 downto 0);
  signal fQ12Zero   : std_logic;
  signal fQ3Bound   : t_signed_array(0 to 7)(BITNESS + 1 downto 0);
  signal fCandCtx   : t_unsigned_array(0 to 8)(CO_CTX_WIDTH - 1 downto 0);
  signal fCandSign  : std_logic_vector(0 to 8);
  signal fCandC     : t_signed_array(0 to 8)(CO_C_WIDTH - 1 downto 0);
  signal fCandB     : t_signed_array(0 to 8)(6 downto 0);
  signal fCandN     : t_unsigned_array(0 to 8)(6 downto 0);

  -- Loop exit
  signal xValid     : std_logic;
  signal xMode      : std_logic_vector(1 downto 0);
  signal xRiType    : std_logic;
  signal xRx        : unsigned(BITNESS - 1 downto 0);
  signal xErrval    : signed(BITNESS downto 0);
  signal xCtx       : unsigned(CO_CTX_WIDTH - 1 downto 0);
  signal xCq        : signed(CO_C_WIDTH - 1 downto 0);
  signal xBq        : signed(6 downto 0);
  signal xNq        : unsigned(6 downto 0);
  signal xBpre      : signed(6 downto 0);
  signal xNpre      : unsigned(6 downto 0);
  signal xNRes      : std_logic;
  signal xLane      : unsigned(1 downto 0);
  signal xFirstPix  : std_logic;
  signal xLastPix   : std_logic;
  signal xEol       : std_logic;

  -- Back end
  signal bWord      : std_logic_vector(LIMIT - 1 downto 0);
  signal bValid     : std_logic;
  signal bLen       : unsigned(LENW - 1 downto 0);
  signal bLane      : unsigned(1 downto 0);
  signal bLast      : std_logic;

  signal sSkidIn    : std_logic_vector(SKID_W - 1 downto 0);
  signal sAlm       : std_logic_vector(3 downto 0);

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
      iWidth      => sWidth,
      iHeight     => sHeight,
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
      iExValid    => xValid,
      iExMode     => xMode,
      iExRx       => xRx,
      iExCtx      => xCtx,
      iExCq       => xCq,
      iExBq       => xBq,
      iExNq       => xNq,
      oExLane     => xLane,
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
      oValid    => xValid,
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
      oNRes     => xNRes
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

  -- Per lane output chain -------------------------------------------------------------------------
  gen_lane : for l in 0 to 3 generate

    signal sSkidWe    : std_logic;
    signal sSkidRdy   : std_logic;
    signal sSkidOut   : std_logic_vector(SKID_W - 1 downto 0);
    signal sSkidValid : std_logic;
    signal sPop       : std_logic;
    signal sBsAlm     : std_logic;
    signal sBsAlmReg  : std_logic;
    signal sBsWord    : std_logic_vector(BS_OUT_W - 1 downto 0);
    signal sBsValid   : std_logic;
    signal sBsBytes   : unsigned(2 downto 0);
    signal sBsDone    : std_logic;
    signal sFrReady   : std_logic;
    signal sFrStart   : std_logic;
    signal sFrBytes   : unsigned(log2ceil(OUT_WIDTH / 8 + 1) - 1 downto 0);

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
        Out_Data  => sSkidOut,
        Out_Valid => sSkidValid,
        Out_Ready => sPop,
        AlmFull   => sAlm(l)
      );

    p_bs_alm : process (iClk) is
    begin

      if rising_edge(iClk) then
        sBsAlmReg <= sBsAlm;
      end if;

    end process p_bs_alm;

    sPop <= sSkidValid and not sBsAlmReg;

    u_bs : entity work.byte_stuffer
      generic map (
        IN_WIDTH            => LIMIT,
        OUT_BYTES_PER_CYCLE => 4,
        BURST_DEPTH         => 16
      )
      port map (
        iClk          => iClk,
        iRst          => iRst,
        iStall        => '0',
        iWord         => sSkidOut(LIMIT - 1 downto 0),
        iWordValid    => sPop,
        iWordValidLen => unsigned(sSkidOut(LIMIT + LENW - 1 downto LIMIT)),
        iFlush        => sPop and sSkidOut(SKID_W - 1),
        oWord         => sBsWord,
        oWordValid    => sBsValid,
        oValidBytes   => sBsBytes,
        iReady        => sFrReady,
        oAlmostFull   => sBsAlm,
        oFlushDone    => sBsDone
      );

    sFrStart <= xValid and xFirstPix and bool2sl(xLane = l);

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
        iStart       => sFrStart,
        iNear        => sNear,
        iImageWidth  => resize(shift_right(sWidth, 2), FW_W),
        iImageHeight => resize(sHeight, FH_W),
        iEoi         => sBsDone,
        iWord        => sBsWord,
        iValid       => sBsValid,
        iByteEnable  => sBsBytes,
        oReady       => sFrReady,
        oWord        => oData((l + 1) * OUT_WIDTH - 1 downto l * OUT_WIDTH),
        oValid       => oValid(l),
        oByteEnable  => sFrBytes,
        oLast        => oLast(l),
        iReady       => iReady(l)
      );

    gen_keep : for i in 0 to OUT_WIDTH / 8 - 1 generate
      oKeep(l * OUT_WIDTH / 8 + OUT_WIDTH / 8 - 1 - i) <= '1' when sFrBytes > i else
                                                         '0';
    end generate gen_keep;

  end generate gen_lane;

end architecture rtl;
