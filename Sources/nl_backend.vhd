-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- nl_backend: entropy coding of the loop exit stream, one OpenJLS pipeline shared by 4 lanes
--
-- X0 run coding and A RAM read, X2 A / N / Nn update and TEMP, X3 k, X4 mapping,
-- X5 limited Golomb, X6 bit packer. A RAM per lane holds {A, N, Nn}: regular contexts use only A
-- (B, N come from the loop), RI contexts sit at address RItype.
-- A lane line is <= 16383 pixels (width <= 65532), so RUNindex <= 30.

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use work.nl_pkg.all;

library work;
  use work.olo_base_pkg_math.log2ceil;

entity nl_backend is
  generic (
    BITNESS : natural range 8 to 16 := 12;
    RESET   : natural               := 64
  );
  port (
    iClk     : in    std_logic;
    iRst     : in    std_logic;
    -- Per image
    iNear    : in    unsigned(7 downto 0);
    iQbpp    : in    unsigned(4 downto 0);
    iAinit   : in    unsigned(BITNESS - 1 downto 0);
    iClr     : in    std_logic;
    iClrAddr : in    unsigned(8 downto 0);
    -- Loop exit + tags
    iValid   : in    std_logic;
    iMode    : in    std_logic_vector(1 downto 0);
    iRiType  : in    std_logic;
    iErrval  : in    signed(BITNESS downto 0);
    iCtx     : in    unsigned(CO_CTX_WIDTH - 1 downto 0);
    iBpre    : in    signed(6 downto 0);
    iNpre    : in    unsigned(6 downto 0);
    iLane    : in    unsigned(1 downto 0);
    iLastPix : in    std_logic;
    iEol     : in    std_logic;
    -- Bit packer output
    oWord    : out   std_logic_vector(2 * (BITNESS + maximum(8, BITNESS)) - 1 downto 0);
    oValid   : out   std_logic;
    oLen     : out   unsigned(log2ceil(2 * (BITNESS + maximum(8, BITNESS)) + 1) - 1 downto 0);
    oLane    : out   unsigned(1 downto 0);
    oLast    : out   std_logic
  );
end entity nl_backend;

architecture behavioral of nl_backend is

  constant LIMIT   : natural := 2 * (BITNESS + maximum(8, BITNESS));
  constant AWID    : natural := BITNESS + 7;
  constant NWID    : natural := 7;
  constant KWID    : natural := log2ceil(AWID + 1);
  constant MW      : natural := BITNESS + 2;                              -- mapped error width
  constant UW      : natural := log2ceil(LIMIT);
  constant SUFW    : natural := AWID;
  constant WORD_W  : natural := AWID + 2 * NWID;
  constant RUN_CNT : natural := 16;

  type t_stage is record
    valid  : std_logic;
    mode   : std_logic_vector(1 downto 0);
    ritype : std_logic;
    errval : signed(BITNESS downto 0);
    ctx    : unsigned(CO_CTX_WIDTH - 1 downto 0);
    bpre   : signed(6 downto 0);
    npre   : unsigned(NWID - 1 downto 0);
    lane   : unsigned(1 downto 0);
    last   : std_logic;
    eol    : std_logic;
    -- run coding
    rawv   : std_logic;
    rawlen : unsigned(4 downto 0);
    rawval : unsigned(RUN_CNT - 1 downto 0);
    runidx : unsigned(4 downto 0);
    -- A.10 / mapping
    n      : unsigned(NWID - 1 downto 0);
    nn     : unsigned(NWID - 1 downto 0);
    temp   : unsigned(AWID - 1 downto 0);
    k      : unsigned(KWID - 1 downto 0);
    merr   : unsigned(MW - 1 downto 0);
  end record t_stage;

  type t_pipe is array (0 to 5) of t_stage;

  type t_cnt_array is array (0 to 3) of unsigned(RUN_CNT - 1 downto 0);
  type t_bound_array is array (0 to 3) of unsigned(RUN_CNT downto 0);
  type t_idx_array is array (0 to 3) of unsigned(4 downto 0);
  type t_word_array is array (0 to 3) of std_logic_vector(WORD_W - 1 downto 0);

  signal r         : t_pipe;

  -- X0 run coding
  signal sL         : natural range 0 to 3;
  signal sRun       : std_logic;
  signal sHit       : std_logic;
  signal sCont      : std_logic;
  signal sEoi       : std_logic;
  signal sRunCnt    : t_cnt_array;                                        -- per lane: A.14 RUNcnt [0, 16383]
  signal sRunIdx    : t_idx_array;                                        --   RUNindex [0, 30] (line <= 16383)
  signal sBound     : t_bound_array;                                      --   A.15 next boundary
  signal sCntIn     : unsigned(RUN_CNT - 1 downto 0);                   -- [0, 16383]
  signal sIdxNext   : unsigned(4 downto 0);                             -- [0, 30]
  signal sBoundNext : unsigned(RUN_CNT downto 0);                       -- A.15 boundary, [1, 2^16]
  signal sRawV      : std_logic;
  signal sGolV      : std_logic := '0';                                   -- a11_1 iValid, registered with r(5)
  signal sRawLen    : unsigned(4 downto 0);                             -- [1, 16]
  signal sRawVal    : unsigned(RUN_CNT - 1 downto 0);                   -- full type
  signal sRunIdxOut : unsigned(4 downto 0);                             -- [0, 30]

  -- X2 context update
  signal sRdAddr   : std_logic_vector(8 downto 0);
  signal sRd       : t_word_array;
  signal sWord     : std_logic_vector(WORD_W - 1 downto 0);
  signal sAq       : unsigned(AWID - 1 downto 0);                       -- [0, RESET*2^(BITNESS-1)]
  signal sRiErr    : signed(BITNESS downto 0);                          -- A.23 inputs, one snapshot
  signal sRiRt     : std_logic;
  signal sRiAqIn   : unsigned(AWID - 1 downto 0);                       -- [0, RESET*2^(BITNESS-1)]
  signal sRiNqIn   : unsigned(NWID - 1 downto 0);                       -- [1, RESET]
  signal sRiNnIn   : unsigned(NWID - 1 downto 0);                       -- [0, RESET-1]
  signal sNq       : unsigned(NWID - 1 downto 0);                       -- [1, RESET]
  signal sNn       : unsigned(NWID - 1 downto 0);                       -- [0, RESET-1]
  signal sN2       : unsigned(NWID - 1 downto 0);                       -- [1, RESET] on valid tokens
  signal sRiType   : std_logic;                                         -- 0 outside valid RI tokens
  signal sTemp     : unsigned(AWID - 1 downto 0);                       -- A.12 temp, [0, 2*RESET*2^(BITNESS-1)]
  signal sRegAq    : unsigned(AWID - 1 downto 0);                       -- [0, RESET*2^(BITNESS-1)]
  signal sRegNq    : unsigned(NWID - 1 downto 0);                       -- [1, RESET]
  signal sRegBq    : signed(6 downto 0);                                -- [1-N, 0]
  signal sRiAq     : unsigned(AWID - 1 downto 0);                       -- [0, RESET*2^(BITNESS-1)]
  signal sRiNq     : unsigned(NWID - 1 downto 0);                       -- [1, RESET]
  signal sRiNn     : unsigned(NWID - 1 downto 0);                       -- [0, RESET-1]
  signal sWe       : std_logic_vector(0 to 3);
  signal sWrAddr   : std_logic_vector(8 downto 0);
  signal sWrData   : std_logic_vector(WORD_W - 1 downto 0);
  signal sAWe      : std_logic;
  signal sAWrAddr  : unsigned(8 downto 0);
  signal sAWrData  : std_logic_vector(WORD_W - 1 downto 0);
  signal sAWrLane  : unsigned(1 downto 0);

  -- X3 .. X5
  signal sK        : unsigned(KWID - 1 downto 0);                       -- [0, BITNESS] on valid tokens
  signal sBqMap    : signed(6 downto 0);                                -- [1-N, 0] (NEAR = 0), else 0
  signal sRegMerr  : unsigned(MW - 1 downto 0);                         -- [0, 2^BITNESS - 1]
  signal sRiMap    : std_logic;
  signal sRiMerr   : unsigned(MW - 1 downto 0);                         -- [0, 2^BITNESS - 1]
  signal sUnary    : unsigned(UW - 1 downto 0);                         -- [0, LIMIT-2]
  signal sSufLen   : unsigned(4 downto 0);                              -- [0, max(k, QBPP)]
  signal sSufVal   : unsigned(SUFW - 1 downto 0);                       -- full type

  -- X6 bit packer inputs
  signal sBpRawV   : std_logic;
  signal sBpRawLen : unsigned(4 downto 0);                              -- [0, 16]
  signal sBpRawVal : unsigned(SUFW - 1 downto 0);                       -- full type
  signal sBpGolV   : std_logic;
  signal sBpUnary  : unsigned(UW - 1 downto 0);                         -- [0, LIMIT-2]
  signal sBpSufLen : unsigned(4 downto 0);                              -- [0, max(k, QBPP)]
  signal sBpSufVal : unsigned(SUFW - 1 downto 0);                       -- full type
  signal sTagLane  : unsigned(1 downto 0);
  signal sTagLast  : std_logic;

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

  pipe_proc : process (iClk) is

    variable vS : t_stage;

  begin

    if rising_edge(iClk) then
      -- Input register
      r(0).valid  <= iValid;
      r(0).mode   <= iMode;
      r(0).ritype <= iRiType;
      r(0).errval <= iErrval;
      r(0).ctx    <= iCtx;
      r(0).bpre   <= iBpre;
      r(0).npre   <= iNpre;
      r(0).lane   <= iLane;
      r(0).last   <= iLastPix;
      r(0).eol    <= iEol;

      -- X0: run coding tokens of the current lane
      vS        := r(0);
      vS.rawv   := sRawV;
      vS.rawlen := sRawLen;
      vS.rawval := sRawVal;
      vS.runidx := sRunIdxOut;
      r(1)      <= vS;

      r(2) <= r(1);

      -- X2: context update (A.12 regular A, A.23 RI), TEMP (A.20)
      vS       := r(2);
      vS.n     := sN2;
      vS.nn    := sNn;
      vS.temp  := sTemp;
      sAWe     <= r(2).valid and bool2sl(r(2).mode /= "01");
      sAWrLane <= r(2).lane;
      if (r(2).mode = "00") then
        sAWrAddr <= r(2).ctx;
        sAWrData <= std_logic_vector(sRegAq) & std_logic_vector(sRegNq) & std_logic_vector(sNn);
      else
        sAWrAddr <= (0 => r(2).ritype, others => '0');
        sAWrData <= std_logic_vector(sRiAq) & std_logic_vector(sRiNq) & std_logic_vector(sRiNn);
      end if;
      r(3) <= vS;

      -- X3: k (A.10)
      vS   := r(3);
      vS.k := sK;
      r(4) <= vS;

      -- X4: mapped error (A.11 regular, A.21/A.22 RI)
      vS := r(4);
      if (r(4).mode = "00") then
        vS.merr := sRegMerr;
      else
        vS.merr := sRiMerr;
      end if;
      r(5) <= vS;
      sGolV <= vS.valid and bool2sl(vS.mode /= "01");                     -- same delta as r(5)

      -- X5: limited Golomb (A.11.1) -> bit packer
      sBpRawV   <= r(5).valid and r(5).rawv;
      sBpRawLen <= r(5).rawlen;
      sBpRawVal <= resize(r(5).rawval, SUFW);
      sBpGolV   <= r(5).valid and bool2sl(r(5).mode /= "01");
      sBpUnary  <= sUnary;
      sBpSufLen <= sSufLen;
      sBpSufVal <= sSufVal;
      -- Last pixel always gives a word (EOL hit or coded sample)
      sTagLane <= r(5).lane;
      sTagLast <= r(5).valid and r(5).last;
      oLane    <= sTagLane;
      oLast    <= sTagLast;

      if (iRst = '1') then
        for i in 0 to 5 loop
          r(i).valid <= '0';
        end loop;
        sAWe    <= '0';
        sBpRawV <= '0';
        sBpGolV <= '0';
        sGolV   <= '0';
      end if;
    end if;

  end process pipe_proc;

  -- X0: one A.15/A.16 for all lanes, run state per lane. RUNcnt is the A.14
  -- count; a lane's state clears after its last pixel.
  sL     <= to_integer(r(0).lane);
  sRun   <= r(0).valid and bool2sl(r(0).mode = "01" or r(0).mode = "10");
  sHit   <= bool2sl(r(0).mode = "01");
  sCont  <= sHit and not r(0).eol;
  sEoi   <= r(0).valid and r(0).last;
  sCntIn <= sRunCnt(sL) + 1 when sHit = '1' else
            sRunCnt(sL);

  p_run_state : process (iClk) is
  begin

    if rising_edge(iClk) then
      if (sEoi = '1') then
        sRunCnt(sL) <= (others => '0');
        sRunIdx(sL) <= (others => '0');
        sBound(sL)  <= to_unsigned(1, RUN_CNT + 1);
      elsif (sRun = '1') then
        if (sCont = '1') then
          sRunCnt(sL) <= sCntIn;
        else
          sRunCnt(sL) <= (others => '0');
        end if;
        sRunIdx(sL) <= sIdxNext;
        sBound(sL)  <= sBoundNext;
      end if;
      if (iRst = '1') then
        sRunCnt <= (others => (others => '0'));
        sRunIdx <= (others => (others => '0'));
        sBound  <= (others => to_unsigned(1, RUN_CNT + 1));
      end if;
    end if;

  end process p_run_state;

  u_a15_16 : entity work.a15_a16_nl_encode_run(behavioral)
    generic map (
      BITNESS        => BITNESS,
      RUN_CNT_WIDTH  => RUN_CNT
    )
    port map (
      iRst           => iRst,
      iRunIndex      => sRunIdx(sL),
      iNextBound     => sBound(sL),
      iInRun         => '0',
      oRunIndexNext  => sIdxNext,
      oNextBoundNext => sBoundNext,
      iRunCnt        => sCntIn,
      iRunHit        => sHit,
      iRunContinue   => sCont,
      iModeIsRun     => sRun,
      iIx            => (others => '0'),
      iRa            => (others => '0'),
      iRb            => (others => '0'),
      oRawValid      => sRawV,
      oRawSuffixLen  => sRawLen,
      oRawSuffixVal  => sRawVal,
      oRiValid       => open,
      oRiIx          => open,
      oRiRa          => open,
      oRiRb          => open,
      oRiRunIndex    => sRunIdxOut,
      oInRunNext     => open
    );

  -- X2: A RAM word {A, N, Nn}; regular N is the loop's pre-update N
  sWord   <= sRd(to_integer(r(2).lane));
  sAq     <= unsigned(sWord(WORD_W - 1 downto 2 * NWID));
  sNq     <= unsigned(sWord(2 * NWID - 1 downto NWID));
  sNn     <= unsigned(sWord(NWID - 1 downto 0));
  sN2     <= r(2).npre when r(2).mode = "00" else
             sNq;
  sRiType <= r(2).valid and r(2).ritype and bool2sl(r(2).mode = "10");

  -- A.23 inputs come from one process so they settle in the same delta; mixed old / new values
  -- could take A + |Errval| - RItype to -1. RItype is dropped when A = Errval = 0, which no real
  -- RItype = 1 token has.
  p_ri_in : process (all) is
  begin

    sRiErr  <= r(2).errval;
    sRiAqIn <= sAq;
    sRiNqIn <= sNq;
    sRiNnIn <= sNn;
    sRiRt   <= sRiType;

    if (sAq = 0 and r(2).errval = 0) then
      sRiRt <= '0';
    end if;

  end process p_ri_in;


  u_a12 : entity work.a12_variables_update(rtl)
    generic map (
      ERROR_WIDTH => BITNESS + 1,
      A_WIDTH     => AWID,
      B_WIDTH     => 7,
      N_WIDTH     => NWID,
      RESET       => RESET
    )
    port map (
      iErrorVal   => r(2).errval,
      iAq         => sAq,
      iBq         => r(2).bpre,
      iNq         => r(2).npre,
      oAq         => sRegAq,
      oBq         => sRegBq,
      oNq         => sRegNq
    );

  u_a23 : entity work.a23_run_interruption_update(behavioral)
    generic map (
      A_WIDTH     => AWID,
      N_WIDTH     => NWID,
      NN_WIDTH    => NWID,
      ERROR_WIDTH => BITNESS + 1,
      RESET       => RESET
    )
    port map (
      iErrVal     => sRiErr,
      iRiType     => sRiRt,
      iAq         => sRiAqIn,
      iNq         => sRiNqIn,
      iNn         => sRiNnIn,
      oAq         => sRiAq,
      oNq         => sRiNq,
      oNn         => sRiNn
    );

  u_a20 : entity work.a20_compute_temp(behavioral)
    generic map (
      A_WIDTH => AWID,
      N_WIDTH => NWID
    )
    port map (
      iRItype => sRiType,
      iAq     => sAq,
      iNq     => sN2,
      oTemp   => sTemp
    );

  -- X3
  u_a10 : entity work.a10_compute_k(behavioral)
    generic map (
      A_WIDTH => AWID,
      K_WIDTH => KWID,
      N_WIDTH => NWID
    )
    port map (
      iNq     => r(3).n,
      iAq     => r(3).temp,
      oK      => sK
    );

  -- X4: the A.11 special mapping is lossless only; B = 0 disables it (2B <= -N needs N <= 0)
  sBqMap <= r(4).bpre when iNear = 0 else
            (others => '0');

  u_a11 : entity work.a11_error_mapping(behavioral)
    generic map (
      N_WIDTH                => NWID,
      B_WIDTH                => 7,
      K_WIDTH                => KWID,
      ERROR_WIDTH            => BITNESS + 1,
      MAPPED_ERROR_VAL_WIDTH => MW
    )
    port map (
      iK                     => r(4).k,
      iBq                    => sBqMap,
      iNq                    => r(4).n,
      iErrorVal              => r(4).errval,
      oMappedErrorVal        => sRegMerr
    );

  u_a21 : entity work.a21_compute_map(behavioral)
    generic map (
      K_WIDTH     => KWID,
      N_WIDTH     => NWID,
      ERROR_WIDTH => BITNESS + 1
    )
    port map (
      iK          => r(4).k,
      iErrval     => r(4).errval,
      iNn         => r(4).nn,
      iNq         => r(4).n,
      oMap        => sRiMap
    );

  u_a22 : entity work.a22_errval_mapping(behavioral)
    generic map (
      ERROR_WIDTH         => BITNESS + 1,
      MAPPED_ERRVAL_WIDTH => MW
    )
    port map (
      iErrval             => r(4).errval,
      iRiType             => r(4).ritype,
      iMap                => sRiMap,
      oEmErrVal           => sRiMerr
    );

  -- X5
  u_a11_1 : entity work.a11_1_nl_golomb_encoder(behavioral)
    generic map (
      K_WIDTH                => KWID,
      LIMIT                  => LIMIT,
      UNARY_WIDTH            => UW,
      SUFFIX_WIDTH           => SUFW,
      SUFFIXLEN_WIDTH        => 5,
      MAPPED_ERROR_VAL_WIDTH => MW
    )
    port map (
      iValid                 => sGolV,
      iK                     => r(5).k,
      iQbpp                  => iQbpp,
      iMappedErrorVal        => r(5).merr,
      iRiMode                => bool2sl(r(5).mode = "10"),
      iRunIndex              => r(5).runidx,
      oUnaryZeros            => sUnary,
      oSuffixLen             => sSufLen,
      oSuffixVal             => sSufVal
    );

  -- A RAMs: clear sweep writes {Ainit, N = 1, Nn = 0}
  sRdAddr <= std_logic_vector(r(0).ctx) when r(0).mode = "00" else
             (0 => r(0).ritype, others => '0');
  sWrAddr <= std_logic_vector(iClrAddr) when iClr = '1' else
             std_logic_vector(sAWrAddr);
  sWrData <= std_logic_vector(resize(iAinit, AWID)) & std_logic_vector(to_unsigned(1, NWID)) &
             std_logic_vector(to_unsigned(0, NWID)) when iClr = '1' else
             sAWrData;

  gen_aram : for l in 0 to 3 generate

    sWe(l) <= iClr or (sAWe and bool2sl(sAWrLane = l));

    u_aram : entity work.olo_base_ram_sdp
      generic map (
        Depth_g     => 512,
        Width_g     => WORD_W,
        RdLatency_g => 2
      )
      port map (
        Clk     => iClk,
        Wr_Addr => sWrAddr,
        Wr_Ena  => sWe(l),
        Wr_Data => sWrData,
        Rd_Addr => sRdAddr,
        Rd_Data => sRd(l)
      );

  end generate gen_aram;

  u_packer : entity work.a11_2_bit_packer
    generic map (
      LIMIT           => LIMIT,
      UNARY_WIDTH     => UW,
      SUFFIX_WIDTH    => SUFW,
      SUFFIXLEN_WIDTH => 5,
      OUT_WIDTH       => LIMIT
    )
    port map (
      iClk         => iClk,
      iRst         => iRst,
      iStall       => '0',
      iRawValid    => sBpRawV,
      iRawLen      => sBpRawLen,
      iRawVal      => sBpRawVal,
      iGolombValid => sBpGolV,
      iUnaryZeros  => sBpUnary,
      iSuffixLen   => sBpSufLen,
      iSuffixVal   => sBpSufVal,
      oWord        => oWord,
      oWordValid   => oValid,
      oValidLen    => oLen
    );

  -- Input contract on valid tokens, A RAM ranges on the tokens that use them
  assert_proc : process (iClk) is
    constant EMAX : integer := 2 ** (BITNESS - 1);
    variable vA   : integer;
  begin
    if rising_edge(iClk) then
      if (iRst = '0' and iValid = '1') then
        assert iMode /= "11" report "nl_backend: token mode 11" severity failure;
        assert iClr = '0' report "nl_backend: token during the clear sweep" severity failure;
        assert not ((r(0).valid = '1' and r(0).lane = iLane) or (r(1).valid = '1' and r(1).lane = iLane) or
                    (r(2).valid = '1' and r(2).lane = iLane))
          report "nl_backend: same-lane tokens less than 4 cycles apart" severity failure;
        assert to_integer(iErrval) >= -EMAX and to_integer(iErrval) < EMAX
          report "nl_backend: Errval outside [-2^(BITNESS-1), 2^(BITNESS-1)-1]" severity failure;
        if (iMode = "00") then
          assert iCtx >= 4 and iCtx <= 368 report "nl_backend: regular ctx outside [4, 368]" severity failure;
          assert iNpre >= 1 and iNpre <= RESET report "nl_backend: N outside [1, RESET]" severity failure;
          assert to_integer(iBpre) >= 1 - to_integer(iNpre) and iBpre <= 0
            report "nl_backend: B outside [1-N, 0]" severity failure;
        end if;
        if (iMode = "10" and iRiType = '1') then
          assert iErrval /= 0 report "nl_backend: RItype 1 with Errval 0" severity failure;
        end if;
      end if;
      if (iRst = '0' and r(0).valid = '1') then
        assert r(0).mode /= "00" or sRunCnt(sL) = 0
          report "nl_backend: regular token inside a run" severity failure;
        assert sRunCnt(sL) <= 16383 report "nl_backend: run longer than a 16383-pixel lane line" severity failure;
      end if;
      if (iRst = '0' and r(2).valid = '1' and r(2).mode /= "01") then
        vA := to_integer(sAq);
        assert vA <= RESET * EMAX and sNq >= 1 and sNq <= RESET and sNn < RESET
          report "nl_backend: A RAM word outside A [0, RESET*2^(BITNESS-1)], N [1, RESET], Nn [0, RESET-1]"
          severity failure;
      end if;
      if (iRst = '0' and r(3).valid = '1' and r(3).mode /= "01") then
        assert sK <= BITNESS report "nl_backend: k above BITNESS" severity failure;
      end if;
    end if;
  end process assert_proc;

  postponed assert r(2).valid /= '1' or is_x(sN2) or (sN2 >= 1 and sN2 <= RESET)
    report "nl_backend: X2 N outside [1, RESET]" severity error;
  postponed assert is_x(sCntIn) or sCntIn <= 16383
    report "nl_backend: RUNcnt above 16383" severity error;

end architecture behavioral;
