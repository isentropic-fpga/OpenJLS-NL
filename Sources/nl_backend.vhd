-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--

----------------------------------------------------------------------------------
-- Engineer:    Vitor Mendes Camilo
--
-- Module Name: nl_backend - Behavioral
--
-- Notes:
--              Entropy coding of the loop exit stream (4 lanes, same-lane pixels
--              at least 4 cycles apart), one shared pipeline:
--                X0  run coding (A.14-A.16) per lane state, A RAM read
--                X2  A RAM data, A/N/Nn update (A.12, A.23), TEMP (A.20)
--                X3  A RAM write, N<<i >= A compares (A.10)
--                X4  k, mapping (A.11, A.21, A.22), glimit
--                X5  limited Golomb (A.11.1)
--                X6  bit packer -> oWord (lane tagged)
--              A RAM per lane: {A, N, Nn}; regular contexts use A only (N comes
--              from the loop), RI contexts live at address RItype.
--
----------------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use work.nl_pkg.all;
  use work.openjls_pkg.CO_J_TABLE;

library work;
  use work.olo_base_pkg_math.log2ceil;

entity nl_backend is
  generic (
    BITNESS : natural range 8 to 16 := 12;
    RESET   : natural               := 64
  );
  port (
    iClk      : in    std_logic;
    iRst      : in    std_logic;
    -- Per image
    iNear     : in    unsigned(7 downto 0);
    iQbpp     : in    unsigned(4 downto 0);
    iAinit    : in    unsigned(BITNESS - 1 downto 0);
    iClr      : in    std_logic;
    iClrAddr  : in    unsigned(8 downto 0);
    -- Loop exit + tags
    iValid    : in    std_logic;
    iMode     : in    std_logic_vector(1 downto 0);
    iRiType   : in    std_logic;
    iErrval   : in    signed(BITNESS downto 0);
    iCtx      : in    unsigned(CO_CTX_WIDTH - 1 downto 0);
    iBpre     : in    signed(6 downto 0);
    iNpre     : in    unsigned(6 downto 0);
    iNRes     : in    std_logic;
    iLane     : in    unsigned(1 downto 0);
    iFirstPix : in    std_logic;
    iLastPix  : in    std_logic;
    iEol      : in    std_logic;
    -- Bit packer output
    oWord     : out   std_logic_vector(2 * (BITNESS + maximum(8, BITNESS)) - 1 downto 0);
    oValid    : out   std_logic;
    oLen      : out   unsigned(log2ceil(2 * (BITNESS + maximum(8, BITNESS)) + 1) - 1 downto 0);
    oLane     : out   unsigned(1 downto 0);
    oLast     : out   std_logic
  );
end entity nl_backend;

architecture behavioral of nl_backend is

  constant LIMIT  : natural := 2 * (BITNESS + maximum(8, BITNESS));
  constant AWID   : natural := BITNESS + 7;
  constant MW     : natural := BITNESS + 2;                               -- mapped error width
  constant UW     : natural := log2ceil(LIMIT);
  constant SUFW   : natural := AWID;
  constant WORD_W : natural := AWID + 14;

  type t_stage is record
    valid  : std_logic;
    mode   : std_logic_vector(1 downto 0);
    ritype : std_logic;
    errval : signed(BITNESS downto 0);
    ctx    : unsigned(CO_CTX_WIDTH - 1 downto 0);
    bpre   : signed(6 downto 0);
    npre   : unsigned(6 downto 0);
    nres   : std_logic;
    lane   : unsigned(1 downto 0);
    first  : std_logic;
    last   : std_logic;
    eol    : std_logic;
    -- run coding
    rawv   : std_logic;
    rawlen : unsigned(4 downto 0);
    rawval : unsigned(15 downto 0);
    j      : natural range 0 to 15;
    -- A.10 / mapping
    n      : unsigned(6 downto 0);
    nn     : unsigned(6 downto 0);
    temp   : unsigned(AWID - 1 downto 0);
    cmp    : std_logic_vector(0 to AWID);
    k      : natural range 0 to AWID;
    merr   : unsigned(MW - 1 downto 0);
    glimit : natural range 0 to LIMIT;
  end record t_stage;

  type t_pipe is array (0 to 5) of t_stage;

  type t_idx_array is array (0 to 3) of natural range 0 to 31;
  type t_cnt_array is array (0 to 3) of unsigned(15 downto 0);
  type t_word_array is array (0 to 3) of std_logic_vector(WORD_W - 1 downto 0);

  signal r        : t_pipe;
  signal sRunIdx  : t_idx_array;
  signal sRunCnt  : t_cnt_array;

  signal sRdAddr  : std_logic_vector(8 downto 0);
  signal sRd      : t_word_array;
  signal sWe      : std_logic_vector(0 to 3);
  signal sWrAddr  : std_logic_vector(8 downto 0);
  signal sWrData  : std_logic_vector(WORD_W - 1 downto 0);
  signal sAWe     : std_logic;
  signal sAWrAddr : unsigned(8 downto 0);
  signal sAWrData : std_logic_vector(WORD_W - 1 downto 0);
  signal sAWrLane : unsigned(1 downto 0);

  signal sGolV    : std_logic;
  signal sUnary   : unsigned(UW - 1 downto 0);
  signal sSufLen  : unsigned(4 downto 0);
  signal sSufVal  : unsigned(SUFW - 1 downto 0);
  signal sRawV    : std_logic;
  signal sRawLen  : unsigned(4 downto 0);
  signal sRawVal  : unsigned(SUFW - 1 downto 0);
  signal sTagLane : unsigned(1 downto 0);
  signal sTagLast : std_logic;

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

    variable vL     : natural range 0 to 3;
    variable vIdx   : natural range 0 to 31;
    variable vCnt   : unsigned(15 downto 0);
    variable vJ     : natural range 0 to 15;
    variable vS     : t_stage;
    variable vWord  : std_logic_vector(WORD_W - 1 downto 0);
    variable vA     : unsigned(AWID - 1 downto 0);
    variable vN     : unsigned(6 downto 0);
    variable vNn    : unsigned(6 downto 0);
    variable vAbs   : unsigned(BITNESS downto 0);
    variable vRes   : boolean;
    variable vK     : natural range 0 to AWID;
    variable vMap   : boolean;
    variable vMerr  : unsigned(MW - 1 downto 0);
    variable vQ     : unsigned(MW - 1 downto 0);
    variable vLim   : integer range -32 to LIMIT;

  begin

    if rising_edge(iClk) then
      -- Input register ------------------------------------------------------------------------
      r(0).valid  <= iValid;
      r(0).mode   <= iMode;
      r(0).ritype <= iRiType;
      r(0).errval <= iErrval;
      r(0).ctx    <= iCtx;
      r(0).bpre   <= iBpre;
      r(0).npre   <= iNpre;
      r(0).nres   <= iNRes;
      r(0).lane   <= iLane;
      r(0).first  <= iFirstPix;
      r(0).last   <= iLastPix;
      r(0).eol    <= iEol;

      -- X0: run coding (A.14-A.16), RUNindex/RUNcnt per lane ----------------------------------
      vS        := r(0);
      vS.rawv   := '0';
      vS.rawlen := to_unsigned(1, 5);
      vS.rawval := to_unsigned(1, 16);
      vL        := to_integer(r(0).lane);
      vIdx      := sRunIdx(vL);
      vCnt      := sRunCnt(vL);
      if (r(0).first = '1') then
        vIdx := 0;
        vCnt := (others => '0');
      end if;
      vJ   := CO_J_TABLE(vIdx);
      vS.j := vJ;
      if (r(0).valid = '1') then
        if (r(0).mode = "01") then
          -- Hit: '1' per full 2^J block; at end of line a partial block also gives '1'
          vCnt := vCnt + 1;
          if (vCnt = shift_left(to_unsigned(1, 16), vJ)) then
            vS.rawv := '1';
            vCnt    := (others => '0');
            vIdx    := minimum(31, vIdx + 1);
          end if;
          if (r(0).eol = '1') then
            vS.rawv := '1';
            vCnt    := (others => '0');
          end if;
        elsif (r(0).mode = "10") then
          -- Interruption: '0' + RUNcnt in J bits, RUNindex decremented after the RI sample
          vS.rawv   := '1';
          vS.rawlen := to_unsigned(vJ + 1, 5);
          vS.rawval := vCnt;
          vCnt      := (others => '0');
          vIdx      := maximum(0, vIdx - 1);
        end if;
        sRunIdx(vL) <= vIdx;
        sRunCnt(vL) <= vCnt;
      end if;
      r(1) <= vS;

      r(2) <= r(1);

      -- X2: A RAM data, update word and TEMP ---------------------------------------------------
      vS    := r(2);
      vWord := sRd(to_integer(r(2).lane));
      vA    := unsigned(vWord(WORD_W - 1 downto 14));
      vNn   := unsigned(vWord(6 downto 0));
      vAbs  := unsigned(abs(r(2).errval));
      if (r(2).mode = "00") then
        vN   := r(2).npre;
        vRes := r(2).nres = '1';
        vA   := vA + vAbs;
      else
        vN   := unsigned(vWord(13 downto 7));
        vRes := vN = RESET;
        vA   := vA + vAbs - ("" & r(2).ritype);
        if (r(2).errval < 0) then
          vNn := vNn + 1;
        end if;
      end if;
      vS.n    := vN;
      vS.nn   := unsigned(vWord(6 downto 0));
      vS.temp := unsigned(vWord(WORD_W - 1 downto 14));
      if (r(2).mode = "10" and r(2).ritype = '1') then
        vS.temp := vS.temp + shift_right(vN, 1);
      end if;
      if (vRes) then
        vA  := shift_right(vA, 1);
        vN  := shift_right(vN, 1);
        vNn := shift_right(vNn, 1);
      end if;
      vN       := vN + 1;
      sAWe     <= r(2).valid and bool2sl(r(2).mode /= "01");
      sAWrLane <= r(2).lane;
      if (r(2).mode = "00") then
        sAWrAddr <= r(2).ctx;
      else
        sAWrAddr <= (0 => r(2).ritype, others => '0');
      end if;
      sAWrData <= std_logic_vector(vA) & std_logic_vector(vN) & std_logic_vector(vNn);
      r(3)     <= vS;

      -- X3: N<<i >= TEMP ------------------------------------------------------------------------
      vS := r(3);
      for i in 0 to AWID loop
        vS.cmp(i) := bool2sl(shift_left(resize(r(3).n, AWID + 8), i) >= resize(r(3).temp, AWID + 8));
      end loop;
      r(4) <= vS;

      -- X4: k, mapped error, glimit -------------------------------------------------------------
      vS := r(4);
      vK := AWID;
      for i in AWID downto 0 loop
        if (r(4).cmp(i) = '1') then
          vK := i;
        end if;
      end loop;
      vS.k := vK;
      vAbs := unsigned(abs(r(4).errval));
      if (r(4).mode = "00") then
        -- A.11: special mapping for NEAR = 0, k = 0, 2B <= -N
        vMap := iNear = 0 and r(4).cmp(0) = '1' and
                resize(r(4).bpre, 9) + resize(r(4).bpre, 9) <= -signed(resize(r(4).npre, 9));
        if (r(4).errval >= 0) then
          vMerr := resize(vAbs & '0', MW);
        else
          vMerr := resize(vAbs & '0', MW) - 1;
        end if;
        if (vMap) then
          if (r(4).errval >= 0) then
            vMerr := vMerr + 1;
          else
            vMerr := vMerr - 1;
          end if;
        end if;
        vS.glimit := LIMIT;
      else
        -- A.21 / A.22
        vMap := (r(4).cmp(0) = '1' and r(4).errval > 0 and (r(4).nn & '0') < ('0' & r(4).n)) or
                (r(4).errval < 0 and ((r(4).nn & '0') >= ('0' & r(4).n) or r(4).cmp(0) = '0'));
        vMerr := resize(vAbs & '0', MW) - ("" & r(4).ritype);
        if (vMap) then
          vMerr := vMerr - 1;
        end if;
        vS.glimit := LIMIT - r(4).j - 1;
      end if;
      vS.merr := vMerr;
      r(5)    <= vS;

      -- X5: limited Golomb (A.11.1) -> bit packer -------------------------------------------------
      vLim := r(5).glimit - to_integer(iQbpp) - 1;
      vQ   := shift_right(r(5).merr, r(5).k);
      sRawV   <= r(5).valid and r(5).rawv;
      sRawLen <= r(5).rawlen;
      sRawVal <= resize(r(5).rawval, SUFW);
      sGolV   <= r(5).valid and bool2sl(r(5).mode /= "01");
      if (vLim < 0 or vQ < vLim) then
        sUnary  <= resize(vQ, UW);
        sSufLen <= to_unsigned(r(5).k, 5);
        sSufVal <= resize(r(5).merr, SUFW);
      else
        sUnary  <= to_unsigned(vLim, UW);
        sSufLen <= resize(iQbpp, 5);
        sSufVal <= resize(r(5).merr - 1, SUFW);
      end if;
      -- Last pixel always gives a word (EOL hit or coded sample)
      sTagLane <= r(5).lane;
      sTagLast <= r(5).valid and r(5).last;
      oLane    <= sTagLane;
      oLast    <= sTagLast;

      if (iRst = '1') then
        for i in 0 to 5 loop
          r(i).valid <= '0';
        end loop;
        sAWe  <= '0';
        sRawV <= '0';
        sGolV <= '0';
      end if;
    end if;

  end process pipe_proc;

  -- A RAMs: clear sweep writes {Ainit, N = 1, Nn = 0}
  sRdAddr <= std_logic_vector(r(0).ctx) when r(0).mode = "00" else
             (0 => r(0).ritype, others => '0');
  sWrAddr <= std_logic_vector(iClrAddr) when iClr = '1' else
             std_logic_vector(sAWrAddr);
  sWrData <= std_logic_vector(resize(iAinit, AWID)) & "0000001" & "0000000" when iClr = '1' else
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
      iRawValid    => sRawV,
      iRawLen      => sRawLen,
      iRawVal      => sRawVal,
      iGolombValid => sGolV,
      iUnaryZeros  => sUnary,
      iSuffixLen   => sSufLen,
      iSuffixVal   => sSufVal,
      oWord        => oWord,
      oWordValid   => oValid,
      oValidLen    => oLen
    );

end architecture behavioral;
