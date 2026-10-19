-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- tb_nl_backend_osvvm: nl_backend against a per-lane transcription of T.87 A.10-A.23
--
-- Each session resets with a random NEAR and runs 1..3 images: a clear sweep, then legal per-lane
-- token sequences interleaved at random, same-lane tokens >= 4 cycles apart. The TB plays the loop
-- (N, B of regular contexts). RI uses RUNindex before the decrement (CharLS order). A long-run
-- session (six 16383-pixel lines of hits) takes RUNindex to its reachable max, 30, then plans the
-- lines that start there for the A.15 bound and raw-suffix maxima. Each word is one token: length,
-- lane, last flag and bits are checked in order. Steered sessions take A, Nn and EMErrval to their
-- extremes: quiet (minimal errors, A halves to 0, then random errors), loud (Errval = eLo at
-- NEAR 0: A to Ainit + (RESET-1)*2^(BITNESS-1), Nn to RESET-1) and spike (quiet, with eLo on RI
-- tokens at k = 0: EMErrval = 2^BITNESS).
--
-- Guidelines:
--   G2.Clamp    G2.MinMax node (N, B, A, Nn, RUNindex 30), G4.Coding corners (halving). RUNindex 31
--               unreachable: its J = 14 segment needs 16384 pixels, a lane line is <= 16383
--   G3.Stim     G2.MinMax node (Errval at -2^(BITNESS-1), B at 1-N), G4.Coding corners (map)
--   G4.Run      G4.Coding corners, G2.MinMax node (RUNindex max)
--   G4.Branch   G4.Coding corners (map bit x RItype)
--   G4.Config   G2.MinMax node (NEAR 0 / max)
--   G4 other    N/A: Edge belongs to tb_nl_frontend_osvvm

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use work.nl_pkg.all;

library osvvm;
  context osvvm.OsvvmContext;

library tb_support;
  use tb_support.tb_support_pkg.all;

entity tb_nl_backend_osvvm is
  generic (
    BITNESS : natural range 8 to 16 := 12
  );
end entity tb_nl_backend_osvvm;

architecture sim of tb_nl_backend_osvvm is

  constant MAXVAL     : natural := 2 ** BITNESS - 1;
  constant NEAR_MAX   : natural := minimum(255, MAXVAL / 2);
  constant RESET_N    : natural := 64;
  constant LIMIT      : natural := 2 * (BITNESS + maximum(8, BITNESS));
  constant WW         : natural := LIMIT;
  constant LW         : natural := 6 + boolean'pos(LIMIT >= 64);          -- ceil(log2(LIMIT + 1)), LIMIT in {32, 48, 64}
  constant CLK_PERIOD : time    := CLK_PERIOD_DEFAULT;

  constant N_SESS     : natural := 10;
  constant N_RST      : natural := 20;
  constant LONG_WS    : natural := 16383;                                -- max lane line (65532 / 4)
  constant LONG_H     : natural := 6;                                    -- RUNindex 30, then planned lines
  constant AINIT_MAX  : natural := maximum(2, (MAXVAL + 1 + 32) / 64);
  constant A_MAX      : natural := AINIT_MAX + (RESET_N - 1) * 2 ** (BITNESS - 1);
  constant STEER_WS   : natural := 64;
  constant STEER_H    : natural := 64;

  -- steered sessions
  constant S_NONE     : natural := 0;
  constant S_QUIET    : natural := 1;                                    -- minimal errors: A falls to 0
  constant S_LOUD     : natural := 2;                                    -- Errval = eLo: A and Nn to max
  constant S_SPIKE    : natural := 3;                                    -- S_QUIET with eLo at k = 0: EMErrval 2^B

  -- requirement mask bits of a token
  constant M_A10      : natural := 1;
  constant M_A11      : natural := 2;
  constant M_A111     : natural := 4;
  constant M_A12      : natural := 8;
  constant M_A15      : natural := 16;
  constant M_A16      : natural := 32;
  constant M_A20      : natural := 64;
  constant M_A21      : natural := 128;
  constant M_A22      : natural := 256;
  constant M_A23      : natural := 512;

  constant RP_IDLE    : integer := 0;
  constant RP_CLEAR   : integer := 1;
  constant RP_BUSY    : integer := 2;
  constant RP_RUN     : integer := 3;
  constant RP_DRAIN   : integer := 4;

  -- token kinds
  constant K_REG      : integer := 0;
  constant K_HIT      : integer := 1;
  constant K_HITEOL   : integer := 2;
  constant K_RI0      : integer := 3;
  constant K_RI1      : integer := 4;

  -- covCode bins
  constant C_ESC_REG  : integer := 0;
  constant C_ESC_RI   : integer := 1;
  constant C_SPECIAL  : integer := 2;
  constant C_MAP0_T0  : integer := 3;
  constant C_MAP1_T0  : integer := 4;
  constant C_MAP0_T1  : integer := 5;
  constant C_MAP1_T1  : integer := 6;
  constant C_HALF_REG : integer := 7;
  constant C_HALF_RI  : integer := 8;
  constant C_SEG      : integer := 9;
  constant C_EOL1     : integer := 10;
  constant C_EOL0     : integer := 11;
  constant C_RI_RC0   : integer := 13;
  constant C_RI_RCP   : integer := 14;

  -- covRange bins
  constant R_NEAR0    : integer := 0;
  constant R_NEARMAX  : integer := 1;
  constant R_AINIT2   : integer := 2;
  constant R_AINITMAX : integer := 3;
  constant R_EMIN     : integer := 4;
  constant R_EMAX     : integer := 5;
  constant R_RIEMIN   : integer := 6;
  constant R_RIEMAX   : integer := 7;
  constant R_N1       : integer := 8;
  constant R_NRESET   : integer := 9;
  constant R_BMIN     : integer := 10;
  constant R_B0       : integer := 11;
  constant R_CTX4     : integer := 12;
  constant R_CTX368   : integer := 13;
  constant R_K0       : integer := 14;
  constant R_KMAX     : integer := 15;
  constant R_LIMIT    : integer := 16;
  constant R_RIDX31   : integer := 17;
  constant R_RCMAX    : integer := 18;
  constant R_AREG0    : integer := 19;
  constant R_AREGMAX  : integer := 20;
  constant R_ARI0_T0  : integer := 21;
  constant R_ARI0_T1  : integer := 22;
  constant R_ARIMAX   : integer := 23;
  constant R_NN0      : integer := 24;
  constant R_NNMAX    : integer := 25;
  constant R_CNTMAX   : integer := 26;
  constant R_RIIDX0   : integer := 27;
  constant R_RIIDX30  : integer := 28;
  constant R_BND1     : integer := 29;
  constant R_BNDMAX   : integer := 30;
  constant R_RAWMAX   : integer := 31;
  constant R_NREG2    : integer := 32;
  constant R_NREGMAX  : integer := 33;
  constant R_NRI2     : integer := 34;
  constant R_NRIMAX   : integer := 35;
  constant R_MREG0    : integer := 36;
  constant R_MREGMAX  : integer := 37;
  constant R_MRI0     : integer := 38;
  constant R_MRIMAX   : integer := 39;
  constant R_UN0      : integer := 40;
  constant R_UNMAX    : integer := 41;
  constant R_SL0      : integer := 42;
  constant R_SLMAX    : integer := 43;
  constant R_SV0      : integer := 44;
  constant R_SVMAX    : integer := 45;
  constant R_NRI1     : integer := 46;
  constant R_NRIRESET : integer := 47;

  type t_int32 is array (0 to 31) of natural;

  constant J : t_int32 := (0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3,
                           4, 4, 5, 5, 6, 6, 7, 7, 8, 9, 10, 11, 12, 13, 14, 15);

  signal clk      : std_logic := '0';
  signal rst      : std_logic := '1';
  signal iNear    : unsigned(7 downto 0) := (others => '0');
  signal iQbpp    : unsigned(4 downto 0) := to_unsigned(BITNESS, 5);
  signal iAinit   : unsigned(BITNESS - 1 downto 0) := to_unsigned(2, BITNESS);
  signal iClr     : std_logic := '0';
  signal iClrAddr : unsigned(8 downto 0) := (others => '0');
  signal iValid   : std_logic := '0';
  signal iMode    : std_logic_vector(1 downto 0) := "00";
  signal iRiType  : std_logic := '0';
  signal iErrval  : signed(BITNESS downto 0) := (others => '0');
  signal iCtx     : unsigned(CO_CTX_WIDTH - 1 downto 0) := to_unsigned(4, CO_CTX_WIDTH);
  signal iBpre    : signed(6 downto 0) := (others => '0');
  signal iNpre    : unsigned(6 downto 0) := to_unsigned(1, 7);
  signal iLane    : unsigned(1 downto 0) := (others => '0');
  signal iLastPix : std_logic := '0';
  signal iEol     : std_logic := '0';
  signal oWord    : std_logic_vector(WW - 1 downto 0);
  signal oValid   : std_logic;
  signal oLen     : unsigned(LW - 1 downto 0);
  signal oLane    : unsigned(1 downto 0);
  signal oLast    : std_logic;

  function sl (
    b : boolean
  ) return std_logic is
  begin
    if (b) then
      return '1';
    else
      return '0';
    end if;
  end function sl;

begin

  clk <= not clk after CLK_PERIOD / 2;

  dut : entity work.nl_backend(behavioral)
    generic map (
      BITNESS => BITNESS,
      RESET   => RESET_N
    )
    port map (
      iClk     => clk,
      iRst     => rst,
      iNear    => iNear,
      iQbpp    => iQbpp,
      iAinit   => iAinit,
      iClr     => iClr,
      iClrAddr => iClrAddr,
      iValid   => iValid,
      iMode    => iMode,
      iRiType  => iRiType,
      iErrval  => iErrval,
      iCtx     => iCtx,
      iBpre    => iBpre,
      iNpre    => iNpre,
      iLane    => iLane,
      iLastPix => iLastPix,
      iEol     => iEol,
      oWord    => oWord,
      oValid   => oValid,
      oLen     => oLen,
      oLane    => oLane,
      oLast    => oLast
    );

  stim : process is

    constant BQ : natural := 65536;                                      -- bit ring per lane
    constant TQ : natural := 4096;                                       -- token ring per lane

    type t_bits is array (0 to 3, 0 to BQ - 1) of std_logic;
    type t_tlen is array (0 to 3, 0 to TQ - 1) of natural;
    type t_tlast is array (0 to 3, 0 to TQ - 1) of boolean;
    type t_req is array (0 to 9) of AlertLogIDType;
    type t_lane_nat is array (0 to 3) of natural;
    type t_lane_int is array (0 to 3) of integer;
    type t_lane_bool is array (0 to 3) of boolean;
    type t_actx is array (0 to 3, 0 to 511) of integer;
    type t_ri is array (0 to 3, 0 to 1) of integer;
    type t_hot is array (0 to 11) of natural;

    variable rv       : RandomPType;
    variable reqA10   : AlertLogIDType;
    variable reqA11   : AlertLogIDType;
    variable reqA111  : AlertLogIDType;
    variable reqA12   : AlertLogIDType;
    variable reqA15   : AlertLogIDType;
    variable reqA16   : AlertLogIDType;
    variable reqA20   : AlertLogIDType;
    variable reqA21   : AlertLogIDType;
    variable reqA22   : AlertLogIDType;
    variable reqA23   : AlertLogIDType;
    variable reqLane  : AlertLogIDType;
    variable reqTag   : AlertLogIDType;
    variable reqRst   : AlertLogIDType;
    variable covTok   : CoverageIDType;
    variable covK     : CoverageIDType;
    variable covCode  : CoverageIDType;
    variable covSpace : CoverageIDType;
    variable covRst   : CoverageIDType;
    variable covRange : CoverageIDType;

    -- Session config
    variable cfgNear  : natural := 0;
    variable cfgRange : natural := 2;
    variable cfgQbpp  : natural := 1;
    variable cfgAinit : natural := 2;
    variable eLo      : integer := 0;
    variable eHi      : integer := 0;
    variable cfgWs    : natural := 8;
    variable cfgH     : natural := 1;
    variable nImg     : natural := 1;
    variable longRun  : boolean := false;
    variable nHot     : natural := 1;
    variable hot      : t_hot;
    variable riLvl    : natural := 0;
    variable steer    : natural := S_NONE;

    -- Golden state per lane
    variable stA      : t_actx;
    variable stN      : t_actx;                                          -- regular N (TB plays the loop)
    variable stLvl    : t_actx;                                          -- error magnitude of a context
    variable riA      : t_ri;
    variable riN      : t_ri;
    variable riNn     : t_ri;
    variable runIdx   : t_lane_nat;
    variable runCnt   : t_lane_nat;
    variable runLen   : t_lane_nat;                                      -- A.14 RUNcnt
    variable rcTarget : t_lane_int;                                      -- long run: RI at this RUNcnt at 30
    variable lnS30    : t_lane_nat;                                      -- long run: lines started at RUNindex 30
    variable lnDesc   : t_lane_bool;                                     -- long run: RIs down to RUNindex 23
    variable lnInRun  : t_lane_bool;                                     -- run open (A.14 RUNcnt > 0)

    -- Expected stream per lane
    variable bits     : t_bits;
    variable bWr      : t_lane_nat;
    variable bRd      : t_lane_nat;
    variable tLen     : t_tlen;
    variable tLast    : t_tlast;
    variable tMask    : t_tlen;
    variable tokMask  : natural;
    variable reqs     : t_req;
    variable tWr      : t_lane_nat;
    variable tRd      : t_lane_nat;
    variable tokLen   : natural;
    variable tokLane  : natural;

    -- Issue state
    variable cycles   : integer := 0;
    variable inRst    : boolean := true;
    variable postRst  : boolean := false;
    variable lnRow    : t_lane_nat;
    variable lnCol    : t_lane_nat;
    variable lnDone   : t_lane_bool;
    variable lnLastAt : t_lane_int;
    variable lnLastOut : t_lane_bool;
    variable st       : integer := 0;                                    -- 0 gap, 1 sweep, 2 tokens, 3 done
    variable gapLeft  : natural := 0;
    variable clrCnt   : natural := 0;
    variable imgLeft  : natural := 0;
    variable idleLeft : natural := 0;
    variable imgsDone : natural := 0;
    variable ph       : integer;
    variable hit      : boolean;

    impure function pending return boolean is
    begin
      for l in 0 to 3 loop
        if (tRd(l) /= tWr(l)) then
          return true;
        end if;
      end loop;
      return false;
    end function pending;

    impure function in_run return boolean is
    begin
      for l in 0 to 3 loop
        if (lnInRun(l)) then
          return true;
        end if;
      end loop;
      return false;
    end function in_run;

    procedure put (
      val : natural;
      n   : natural
    ) is
      variable v : natural := val;
    begin
      for i in n - 1 downto 0 loop
        bits(tokLane, bWr(tokLane) mod BQ) := sl(i < 30 and (v / 2 ** minimum(i, 29)) mod 2 = 1);
        bWr(tokLane) := bWr(tokLane) + 1;
      end loop;
      tokLen := tokLen + n;
    end procedure put;

    -- A.11.1 LG(k, lim) with qbpp; returns true for an escape.
    procedure lg (
      k      : natural;
      merr   : natural;
      lim    : natural;
      escape : out boolean
    ) is
      variable thr : integer;
    begin
      thr := lim - cfgQbpp - 1;
      if (merr / 2 ** k < thr) then
        put(0, merr / 2 ** k);
        put(1, 1);
        put(merr mod 2 ** k, k);
        escape := false;
        if (merr / 2 ** k = 0) then ICover(covRange, R_UN0); end if;
        if (k = 0) then ICover(covRange, R_SL0); end if;
        if (merr mod 2 ** k = 0) then ICover(covRange, R_SV0); end if;
        if (merr mod 2 ** k = 2 ** BITNESS - 1) then ICover(covRange, R_SVMAX); end if;
      else
        put(0, thr);
        put(1, 1);
        put(merr - 1, cfgQbpp);
        escape := true;
        if (thr = LIMIT - cfgQbpp - 1) then ICover(covRange, R_UNMAX); end if;
        if (cfgQbpp = BITNESS) then ICover(covRange, R_SLMAX); end if;
        if (merr - 1 = 2 ** BITNESS - 1) then ICover(covRange, R_SVMAX); end if;
      end if;
    end procedure lg;

    procedure end_token (last : boolean) is
    begin
      if (tokLen > 0) then
        tLen(tokLane, tWr(tokLane) mod TQ)  := tokLen;
        tLast(tokLane, tWr(tokLane) mod TQ) := last;
        tMask(tokLane, tWr(tokLane) mod TQ) := tokMask;
        tWr(tokLane) := tWr(tokLane) + 1;
      end if;
      AffirmIf(reqTag, tokLen > 0 or not last, "last pixel of a lane gives a word");
    end procedure end_token;

    procedure clear_streams is
    begin
      for l in 0 to 3 loop
        bWr(l) := 0;
        bRd(l) := 0;
        tWr(l) := 0;
        tRd(l) := 0;
      end loop;
    end procedure clear_streams;

    procedure start_image is
    begin
      for l in 0 to 3 loop
        for x in 0 to 511 loop
          stA(l, x) := cfgAinit;
          stN(l, x) := 1;
        end loop;
        for t in 0 to 1 loop
          riA(l, t)  := cfgAinit;
          riN(l, t)  := 1;
          riNn(l, t) := 0;
        end loop;
        lnRow(l)     := 0;
        lnCol(l)     := 0;
        lnDone(l)    := false;
        lnLastOut(l) := false;
        lnLastAt(l)  := -100;
        lnS30(l)     := 0;
        lnDesc(l)    := false;
        rcTarget(l)  := -1;
      end loop;
      nHot := rv.RandInt(1, 12);
      for i in 0 to nHot - 1 loop
        hot(i) := rv.DistValInt(((4, 1), (368, 1), (rv.RandInt(5, 367), 6)));
      end loop;
      for l in 0 to 3 loop
        for x in 0 to 511 loop
          stLvl(l, x) := rv.RandInt(0, BITNESS - 1);
        end loop;
      end loop;
      for l in 0 to 3 loop
        stLvl(l, hot(0)) := 0;                                           -- quiet context: A falls to N, k = 0
      end loop;
      riLvl := rv.RandInt(0, BITNESS - 1);
    end procedure start_image;

    -- Error with magnitude around 2^lvl, plus extremes and surprises.
    impure function gen_err (lvl : natural) return integer is
      variable m : integer;
      variable e : integer;
    begin
      if (lvl = 0) then
        return rv.RandInt(-1, 1);
      end if;
      case rv.DistValInt(((0, 85), (1, 5), (2, 5), (3, 5))) is
        when 0 =>
          m := rv.RandInt(2 ** lvl / 2, 2 ** lvl);
          if (rv.RandInt(0, 1) = 0) then
            e := -m;
          else
            e := m;
          end if;
        when 1 =>
          e := rv.DistValInt(((eLo, 1), (eHi, 1)));
        when 2 =>
          e := 0;
        when others =>
          e := rv.RandInt(eLo, eHi);
      end case;
      return maximum(eLo, minimum(eHi, e));
    end function gen_err;

    -- Steered Errval for a regular (rt 0) or RI token. Quiet images turn to small random errors in
    -- their second half, so tokens follow A = 0 with real errors.
    impure function steer_err (l, rt : natural) return integer is
      variable e : integer;
    begin
      if (steer = S_LOUD) then
        return eLo;
      elsif (steer = S_QUIET and 2 * lnRow(l) >= cfgH) then
        e := maximum(eLo, minimum(eHi, rv.RandInt(-4, 4)));              -- small A: an off-by-one flips k
        if (rt = 1 and e = 0) then
          e := 1;
        end if;
        return e;
      elsif (rt = 0) then
        return 0;
      else
        return 2 * rv.RandInt(0, 1) - 1;
      end if;
    end function steer_err;

    -- One token of lane l: pick a legal kind, drive it, append its code.
    procedure issue (l : natural) is
      variable kind  : integer;
      variable eol   : boolean;
      variable last  : boolean;
      variable ctx   : natural;
      variable e     : integer;
      variable n, a  : integer;
      variable npre  : natural;
      variable b     : integer;
      variable k     : natural;
      variable merr  : natural;
      variable temp  : integer;
      variable rt    : natural;
      variable mp    : natural;
      variable esc   : boolean;
      variable dist  : integer;
      variable bnd   : natural;
    begin
      eol  := lnCol(l) = cfgWs - 1;
      last := eol and lnRow(l) = cfgH - 1;
      -- Lines started at RUNindex 30: 1st descends to 23 (A.15 bound max), 2nd ends in an RI after
      -- 2^14-2 hits (raw max), later ones RI at a random RUNcnt.
      if (longRun and lnCol(l) = 0) then
        rcTarget(l) := -1;
        if (runIdx(l) = 30) then
          lnS30(l) := lnS30(l) + 1;
          lnDesc(l) := lnS30(l) = 1;
          if (lnS30(l) = 2) then
            rcTarget(l) := 2 ** 14 - 2;
          elsif (lnS30(l) > 2) then
            rcTarget(l) := rv.RandInt(1, 2 ** 14 - 3);
          end if;
        end if;
      end if;
      if (longRun) then
        if (lnDesc(l) and runIdx(l) = 23) then
          lnDesc(l) := false;
        end if;
        if (lnDesc(l) or (runIdx(l) = 30 and runCnt(l) = rcTarget(l))) then
          kind := K_RI0 + rv.RandInt(0, 1);
        else
          kind := K_HIT;
        end if;
      elsif (lnInRun(l) and steer /= S_NONE) then
        kind := rv.DistValInt(((K_HIT, 2), (K_RI0, 1), (K_RI1, 1)));
      elsif (lnInRun(l)) then
        kind := rv.DistValInt(((K_HIT, 6), (K_RI0, 1), (K_RI1, 1)));
      elsif (steer /= S_NONE) then
        kind := rv.DistValInt(((K_REG, 2), (K_HIT, 2), (K_RI0, 1), (K_RI1, 1)));
      else
        kind := rv.DistValInt(((K_REG, 6), (K_HIT, 2), (K_RI0, 1), (K_RI1, 1)));
      end if;
      if (kind = K_HIT and eol) then
        kind := K_HITEOL;
      end if;

      dist := cycles - lnLastAt(l);
      if (dist = 4) then
        ICover(covSpace, (l, 0));
      elsif (dist <= 8) then
        ICover(covSpace, (l, 1));
      else
        ICover(covSpace, (l, 2));
      end if;
      lnLastAt(l) := cycles;
      ICover(covTok, (l, kind));

      tokLane := l;
      tokLen  := 0;
      tokMask := 0;
      npre    := 1;
      e       := 0;
      ctx     := 0;
      b       := 0;
      n       := 1;

      case kind is
        when K_REG =>
          case rv.DistValInt(((0, 3), (1, 6), (2, 1))) is
            when 0      => ctx := hot(0);                                -- quiet context
            when 1      => ctx := hot(rv.RandInt(0, nHot - 1));
            when others => ctx := rv.RandInt(4, 368);
          end case;
          e := gen_err(stLvl(l, ctx));
          if (steer /= S_NONE) then
            ctx := hot(0);
            e   := steer_err(l, 0);
          end if;
          n    := stN(l, ctx);
          npre := n;
          b    := rv.DistValInt(((1 - n, 1), (0, 1), (rv.RandInt(1 - n, 0), 2)));
          tokMask := M_A10 + M_A11 + M_A111 + M_A12;
          if (n = 1) then ICover(covRange, R_N1); end if;
          if (n = RESET_N) then ICover(covRange, R_NRESET); end if;
          if (b = 1 - n) then ICover(covRange, R_BMIN); end if;
          if (b = 0) then ICover(covRange, R_B0); end if;
          a := stA(l, ctx);
          if (a = 0) then ICover(covRange, R_AREG0); end if;
          if (a = A_MAX) then ICover(covRange, R_AREGMAX); end if;
          -- A.10
          k := 0;
          while n * 2 ** k < a loop
            k := k + 1;
          end loop;
          -- A.11
          if (cfgNear = 0 and k = 0 and 2 * b <= -n) then
            ICover(covCode, C_SPECIAL);
            if (e >= 0) then
              merr := 2 * e + 1;
            else
              merr := -2 * (e + 1);
            end if;
          elsif (e >= 0) then
            merr := 2 * e;
          else
            merr := -2 * e - 1;
          end if;
          if (merr = 0) then ICover(covRange, R_MREG0); end if;
          if (merr = 2 ** BITNESS - 1) then ICover(covRange, R_MREGMAX); end if;
          lg(k, merr, LIMIT, esc);
          -- A.12 (A only; B, N are the loop's)
          a := a + abs(e);
          if (n = RESET_N) then
            a := a / 2;
            n := n / 2;
            ICover(covCode, C_HALF_REG);
          end if;
          stA(l, ctx) := a;
          stN(l, ctx) := n + 1;
          if (n + 1 = 2) then ICover(covRange, R_NREG2); end if;
          if (n + 1 = RESET_N) then ICover(covRange, R_NREGMAX); end if;
          ICover(covK, (k, 0));
          if (esc) then ICover(covCode, C_ESC_REG); end if;
          if (e = eLo) then ICover(covRange, R_EMIN); end if;
          if (e = eHi) then ICover(covRange, R_EMAX); end if;
          if (ctx = 4) then ICover(covRange, R_CTX4); end if;
          if (ctx = 368) then ICover(covRange, R_CTX368); end if;
          if (k = 0) then ICover(covRange, R_K0); end if;
          if (k = BITNESS - 1) then ICover(covRange, R_KMAX); end if;
          if (tokLen = LIMIT) then ICover(covRange, R_LIMIT); end if;

        when K_HIT | K_HITEOL =>
          tokMask := M_A15 + M_A16;
          lnInRun(l) := kind = K_HIT;
          -- A.15, one hit at a time: a completed segment gives '1'.
          runLen(l) := runLen(l) + 1;
          if (runLen(l) = 16383) then ICover(covRange, R_CNTMAX); end if;
          runCnt(l) := runCnt(l) + 1;
          if (runCnt(l) = 2 ** J(runIdx(l))) then
            put(1, 1);
            runCnt(l) := 0;
            ICover(covCode, C_SEG);
            if (runIdx(l) < 31) then                                     -- 31 unreachable on 16383-pixel lines
              runIdx(l) := runIdx(l) + 1;
            end if;
          end if;
          -- A.16 at the end of the line.
          if (kind = K_HITEOL) then
            if (runCnt(l) > 0) then
              put(1, 1);
              ICover(covCode, C_EOL1);
            else
              ICover(covCode, C_EOL0);
            end if;
            runCnt(l) := 0;
            runLen(l) := 0;
          end if;
          if (runIdx(l) = 30) then ICover(covRange, R_RIDX31); end if;
          bnd := runLen(l) - runCnt(l) + 2 ** J(runIdx(l));                -- run count of the next '1'
          if (bnd = 1) then ICover(covRange, R_BND1); end if;
          if (bnd = 32640) then ICover(covRange, R_BNDMAX); end if;

        when others =>
          -- A.16 interruption: '0' + J[RUNindex] bits of RUNcnt.
          rt := kind - K_RI0;
          tokMask := M_A10 + M_A111 + M_A16 + M_A20 + M_A21 + M_A22 + M_A23;
          lnInRun(l) := false;
          if (runCnt(l) = 0) then
            ICover(covCode, C_RI_RC0);
          else
            ICover(covCode, C_RI_RCP);
          end if;
          if (J(runIdx(l)) >= 2 and runCnt(l) = 2 ** J(runIdx(l)) - 1) then
            ICover(covRange, R_RCMAX);
          end if;
          if (runIdx(l) = 0) then ICover(covRange, R_RIIDX0); end if;
          if (runIdx(l) = 30) then ICover(covRange, R_RIIDX30); end if;
          if (runIdx(l) = 30 and runCnt(l) = 2 ** 14 - 2) then ICover(covRange, R_RAWMAX); end if;
          put(0, 1);
          put(runCnt(l), J(runIdx(l)));
          -- Errval: RItype 1 never 0.
          e := gen_err(riLvl);
          if (rt = 1 and e = 0) then
            e := eHi;
            if (e = 0) then
              e := eLo;
            end if;
          end if;
          if (steer /= S_NONE) then
            e := steer_err(l, rt);
          end if;
          if (steer = S_SPIKE and rt = 0 and riA(l, 0) <= riN(l, 0) and 2 * riNn(l, 0) < riN(l, 0) and
              rv.RandInt(0, 7) = 0) then
            e := eLo;                                                    -- k = 0, map 0: EMErrval = 2^BITNESS
          end if;
          a  := riA(l, rt);
          n  := riN(l, rt);
          if (a = 0) then ICover(covRange, R_ARI0_T0 + rt); end if;
          if (a = A_MAX) then ICover(covRange, R_ARIMAX); end if;
          if (n = 1) then ICover(covRange, R_NRI1); end if;
          if (n = RESET_N) then ICover(covRange, R_NRIRESET); end if;
          if (riNn(l, rt) = 0) then ICover(covRange, R_NN0); end if;
          if (riNn(l, rt) = RESET_N - 1) then ICover(covRange, R_NNMAX); end if;
          -- A.20
          if (rt = 0) then
            temp := a;
          else
            temp := a + n / 2;
          end if;
          -- A.10
          k := 0;
          while n * 2 ** k < temp loop
            k := k + 1;
          end loop;
          -- A.21
          if ((k = 0 and e > 0 and 2 * riNn(l, rt) < n) or (e < 0 and (2 * riNn(l, rt) >= n or k /= 0))) then
            mp := 1;
          else
            mp := 0;
          end if;
          ICover(covCode, C_MAP0_T0 + 2 * rt + mp);
          -- A.22
          merr := 2 * abs(e) - rt - mp;
          if (merr = 0) then ICover(covRange, R_MRI0); end if;
          if (merr = 2 ** BITNESS) then ICover(covRange, R_MRIMAX); end if;
          lg(k, merr, LIMIT - J(runIdx(l)) - 1, esc);
          if (esc) then ICover(covCode, C_ESC_RI); end if;
          if (tokLen = LIMIT) then ICover(covRange, R_LIMIT); end if;
          ICover(covK, (k, 1));
          if (e = eLo) then ICover(covRange, R_RIEMIN); end if;
          if (e = eHi) then ICover(covRange, R_RIEMAX); end if;
          -- A.23
          if (e < 0) then
            riNn(l, rt) := riNn(l, rt) + 1;
          end if;
          a := a + (merr + 1 - rt) / 2;
          if (n = RESET_N) then
            a           := a / 2;
            n           := n / 2;
            riNn(l, rt) := riNn(l, rt) / 2;
            ICover(covCode, C_HALF_RI);
          end if;
          riA(l, rt) := a;
          riN(l, rt) := n + 1;
          if (n + 1 = 2) then ICover(covRange, R_NRI2); end if;
          if (n + 1 = RESET_N) then ICover(covRange, R_NRIMAX); end if;
          if (runIdx(l) > 0) then
            runIdx(l) := runIdx(l) - 1;
          end if;
          runCnt(l) := 0;
          runLen(l) := 0;
          if (2 ** J(runIdx(l)) = 1) then ICover(covRange, R_BND1); end if;
      end case;

      end_token(last);

      -- Run state clears after the lane's last pixel.
      if (last) then
        runCnt(l)  := 0;
        runLen(l)  := 0;
        runIdx(l)  := 0;
        lnInRun(l) := false;
        lnDone(l)  := true;
      elsif (eol) then
        lnCol(l) := 0;
        lnRow(l) := lnRow(l) + 1;
      else
        lnCol(l) := lnCol(l) + 1;
      end if;

      iValid   <= '1';
      iLane    <= to_unsigned(l, 2);
      iEol     <= sl(eol);
      iLastPix <= sl(last);
      iCtx     <= to_unsigned(ctx, CO_CTX_WIDTH);
      iErrval  <= to_signed(e, BITNESS + 1);
      case kind is
        when K_REG =>
          iMode   <= "00";
          iRiType <= sl(rv.RandInt(0, 1) = 1);                           -- don't care
          iBpre   <= to_signed(b, 7);
          iNpre   <= to_unsigned(npre, 7);
        when K_HIT | K_HITEOL =>
          iMode   <= "01";
          iRiType <= sl(rv.RandInt(0, 1) = 1);
          iBpre   <= to_signed(rv.RandInt(-63, 0), 7);
          iNpre   <= to_unsigned(rv.RandInt(1, 64), 7);
        when others =>
          iMode   <= "10";
          iRiType <= sl(kind = K_RI1);
          iBpre   <= to_signed(rv.RandInt(-63, 0), 7);
          iNpre   <= to_unsigned(rv.RandInt(1, 64), 7);
      end case;
    end procedure issue;

    -- Words of this cycle against the lane streams.
    procedure monitor is
      variable l   : natural;
      variable len : natural;
      variable ok  : boolean;
    begin
      if (postRst) then
        postRst := false;
        AffirmIf(reqRst, oValid = '0', "after reset: no word");
      end if;
      if (oValid /= '1') then
        return;
      end if;
      l   := checked_integer(oLane);
      len := checked_integer(oLen);
      if (tRd(l) = tWr(l)) then
        AffirmIf(reqTag, false, "word on lane " & to_string(l) & " with no coded token pending");
        return;
      end if;
      AffirmIfEqual(reqTag, len, tLen(l, tRd(l) mod TQ), "word length, lane " & to_string(l));
      AffirmIfEqual(reqTag, oLast, sl(tLast(l, tRd(l) mod TQ)), "oLast, lane " & to_string(l));
      ok := true;
      for i in 0 to len - 1 loop
        if (bRd(l) < bWr(l)) then
          if (oWord(WW - 1 - i) /= bits(l, bRd(l) mod BQ)) then
            ok := false;
          end if;
          bRd(l) := bRd(l) + 1;
        else
          ok := false;
        end if;
      end loop;
      AffirmIf(reqLane, ok, "code bits, lane " & to_string(l) & " token " & to_string(tRd(l)));
      for i in 0 to 9 loop
        if ((tMask(l, tRd(l) mod TQ) / 2 ** i) mod 2 = 1) then
          AffirmIf(reqs(i), ok, "code bits, lane " & to_string(l));
        end if;
      end loop;
      if (tLast(l, tRd(l) mod TQ)) then
        lnLastOut(l) := true;
        AffirmIfEqual(reqTag, bRd(l), bWr(l), "lane " & to_string(l) & " stream complete at oLast");
      end if;
      tRd(l) := tRd(l) + 1;
    end procedure monitor;

    procedure drive is
      variable cand : t_lane_nat;
      variable nc   : natural;
      variable done : boolean;
    begin
      iValid <= '0';
      iClr   <= '0';
      iMode  <= rv.RandSlv(2);
      iErrval <= to_signed(rv.RandInt(-2 ** (BITNESS - 1), 2 ** (BITNESS - 1) - 1), BITNESS + 1);
      iLastPix <= sl(rv.RandInt(0, 1) = 1);
      if (inRst) then
        return;
      end if;
      case st is
        when 0 =>                                                        -- gap before the sweep
          if (gapLeft > 0) then
            gapLeft := gapLeft - 1;
          elsif (imgLeft > 0) then
            st     := 1;
            clrCnt := 0;
            start_image;
          end if;
        when 1 =>                                                        -- sweep
          null;
        when 2 =>                                                        -- tokens
          done := true;
          for l in 0 to 3 loop
            done := done and lnDone(l);
          end loop;
          if (done) then
            imgLeft  := imgLeft - 1;
            imgsDone := imgsDone + 1;
            gapLeft  := rv.RandInt(6, 40);
            st       := 0;
          elsif (idleLeft > 0) then
            idleLeft := idleLeft - 1;
          elsif (not longRun and rv.DistValInt(((1, 1), (0, 100))) = 1) then
            idleLeft := rv.RandInt(1, 30);
          elsif (longRun or rv.DistValInt(((1, 5), (0, 1))) = 1) then
            nc := 0;
            for l in 0 to 3 loop
              if (not lnDone(l) and cycles - lnLastAt(l) >= 4) then
                cand(nc) := l;
                nc       := nc + 1;
              end if;
            end loop;
            if (nc > 0) then
              issue(cand(rv.RandInt(0, nc - 1)));
            end if;
          end if;
        when others =>
          null;
      end case;
      if (st = 1) then
        iClr     <= '1';
        iClrAddr <= to_unsigned(clrCnt, 9);
        clrCnt   := clrCnt + 1;
        if (clrCnt = 512) then
          st := 2;
        end if;
      end if;
    end procedure drive;

    procedure step is
    begin
      wait until rising_edge(clk);
      wait for 1 ns;
      cycles := cycles + 1;
      if (not inRst) then
        monitor;
      end if;
      drive;
    end procedure step;

    impure function session_done return boolean is
    begin
      return imgLeft = 0 and st = 0 and not pending;
    end function session_done;

    procedure begin_session (n : positive; long : boolean; mode : natural := S_NONE) is
    begin
      longRun := long;
      steer   := mode;
      cfgNear := rv.DistValInt(((0, 3), (NEAR_MAX, 1), (rv.RandInt(1, 7), 1), (rv.RandInt(0, NEAR_MAX), 1)));
      if (mode = S_LOUD or mode = S_SPIKE) then
        cfgNear := 0;                                                    -- widest Errval, Ainit max
      end if;
      if (mode /= S_NONE) then
        cfgWs := STEER_WS;
        cfgH  := STEER_H;
        nImg  := 3;
      elsif (long) then
        cfgWs := LONG_WS;
        cfgH  := LONG_H;
        nImg  := 1;
      else
        cfgWs := rv.DistValInt(((8, 1), (rv.RandInt(8, 64), 3)));
        cfgH  := rv.DistValInt(((1, 1), (rv.RandInt(2, 64), 3)));
        nImg  := rv.RandInt(1, 3);
      end if;
      -- T.87 C.2.4.1.1 / A.2.1, from the TB's NEAR.
      cfgRange := (MAXVAL + 2 * cfgNear) / (2 * cfgNear + 1) + 1;
      cfgQbpp  := 1;
      while 2 ** cfgQbpp < cfgRange loop
        cfgQbpp := cfgQbpp + 1;
      end loop;
      cfgAinit := maximum(2, (cfgRange + 32) / 64);
      eLo      := -(cfgRange / 2);
      eHi      := (cfgRange + 1) / 2 - 1;
      if (cfgNear = 0) then ICover(covRange, R_NEAR0); end if;
      if (cfgNear = NEAR_MAX) then ICover(covRange, R_NEARMAX); end if;
      if (cfgAinit = 2) then ICover(covRange, R_AINIT2); end if;
      if (cfgAinit = AINIT_MAX) then ICover(covRange, R_AINITMAX); end if;

      inRst  := true;
      rst    <= '1';
      iNear  <= to_unsigned(cfgNear, 8);
      iQbpp  <= to_unsigned(cfgQbpp, 5);
      iAinit <= to_unsigned(cfgAinit, BITNESS);
      for i in 1 to n loop
        step;
      end loop;
      rst     <= '0';
      inRst   := false;
      postRst := true;
      clear_streams;
      for l in 0 to 3 loop
        runCnt(l)   := 0;
        runIdx(l)   := 0;
        runLen(l)   := 0;
        lnInRun(l)  := false;
        lnLastAt(l) := -100;
      end loop;
      st      := 0;
      gapLeft := rv.RandInt(0, 10);
      imgLeft := nImg;
    end procedure begin_session;

    procedure run_session is
    begin
      while not session_done loop
        step;
        exit when cycles > 20_000_000;
      end loop;
      AffirmIf(GetAlertLogID("Completion"), session_done, "session coded every image");
      for l in 0 to 3 loop
        AffirmIf(reqLane, lnLastOut(l), "lane " & to_string(l) & ": last word seen");
      end loop;
    end procedure run_session;

    impure function phase_hit (p : integer) return boolean is
    begin
      case p is
        when RP_IDLE  => return st = 0 and not pending;
        when RP_CLEAR => return st = 1;
        when RP_BUSY  => return st = 2 and not in_run;
        when RP_RUN   => return st = 2 and in_run;
        when others   => return st = 0 and pending;
      end case;
    end function phase_hit;

  begin

    SetAlertLogName("tb_nl_backend_osvvm");
    SetLogEnable(PASSED, FALSE);
    rv.InitSeed(rv'instance_name);
    reqA10  := GetReqID("T87.A10", 1000);
    reqA11  := GetReqID("T87.A11", 1000);
    reqA111 := GetReqID("T87.A11.1", 1000);
    reqA12  := GetReqID("T87.A12", 1000);
    reqA15  := GetReqID("T87.A15", 1000);
    reqA16  := GetReqID("T87.A16", 1000);
    reqA20  := GetReqID("T87.A20", 1000);
    reqA21  := GetReqID("T87.A21", 1000);
    reqA22  := GetReqID("T87.A22", 1000);
    reqA23  := GetReqID("T87.A23", 1000);
    reqLane := GetReqID("NL.LaneState", 1000);
    reqTag  := GetReqID("NL.WordTags", 1000);
    reqRst  := GetReqID("NL.Reset", N_RST);
    reqs    := (reqA10, reqA11, reqA111, reqA12, reqA15, reqA16, reqA20, reqA21, reqA22, reqA23);

    covTok := NewID("G4.Lane lane x token kind");
    SetFieldName(covTok, "lane", "kind");
    for l in 0 to 3 loop
      AddCross(covTok, "lane=" & to_string(l) & " / regular", GenBin(l), GenBin(K_REG));
      AddCross(covTok, "lane=" & to_string(l) & " / run hit", GenBin(l), GenBin(K_HIT));
      AddCross(covTok, "lane=" & to_string(l) & " / run hit at EOL", GenBin(l), GenBin(K_HITEOL));
      AddCross(covTok, "lane=" & to_string(l) & " / RI, RItype 0", GenBin(l), GenBin(K_RI0));
      AddCross(covTok, "lane=" & to_string(l) & " / RI, RItype 1", GenBin(l), GenBin(K_RI1));
    end loop;

    covK := NewID("G4.Coding k x token");
    SetFieldName(covK, "k", "token");
    for k in 0 to BITNESS - 1 loop
      AddCross(covK, "k=" & to_string(k) & " / regular", GenBin(k), GenBin(0));
      AddCross(covK, "k=" & to_string(k) & " / RI", GenBin(k), GenBin(1));
    end loop;

    covCode := NewID("G4.Coding corners");
    AddBins(covCode, "regular escape", GenBin(C_ESC_REG));
    AddBins(covCode, "RI escape (glimit)", GenBin(C_ESC_RI));
    AddBins(covCode, "A.11 lossless special mapping", GenBin(C_SPECIAL));
    AddBins(covCode, "RItype 0, map 0", GenBin(C_MAP0_T0));
    AddBins(covCode, "RItype 0, map 1", GenBin(C_MAP1_T0));
    AddBins(covCode, "RItype 1, map 0", GenBin(C_MAP0_T1));
    AddBins(covCode, "RItype 1, map 1", GenBin(C_MAP1_T1));
    AddBins(covCode, "regular halving at N = RESET", GenBin(C_HALF_REG));
    AddBins(covCode, "RI halving at N = RESET", GenBin(C_HALF_RI));
    AddBins(covCode, "run segment completed", GenBin(C_SEG));
    AddBins(covCode, "EOL with a partial segment ('1')", GenBin(C_EOL1));
    AddBins(covCode, "EOL on a segment boundary (no bit)", GenBin(C_EOL0));
    AddBins(covCode, "RI with RUNcnt 0", GenBin(C_RI_RC0));
    AddBins(covCode, "RI after a partial run", GenBin(C_RI_RCP));

    covSpace := NewID("G4.Stream lane x same-lane distance");
    SetFieldName(covSpace, "lane", "distance");
    for l in 0 to 3 loop
      AddCross(covSpace, "lane=" & to_string(l) & " / 4 cycles (min)", GenBin(l), GenBin(0));
      AddCross(covSpace, "lane=" & to_string(l) & " / 5..8 cycles", GenBin(l), GenBin(1));
      AddCross(covSpace, "lane=" & to_string(l) & " / > 8 cycles", GenBin(l), GenBin(2));
    end loop;

    covRst := NewID("G5.States resetPhase");
    AddBins(covRst, "reset before the sweep", 3, GenBin(RP_IDLE));
    AddBins(covRst, "reset during the sweep", 3, GenBin(RP_CLEAR));
    AddBins(covRst, "reset mid-image, no run open", 3, GenBin(RP_BUSY));
    AddBins(covRst, "reset mid-run", 3, GenBin(RP_RUN));
    AddBins(covRst, "reset with words in flight", 3, GenBin(RP_DRAIN));

    covRange := NewID("G2.MinMax node");
    AddBins(covRange, "NEAR = 0", GenBin(R_NEAR0));
    AddBins(covRange, "NEAR = max", GenBin(R_NEARMAX));
    AddBins(covRange, "Ainit = 2", GenBin(R_AINIT2));
    AddBins(covRange, "Ainit = max", GenBin(R_AINITMAX));
    AddBins(covRange, "regular Errval min", GenBin(R_EMIN));
    AddBins(covRange, "regular Errval max", GenBin(R_EMAX));
    AddBins(covRange, "RI Errval min", GenBin(R_RIEMIN));
    AddBins(covRange, "RI Errval max", GenBin(R_RIEMAX));
    AddBins(covRange, "N = 1", GenBin(R_N1));
    AddBins(covRange, "N = RESET", GenBin(R_NRESET));
    AddBins(covRange, "B = 1-N", GenBin(R_BMIN));
    AddBins(covRange, "B = 0", GenBin(R_B0));
    AddBins(covRange, "ctx = 4", GenBin(R_CTX4));
    AddBins(covRange, "ctx = 368", GenBin(R_CTX368));
    AddBins(covRange, "k = 0", GenBin(R_K0));
    AddBins(covRange, "k = BITNESS-1", GenBin(R_KMAX));
    AddBins(covRange, "word = LIMIT bits", GenBin(R_LIMIT));
    AddBins(covRange, "RUNindex = 30 (max on a 16383-pixel line)", GenBin(R_RIDX31));
    AddBins(covRange, "RUNcnt = 2^J - 1 at RI (J >= 2)", GenBin(R_RCMAX));
    AddBins(covRange, "regular A = 0", GenBin(R_AREG0));
    AddBins(covRange, "regular A = Ainit + (RESET-1)*2^(BITNESS-1)", GenBin(R_AREGMAX));
    AddBins(covRange, "RI A = 0, RItype 0", GenBin(R_ARI0_T0));
    AddBins(covRange, "RI A = 0, RItype 1", GenBin(R_ARI0_T1));
    AddBins(covRange, "RI A = Ainit + (RESET-1)*2^(BITNESS-1)", GenBin(R_ARIMAX));
    AddBins(covRange, "Nn = 0", GenBin(R_NN0));
    AddBins(covRange, "Nn = RESET-1", GenBin(R_NNMAX));
    AddBins(covRange, "A.14 RUNcnt = 16383", GenBin(R_CNTMAX));
    AddBins(covRange, "RI at RUNindex = 0", GenBin(R_RIIDX0));
    AddBins(covRange, "RI at RUNindex = 30", GenBin(R_RIIDX30));
    AddBins(covRange, "A.15 bound = 1", GenBin(R_BND1));
    AddBins(covRange, "A.15 bound = 32640 (16256 hits from RUNindex 23)", GenBin(R_BNDMAX));
    AddBins(covRange, "RI RUNcnt = 2^14-2 at RUNindex 30", GenBin(R_RAWMAX));
    AddBins(covRange, "regular updated N = 2", GenBin(R_NREG2));
    AddBins(covRange, "regular updated N = RESET", GenBin(R_NREGMAX));
    AddBins(covRange, "RI updated N = 2", GenBin(R_NRI2));
    AddBins(covRange, "RI updated N = RESET", GenBin(R_NRIMAX));
    AddBins(covRange, "regular MErrval = 0", GenBin(R_MREG0));
    AddBins(covRange, "regular MErrval = 2^BITNESS-1", GenBin(R_MREGMAX));
    AddBins(covRange, "RI EMErrval = 0", GenBin(R_MRI0));
    AddBins(covRange, "RI EMErrval = 2^BITNESS", GenBin(R_MRIMAX));
    AddBins(covRange, "unary = 0", GenBin(R_UN0));
    AddBins(covRange, "unary = LIMIT-QBPP-1", GenBin(R_UNMAX));
    AddBins(covRange, "suffix length = 0", GenBin(R_SL0));
    AddBins(covRange, "suffix length = BITNESS", GenBin(R_SLMAX));
    AddBins(covRange, "suffix value = 0", GenBin(R_SV0));
    AddBins(covRange, "suffix value = 2^BITNESS-1", GenBin(R_SVMAX));
    AddBins(covRange, "RI N = 1", GenBin(R_NRI1));
    AddBins(covRange, "RI N = RESET", GenBin(R_NRIRESET));

    begin_session(rv.RandInt(1, 6), true);
    run_session;
    for s in 0 to N_SESS - 1 loop
      begin_session(rv.RandInt(1, 6), false);
      run_session;
    end loop;
    begin_session(rv.RandInt(1, 6), false, S_QUIET);
    run_session;
    begin_session(rv.RandInt(1, 6), false, S_LOUD);
    run_session;
    begin_session(rv.RandInt(1, 6), false, S_SPIKE);
    run_session;

    for r in 0 to N_RST - 1 loop
      if (r < 5 * 3) then
        ph := r mod 5;
      else
        ph := rv.RandInt(RP_IDLE, RP_DRAIN);
      end if;
      begin_session(rv.RandInt(1, 6), false);
      hit := false;
      while not hit loop
        step;
        hit := phase_hit(ph) and rv.DistValInt(((1, 1), (0, 7))) = 1;
        if (session_done and not hit) then
          begin_session(rv.RandInt(1, 6), false);
        end if;
        exit when cycles > 20_000_000;
      end loop;
      AffirmIf(reqRst, hit, "reset round " & to_string(r) & ": phase " & to_string(ph) & " reached");
      ICover(covRst, ph);
      begin_session(rv.RandInt(1, 6), false);
      run_session;
    end loop;

    AffirmIf(GetAlertLogID("Completion"), imgsDone > 0, "images coded");

    WriteBin(covTok);
    WriteBin(covK);
    WriteBin(covCode);
    WriteBin(covSpace);
    WriteBin(covRst);
    WriteBin(covRange);
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covTok), "token kind coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covK), "k coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covCode), "coding corner coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covSpace), "lane distance coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covRst), "reset-phase coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covRange), "value range coverage closed");

    end_of_test("tb_nl_backend_osvvm");
    wait;

  end process stim;

  watchdog : process is
  begin
    wait for 200 ms;
    Alert(GetAlertLogID("Watchdog"), "tb_nl_backend_osvvm: watchdog timeout", FAILURE);
    std.env.stop;
  end process watchdog;

end architecture sim;
