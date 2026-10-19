-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- tb_nl_loop_osvvm: nl_loop against a per-stripe transcription of the T.87 C code (A.2-A.19)
--
-- The candidates fed to the DUT come from a context memory without each stripe's pending update,
-- so the DUT must forward it. Slot k feeds lane k mod 4; STRIPES lanes carry pixels (1 = 3 empty
-- slots between pixels). The random loop runs CYCLES slots, then until all coverage closes (cap
-- 20 x CYCLES). A.4.1 / A.4.2 arrive precomputed in iCand* (tb_nl_frontend_osvvm).
--
-- Guidelines:
--   G4.Edge     first line, last column, 1-line, Ws = 8 N/A: Rb, Rc, Rd come in
--               (tb_nl_frontend_osvvm)
--   G4.Run      RUNindex N/A: tb_a15_a16_nl_osvvm
--   G4.Coding   k, escape, A.11 N/A: tb_nl_backend_osvvm
--   G4.Config   NEAR change N/A: no NEAR-dependent state (tb_nl_config_osvvm, tb_nl_top_osvvm)
--   G5          N/A: no reset; lane state restarts on iFirstPix

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use work.nl_pkg.all;

library osvvm;
  context osvvm.OsvvmContext;

library tb_support;
  use tb_support.tb_support_pkg.all;

entity tb_nl_loop_osvvm is
  generic (
    BITNESS : natural range 8 to 16 := 12;
    STRIPES : positive range 1 to 4 := 1;
    NEAR    : natural               := 3;
    CYCLES  : positive              := 40000
  );
end entity tb_nl_loop_osvvm;

architecture sim of tb_nl_loop_osvvm is

  constant RESET   : natural := 64;
  constant NW      : natural := 7;
  constant BW      : natural := 7;
  constant MAXVAL  : natural := 2 ** BITNESS - 1;
  constant D       : natural := 2 * NEAR + 1;
  constant RANGE_V : natural := (MAXVAL + 2 * NEAR) / D + 1;
  constant RECIP   : natural := (2 ** (BITNESS + 10) + D - 1) / D;
  constant MAX_CYC : natural := 20 * CYCLES;
  -- Coverage at-least goals; the requirement goals follow from them
  constant MODE_AT : natural := 50;
  constant FWD_AT  : natural := 10;
  constant WRAP_AT : natural := 10;
  -- T.87 C literals (A.13)
  constant MIN_C   : integer := -128;
  constant MAX_C   : integer := 127;
  -- A.9 output range
  constant EMIN    : integer := -(RANGE_V / 2);
  constant EMAX    : integer := (RANGE_V + 1) / 2 - 1;

  -- covRange bins
  constant R_EMIN   : natural := 0;
  constant R_EMAX   : natural := 1;
  constant R_PX0    : natural := 2;
  constant R_PXMAX  : natural := 3;
  constant R_RX0    : natural := 4;
  constant R_RXMAX  : natural := 5;
  constant R_BMIN   : natural := 6;
  constant R_BMAX   : natural := 7;
  constant R_N1     : natural := 8;
  constant R_NMAX   : natural := 9;
  constant R_CMIN   : natural := 10;
  constant R_CMAX   : natural := 11;
  constant R_CMINH  : natural := 12;
  constant R_CMAXH  : natural := 13;
  constant R_BCLN   : natural := 14;
  constant R_BCL0   : natural := 15;
  constant R_IX0    : natural := 16;
  constant R_IXMAX  : natural := 17;

  constant M_REG   : natural := 0;
  constant M_RUN   : natural := 1;
  constant M_RI    : natural := 2;

  function clampt (
    i, j : integer
  ) return integer is
  begin

    if (i > MAXVAL or i < j) then
      return j;
    else
      return i;
    end if;

  end function clampt;

  function factor_t (
    basic, dec, mult : integer
  ) return integer is
  begin

    if (MAXVAL >= 128) then
      return ((minimum(MAXVAL, 4095) + 128) / 256) * (basic - dec) + dec + mult * NEAR;
    else
      return maximum(2, basic / (256 / (MAXVAL + 1)) + mult * NEAR);
    end if;

  end function factor_t;

  constant T1 : integer := clampt(factor_t(3, 2, 3), NEAR + 1);
  constant T2 : integer := clampt(factor_t(7, 3, 5), T1);
  constant T3 : integer := clampt(factor_t(21, 4, 7), T2);

  type t_ctx is array (0 to 364) of integer;

  type t_stripe is record
    ra      : integer;
    inRun   : boolean;
    col     : integer;
    width   : integer;
    first   : boolean;
    hq1     : integer;
    hq2     : integer;
    bias    : integer;
    lastCyc : integer;
    cMem    : t_ctx;
    bMem    : t_ctx;
    nMem    : t_ctx;
    pend    : boolean;
    pCtx    : integer;
    pC      : integer;
    pB      : integer;
    pN      : integer;
    stale   : boolean;
    sCtx    : integer;
  end record t_stripe;

  type t_stripes is array (0 to 3) of t_stripe;

  type t_exp is record
    lane   : integer;
    mode   : integer;
    riType : boolean;
    rx     : integer;
    errval : integer;
    ctx    : integer;
    px     : integer;
    c      : integer;
    b      : integer;
    n      : integer;
    b0     : integer;
    n0     : integer;
    fwd    : boolean;
    wrap   : boolean;
    ix     : integer;
  end record t_exp;

  type t_exp_q is array (0 to 15) of t_exp;

  signal clk       : std_logic := '0';
  signal done      : boolean   := false;

  signal iValid    : std_logic := '0';
  signal iFirstPix : std_logic := '0';
  signal iFirstCol : std_logic := '0';
  signal iEol      : std_logic := '0';
  signal iIx       : unsigned(BITNESS - 1 downto 0) := (others => '0');
  signal iRb       : unsigned(BITNESS - 1 downto 0) := (others => '0');
  signal iRc       : unsigned(BITNESS - 1 downto 0) := (others => '0');
  signal iQ12Zero  : std_logic := '0';
  signal iQ3Bound  : t_signed_array(0 to 7)(BITNESS + 1 downto 0) := (others => (others => '0'));
  signal iCandCtx  : t_unsigned_array(0 to 8)(CO_CTX_WIDTH - 1 downto 0) := (others => (others => '0'));
  signal iCandSign : std_logic_vector(0 to 8) := (others => '0');
  signal iCandC    : t_signed_array(0 to 8)(CO_C_WIDTH - 1 downto 0) := (others => (others => '0'));
  signal iCandB    : t_signed_array(0 to 8)(BW - 1 downto 0) := (others => (others => '0'));
  signal iCandN    : t_unsigned_array(0 to 8)(NW - 1 downto 0) := (others => (others => '0'));

  signal oValid    : std_logic;
  signal oMode     : std_logic_vector(1 downto 0);
  signal oRiType   : std_logic;
  signal oRx       : unsigned(BITNESS - 1 downto 0);
  signal oErrval   : signed(BITNESS downto 0);
  signal oCtx      : unsigned(CO_CTX_WIDTH - 1 downto 0);
  signal oCq       : signed(CO_C_WIDTH - 1 downto 0);
  signal oBq       : signed(BW - 1 downto 0);
  signal oNq       : unsigned(NW - 1 downto 0);
  signal oBpre     : signed(BW - 1 downto 0);
  signal oNpre     : unsigned(NW - 1 downto 0);
  signal oNRes     : std_logic;
  signal oPx       : unsigned(BITNESS - 1 downto 0);

  function quant (
    di : integer
  ) return integer is
  begin

    if (di <= -T3) then
      return -4;
    elsif (di <= -T2) then
      return -3;
    elsif (di <= -T1) then
      return -2;
    elsif (di < -NEAR) then
      return -1;
    elsif (di <= NEAR) then
      return 0;
    elsif (di < T1) then
      return 1;
    elsif (di < T2) then
      return 2;
    elsif (di < T3) then
      return 3;
    else
      return 4;
    end if;

  end function quant;

  -- A.7 near-lossless quantization (C integer division truncates toward 0)
  function quantize (
    e : integer
  ) return integer is
  begin

    if (e > 0) then
      return (e + NEAR) / D;
    else
      return -((NEAR - e) / D);
    end if;

  end function quantize;

  function mod_range (
    e : integer
  ) return integer is
    variable v : integer := e;
  begin

    if (v < 0) then
      v := v + RANGE_V;
    end if;
    if (v >= (RANGE_V + 1) / 2) then
      v := v - RANGE_V;
    end if;
    return v;

  end function mod_range;

  function clampv (
    v : integer
  ) return integer is
  begin

    return maximum(0, minimum(MAXVAL, v));

  end function clampv;

  function mode_name (
    m : integer
  ) return string is
  begin

    if (m = 0) then
      return "regular";
    else
      return "RI";
    end if;

  end function mode_name;

  function sign_name (
    v : integer
  ) return string is
  begin

    case v is

      when 0 =>
        return "neg";
      when 1 =>
        return "zero";
      when others =>
        return "pos";

    end case;

  end function sign_name;

  function clamp_name (
    v : integer
  ) return string is
  begin

    case v is

      when 0 =>
        return "none";
      when 1 =>
        return "below 0";
      when others =>
        return "above MAXVAL";

    end case;

  end function clamp_name;

  function gap_name (
    g : integer
  ) return string is
  begin

    if (g = 0) then
      return "4 slots";
    else
      return ">4 slots";
    end if;

  end function gap_name;

  -- Reconstruction before clamping: 0 in range, 1 below 0, 2 above MAXVAL
  function clamp_bin (
    raw : integer
  ) return integer is
  begin

    if (raw < 0) then
      return 1;
    elsif (raw > MAXVAL) then
      return 2;
    else
      return 0;
    end if;

  end function clamp_bin;

  function sign3 (
    v : integer
  ) return integer is
  begin

    if (v < 0) then
      return 0;
    elsif (v = 0) then
      return 1;
    else
      return 2;
    end if;

  end function sign3;

begin

  assert NEAR <= minimum(255, MAXVAL / 2)
    report "NEAR outside the T.87 range min(255, MAXVAL/2)" severity failure;

  clk <= not clk after 2 ns when not done;

  dut : entity work.nl_loop
    generic map (
      BITNESS => BITNESS,
      RESET   => RESET
    )
    port map (
      iClk      => clk,
      iNear     => to_unsigned(NEAR, 8),
      iMaxVal   => to_unsigned(MAXVAL, BITNESS),
      iRecip    => to_unsigned(RECIP, BITNESS + 11),
      iRange    => to_unsigned(RANGE_V, BITNESS + 1),
      iRangeD   => to_unsigned(RANGE_V * D, BITNESS + 2),
      iValid    => iValid,
      iFirstPix => iFirstPix,
      iFirstCol => iFirstCol,
      iEol      => iEol,
      iIx       => iIx,
      iRb       => iRb,
      iRc       => iRc,
      iQ12Zero  => iQ12Zero,
      iQ3Bound  => iQ3Bound,
      iCandCtx  => iCandCtx,
      iCandSign => iCandSign,
      iCandC    => iCandC,
      iCandB    => iCandB,
      iCandN    => iCandN,
      oValid    => oValid,
      oMode     => oMode,
      oRiType   => oRiType,
      oRx       => oRx,
      oErrval   => oErrval,
      oCtx      => oCtx,
      oCq       => oCq,
      oBq       => oBq,
      oNq       => oNq,
      oBpre     => oBpre,
      oNpre     => oNpre,
      oNRes     => oNRes,
      oPx       => oPx
    );

  stim : process is

    variable rv       : RandomPType;
    variable reqMode  : AlertLogIDType;
    variable reqA4    : AlertLogIDType;
    variable reqA5    : AlertLogIDType;
    variable reqA6    : AlertLogIDType;
    variable reqA7    : AlertLogIDType;
    variable reqRx    : AlertLogIDType;
    variable reqA9    : AlertLogIDType;
    variable reqUpd   : AlertLogIDType;
    variable reqBias  : AlertLogIDType;
    variable reqRun   : AlertLogIDType;
    variable reqRiT   : AlertLogIDType;
    variable reqA18   : AlertLogIDType;
    variable reqA19   : AlertLogIDType;
    variable reqFwd   : AlertLogIDType;
    variable protoId  : AlertLogIDType;
    variable closeId  : AlertLogIDType;
    variable covMode  : CoverageIDType;
    variable covFwd   : CoverageIDType;
    variable covErr   : CoverageIDType;
    variable covWrap  : CoverageIDType;
    variable covClamp : CoverageIDType;
    variable covEdge  : CoverageIDType;
    variable covRange : CoverageIDType;
    variable st       : t_stripes;
    variable s        : natural;
    variable cyc      : natural := 0;
    variable ix, rb, rc, ra : integer;
    variable q1, q2, q3     : integer;
    variable sg       : integer;
    variable ctx      : integer;
    variable cv, bv, nv : integer;
    variable px, e, eq, rx, raw : integer;
    variable ex       : t_exp;
    variable ri       : boolean;
    variable kind     : natural;
    variable biased   : boolean;
    variable extreme  : integer;
    variable expQ     : t_exp_q;
    variable qWr      : natural := 0;
    variable qRd      : natural := 0;
    variable closed   : boolean := false;

    impure function near_val (
      center, spread : integer
    ) return integer is
    begin

      return clampv(center + rv.RandInt(-spread, spread));

    end function near_val;

    impure function edge_val return integer is
    begin

      if (rv.RandInt(0, 1) = 0) then
        return rv.RandInt(0, minimum(MAXVAL, D));
      else
        return rv.RandInt(maximum(0, MAXVAL - D), MAXVAL);
      end if;

    end function edge_val;

    procedure ctx_of (
      a, b, c : in integer;
      vctx    : out integer;
      vsign   : out integer
    ) is
      variable x, y, z : integer := 0;
    begin

      x := a; y := b; z := c;
      if (x < 0 or (x = 0 and y < 0) or (x = 0 and y = 0 and z < 0)) then
        x := -x; y := -y; z := -z;
        vsign := -1;
      else
        vsign := 1;
      end if;
      vctx := 81 * x + 9 * y + z;

    end procedure ctx_of;

    procedure cover_values (
      err, x, p, r : integer
    ) is
    begin

      if (err = EMIN) then
        ICover(covRange, R_EMIN);
      end if;
      if (err = EMAX) then
        ICover(covRange, R_EMAX);
      end if;
      if (x = 0) then
        ICover(covRange, R_IX0);
      elsif (x = MAXVAL) then
        ICover(covRange, R_IXMAX);
      end if;
      if (p = 0) then
        ICover(covRange, R_PX0);
      elsif (p = MAXVAL) then
        ICover(covRange, R_PXMAX);
      end if;
      if (r = 0) then
        ICover(covRange, R_RX0);
      elsif (r = MAXVAL) then
        ICover(covRange, R_RXMAX);
      end if;

    end procedure cover_values;

    impure function all_covered return boolean is
    begin

      return IsCovered(covMode) and IsCovered(covFwd) and IsCovered(covErr) and
             IsCovered(covWrap) and IsCovered(covClamp) and IsCovered(covEdge) and
             IsCovered(covRange);

    end function all_covered;

    -- Outputs sampled on the clock edge, before this edge's update
    procedure check_out is
      variable ex : t_exp;
    begin

      if (oValid = '1') then
        if (qRd >= qWr) then
          Alert(protoId, "oValid with no pixel outstanding");
          return;
        end if;
        ex := expQ(qRd mod 16);
        AffirmIfEqual(reqMode, checked_integer(unsigned(oMode)), ex.mode,
                      "mode, pixel " & to_string(qRd) & " lane " & to_string(ex.lane));
        case ex.mode is

          when M_RUN =>
            AffirmIfEqual(reqRun, checked_integer(oRx), ex.rx, "run Rx, pixel " & to_string(qRd));

          when M_RI =>
            AffirmIfEqual(reqRiT, checked_bit(oRiType), boolean'pos(ex.riType), "RItype, pixel " & to_string(qRd));
            AffirmIfEqual(reqA18, checked_integer(oPx), ex.px, "RI Px, pixel " & to_string(qRd));
            AffirmIfEqual(reqA19, checked_integer(oRx), ex.rx, "RI Rx, pixel " & to_string(qRd));
            AffirmIfEqual(reqA19, checked_integer(oErrval), ex.errval, "RI Errval, pixel " & to_string(qRd));
            if (NEAR = 0) then
              AffirmIfEqual(reqRx, to_integer(oRx), ex.ix, "lossless RI Rx = Ix, pixel " & to_string(qRd));
            end if;
            if (ex.wrap) then
              AffirmIfEqual(reqA9, checked_integer(oErrval), ex.errval, "RI wrapped Errval, pixel " & to_string(qRd));
            end if;

          when others =>
            AffirmIfEqual(reqA4, checked_integer(oCtx), ex.ctx, "ctx, pixel " & to_string(qRd));
            AffirmIfEqual(reqA5, checked_integer(oPx), ex.px, "Px (A.5), pixel " & to_string(qRd));
            AffirmIfEqual(reqA6, checked_integer(oPx), ex.px, "Px (A.6), pixel " & to_string(qRd));
            AffirmIfEqual(reqRx, checked_integer(oRx), ex.rx, "regular Rx, pixel " & to_string(qRd));
            if (NEAR = 0) then
              AffirmIfEqual(reqRx, to_integer(oRx), ex.ix, "lossless Rx = Ix, pixel " & to_string(qRd));
            end if;
            AffirmIfEqual(reqA7, checked_integer(oErrval), ex.errval, "regular Errval, pixel " & to_string(qRd));
            if (ex.wrap) then
              AffirmIfEqual(reqA9, checked_integer(oErrval), ex.errval, "regular wrapped Errval, pixel " & to_string(qRd));
            end if;
            AffirmIfEqual(reqUpd, checked_integer(oNq), ex.n, "N, pixel " & to_string(qRd));
            AffirmIfEqual(reqUpd, checked_integer(oBpre), ex.b0, "B before update, pixel " & to_string(qRd));
            AffirmIfEqual(reqUpd, checked_integer(oNpre), ex.n0, "N before update, pixel " & to_string(qRd));
            AffirmIfEqual(reqUpd, checked_bit(oNRes), boolean'pos(ex.n0 = RESET), "N = RESET flag, pixel " & to_string(qRd));
            AffirmIfEqual(reqBias, checked_integer(oCq), ex.c, "C, pixel " & to_string(qRd));
            AffirmIfEqual(reqBias, checked_integer(oBq), ex.b, "B, pixel " & to_string(qRd));
            if (ex.fwd) then
              AffirmIf(reqFwd,
                       to_integer(oCq) = ex.c and to_integer(oBq) = ex.b and to_integer(oNq) = ex.n and
                       to_integer(oBpre) = ex.b0 and to_integer(oNpre) = ex.n0 and to_integer(oRx) = ex.rx,
                       "forwarded context update, pixel " & to_string(qRd) & " lane " & to_string(ex.lane));
            end if;

        end case;
        qRd := qRd + 1;
      end if;

    end procedure check_out;

  begin

    SetAlertLogName("tb_nl_loop_osvvm");
    SetLogEnable(PASSED, FALSE);
    rv.InitSeed(rv'instance_name & to_string(BITNESS) & to_string(NEAR) & to_string(STRIPES));
    protoId := GetAlertLogID("StreamProtocol");
    closeId := GetAlertLogID("CoverageClosure");

    -- At NEAR = 0, A.2 becomes A.3 and A.8 becomes A.8.1 (Rx = Ix, on top of the reference Rx)
    if (NEAR = 0) then
      reqMode := GetReqID("T87.A3", 3 * MODE_AT * STRIPES);
      reqRx   := GetReqID("T87.A8.1", 3 * MODE_AT * STRIPES);
    else
      reqMode := GetReqID("T87.A2", 3 * MODE_AT * STRIPES);
      reqRx   := GetReqID("T87.A8", MODE_AT * STRIPES);
    end if;
    reqA4   := GetReqID("T87.A4", MODE_AT * STRIPES);
    reqA5   := GetReqID("T87.A5", MODE_AT * STRIPES);
    reqA6   := GetReqID("T87.A6", MODE_AT * STRIPES);
    reqA7   := GetReqID("T87.A7", MODE_AT * STRIPES);
    reqA9   := GetReqID("T87.A9", 2 * WRAP_AT);
    reqUpd  := GetReqID("T87.A12", 4 * MODE_AT * STRIPES);
    reqBias := GetReqID("T87.A13", 2 * MODE_AT * STRIPES);
    reqRun  := GetReqID("T87.A14", MODE_AT * STRIPES);
    reqRiT  := GetReqID("T87.A17", MODE_AT * STRIPES);
    reqA18  := GetReqID("T87.A18", MODE_AT * STRIPES);
    reqA19  := GetReqID("T87.A19", 2 * MODE_AT * STRIPES);
    reqFwd  := GetReqID("NL.CtxForward", 2 * FWD_AT * STRIPES);

    covMode := NewID("G4.Lane G4.Run lane x mode");
    SetFieldName(covMode, "lane", "mode");
    for l in 0 to STRIPES - 1 loop
      AddCross(covMode, "lane=" & to_string(l) & " / regular", MODE_AT, GenBin(l), GenBin(M_REG));
      AddCross(covMode, "lane=" & to_string(l) & " / run", MODE_AT, GenBin(l), GenBin(M_RUN));
      AddCross(covMode, "lane=" & to_string(l) & " / RI", MODE_AT, GenBin(l), GenBin(M_RI));
    end loop;

    -- Regular pixels whose context is the lane's pending (not yet written)
    -- update, back-to-back (4 slots after the previous lane pixel) or after
    -- empty slots.
    covFwd := NewID("G4.Stream lane x forward x gap");
    SetFieldName(covFwd, "lane", "forward", "gap");
    for l in 0 to STRIPES - 1 loop
      for f in 0 to 1 loop
        for g in 0 to 1 loop
          AddCross(covFwd, "lane=" & to_string(l) & " / forward=" & to_string(f) &
                   " / gap=" & gap_name(g), FWD_AT,
                   GenBin(l), GenBin(f), GenBin(g));
        end loop;
      end loop;
    end loop;

    -- Sign of the quantized, modulo-reduced error. RANGE = 2 (NEAR at the
    -- ceiling, 8 bits) maps every positive error to -1.
    covErr := NewID("G4.Branch mode x errval sign");
    SetFieldName(covErr, "mode", "sign");
    for m in 0 to 1 loop
      for sgn in 0 to 2 loop
        if (sgn /= 2 or RANGE_V > 2) then
          AddCross(covErr, mode_name(m) & " / " &
                   sign_name(sgn),
                   GenBin(m), GenBin(sgn));
        end if;
      end loop;
    end loop;
    AddCross(covErr, "RI / RItype=0", GenBin(2), GenBin(0));
    AddCross(covErr, "RI / RItype=1", GenBin(2), GenBin(1));

    covWrap := NewID("G4.Branch mode x modulo wrap");
    SetFieldName(covWrap, "mode", "wrapped");
    for m in 0 to 1 loop
      for w in 0 to 1 loop
        AddCross(covWrap, mode_name(m) & " / wrapped=" & to_string(w), WRAP_AT,
                 GenBin(m), GenBin(w));
      end loop;
    end loop;

    -- Reconstruction Px + SIGN*Errval*(2*NEAR+1) outside [0, MAXVAL] needs
    -- NEAR > 0; lossless reconstructs Ix exactly.
    covClamp := NewID("G2.Clamp mode x reconstruction");
    SetFieldName(covClamp, "mode", "clamp");
    for m in 0 to 1 loop
      for c in 0 to 2 loop
        if (c = 0 or NEAR > 0) then
          AddCross(covClamp, mode_name(m) & " / " &
                   clamp_name(c),
                   GenBin(m), GenBin(c));
        end if;
      end loop;
    end loop;

    covEdge := NewID("G4.Edge G4.Run stripe events");
    AddBins(covEdge, "new image with update pending", GenBin(0));
    AddBins(covEdge, "first regular of image on stale pending ctx", GenBin(1));
    AddBins(covEdge, "regular at first column", GenBin(2));
    AddBins(covEdge, "run hit ends at EOL", GenBin(3));
    AddBins(covEdge, "N = RESET rescale", GenBin(4));

    covRange := NewID("G2.MinMax G2.Clamp G3.Stim G4.Coding node");
    AddBins(covRange, "Errval = -floor(RANGE/2)", GenBin(R_EMIN));
    AddBins(covRange, "Errval = ceil(RANGE/2)-1", GenBin(R_EMAX));
    AddBins(covRange, "Ix = 0", GenBin(R_IX0));
    AddBins(covRange, "Ix = MAXVAL (MSB set)", GenBin(R_IXMAX));
    AddBins(covRange, "Px = 0", GenBin(R_PX0));
    AddBins(covRange, "Px = MAXVAL", GenBin(R_PXMAX));
    AddBins(covRange, "Rx = 0", GenBin(R_RX0));
    AddBins(covRange, "Rx = MAXVAL", GenBin(R_RXMAX));
    AddBins(covRange, "B = 1-RESET", GenBin(R_BMIN));
    AddBins(covRange, "B = 0", GenBin(R_BMAX));
    AddBins(covRange, "N before = 1", GenBin(R_N1));
    AddBins(covRange, "N = RESET", GenBin(R_NMAX));
    AddBins(covRange, "C = -128", GenBin(R_CMIN));
    AddBins(covRange, "C held at -128 (decrement)", GenBin(R_CMINH));
    AddBins(covRange, "B clamped to 1-N", GenBin(R_BCLN));
    -- RANGE = 2 (8 bits, NEAR 127): Errval in {-1, 0}, so B never goes
    -- positive and C never increments.
    if (RANGE_V > 2) then
      AddBins(covRange, "C = 127", GenBin(R_CMAX));
      AddBins(covRange, "C held at 127 (increment)", GenBin(R_CMAXH));
      AddBins(covRange, "B clamped to 0", GenBin(R_BCL0));
    end if;

    for i in 0 to 3 loop
      st(i).first   := true;
      st(i).ra      := 0;
      st(i).inRun   := false;
      st(i).pend    := false;
      st(i).stale   := false;
      st(i).lastCyc := -100;
    end loop;

    wait until rising_edge(clk);

    while not closed loop
      s := cyc mod 4;
      iValid <= '0';

      if (s < STRIPES and rv.RandInt(0, 9) /= 0) then
        -- New stripe image
        if (st(s).first) then
          st(s).first := false;
          st(s).width := rv.RandInt(4, 43);
          st(s).col   := 0;
          st(s).inRun := false;
          st(s).stale := st(s).pend;
          st(s).sCtx  := st(s).pCtx;
          if (st(s).pend) then
            ICover(covEdge, 0);
          end if;
          st(s).pend  := false;
          st(s).cMem  := (others => 0);
          st(s).bMem  := (others => 0);
          st(s).nMem  := (others => 1);
          loop
            st(s).hq1 := rv.RandInt(-4, 4);
            st(s).hq2 := rv.RandInt(-4, 4);
            exit when st(s).hq1 /= 0 or st(s).hq2 /= 0;
          end loop;
          -- One image in six drives a single context with a constant
          -- error so C saturates (A.13 MIN_C / MAX_C).
          if (rv.RandInt(0, 5) = 0) then
            st(s).bias := (2 * rv.RandInt(0, 1) - 1) *
                          (maximum(NEAR + 1, minimum(RESET, MAXVAL / 8)) + rv.RandInt(0, 2 * D));
          else
            st(s).bias := 0;
          end if;
          iFirstPix   <= '1';
        else
          iFirstPix <= '0';
        end if;

        -- Pixels: mostly smooth (runs), some uniform (all Q3, wrap), some at
        -- the range ends (reconstruction clamp).
        kind   := rv.DistInt((60, 25, 15));
        biased := st(s).bias /= 0 and rv.RandInt(0, 4) /= 0;
        -- One uniform pixel in ten aims its regular error at the A.9 range
        -- end (Errval = EMAX or EMIN), rare at 16 bits otherwise.
        if (kind = 1 and rv.RandInt(0, 9) = 0) then
          extreme := 2 * rv.RandInt(0, 1) - 1;
        else
          extreme := 0;
        end if;
        if (biased) then
          -- Hot (Q1, Q2), Q3 = 0, Rb anchored a quarter range away from the
          -- side the bias pushes to (Px ~ Rb + C); Ix = Px + SIGN*bias is set
          -- once Px is known (regular branch).
          if (st(s).bias > 0) then
            rb := near_val(MAXVAL / 4, NEAR);
          else
            rb := near_val(MAXVAL - MAXVAL / 4, NEAR);
          end if;
          if (st(s).col = 0) then
            rc := near_val(rb, NEAR);
          else
            rc := near_val(st(s).ra, NEAR);
          end if;
          ix := rb;
        elsif (kind = 1) then
          ix := rv.RandInt(0, MAXVAL);
          rb := rv.RandInt(0, MAXVAL);
          rc := rv.RandInt(0, MAXVAL);
        elsif (kind = 2) then
          ix := edge_val;
          rb := edge_val;
          rc := edge_val;
        else
          ix := near_val(st(s).ra, 2 * NEAR + 3);
          rb := near_val(ix, NEAR + 2);
          rc := near_val(rb, T2);
        end if;
        case rv.DistInt((50, 30, 20)) is
          when 0 =>
            if (biased) then
              q1 := st(s).hq1; q2 := st(s).hq2;
            else
              q1 := 0; q2 := 0;
            end if;
          when 1 =>
            q1 := st(s).hq1; q2 := st(s).hq2;
          when others =>
            if (biased) then
              q1 := st(s).hq1; q2 := st(s).hq2;
            else
              q1 := rv.RandInt(-4, 4); q2 := rv.RandInt(-4, 4);
            end if;
        end case;

        iIx       <= to_unsigned(ix, BITNESS);
        iRb       <= to_unsigned(rb, BITNESS);
        iRc       <= to_unsigned(rc, BITNESS);
        iFirstCol <= '1' when st(s).col = 0 else '0';
        iEol      <= '1' when st(s).col = st(s).width - 1 else '0';
        iQ12Zero  <= '1' when q1 = 0 and q2 = 0 else '0';
        iQ3Bound  <= (to_signed(rc + T3, BITNESS + 2), to_signed(rc + T2, BITNESS + 2),
                      to_signed(rc + T1, BITNESS + 2), to_signed(rc + NEAR + 1, BITNESS + 2),
                      to_signed(rc - NEAR, BITNESS + 2), to_signed(rc - T1 + 1, BITNESS + 2),
                      to_signed(rc - T2 + 1, BITNESS + 2), to_signed(rc - T3 + 1, BITNESS + 2));

        -- Candidates: memory without this stripe's pending update
        for t in 0 to 8 loop
          ctx_of(q1, q2, t - 4, ctx, sg);
          iCandCtx(t)  <= to_unsigned(ctx, CO_CTX_WIDTH);
          iCandSign(t) <= '1' when sg < 0 else '0';
          iCandC(t)    <= to_signed(st(s).cMem(ctx), CO_C_WIDTH);
          iCandB(t)    <= to_signed(st(s).bMem(ctx), BW);
          iCandN(t)    <= to_unsigned(st(s).nMem(ctx), NW);
        end loop;

        -- Reference (T.87), memory with pending applied
        if (st(s).pend) then
          st(s).cMem(st(s).pCtx) := st(s).pC;
          st(s).bMem(st(s).pCtx) := st(s).pB;
          st(s).nMem(st(s).pCtx) := st(s).pN;
        end if;

        if (st(s).col = 0) then
          ra := rb;
        else
          ra := st(s).ra;
        end if;
        q3 := quant(rc - ra);

        ex.lane := s;
        ex.fwd  := false;
        ex.wrap := false;
        ex.px   := 0;
        if (st(s).inRun or (q1 = 0 and q2 = 0 and q3 = 0)) then
          if (abs(ix - ra) <= NEAR) then
            ex.mode := M_RUN;
            rx      := ra;
            e       := 0;
            if (st(s).col = st(s).width - 1) then
              ICover(covEdge, 3);
            end if;
            st(s).inRun := st(s).col /= st(s).width - 1;
          else
            ex.mode := M_RI;
            -- A.17
            ri := abs(ra - rb) <= NEAR;
            ex.riType := ri;
            ICover(covErr, (2, boolean'pos(ri)));
            -- A.18
            if (ri) then
              px := ra;
            else
              px := rb;
            end if;
            e := ix - px;
            if (not ri and ra > rb) then
              e  := -e;
              sg := -1;
            else
              sg := 1;
            end if;
            -- A.19 (A.8 quantization, A.9 modulo)
            eq  := quantize(e);
            raw := px + sg * eq * D;
            rx  := clampv(raw);
            e   := mod_range(eq);
            ex.px   := px;
            ex.wrap := e /= eq;
            cover_values(e, ix, px, rx);
            ICover(covErr, (1, sign3(e)));
            ICover(covWrap, (1, boolean'pos(e /= eq)));
            ICover(covClamp, (1, clamp_bin(raw)));
            st(s).inRun := false;
          end if;
        else
          ex.mode := M_REG;
          ctx_of(q1, q2, q3, ctx, sg);
          ex.fwd := st(s).pend and st(s).pCtx = ctx;
          ICover(covFwd, (s, boolean'pos(ex.fwd), boolean'pos(cyc - st(s).lastCyc /= 4)));
          if (st(s).stale) then
            if (st(s).sCtx = ctx) then
              ICover(covEdge, 1);
            end if;
            st(s).stale := false;
          end if;
          if (st(s).col = 0) then
            ICover(covEdge, 2);
          end if;
          cv := st(s).cMem(ctx);
          bv := st(s).bMem(ctx);
          nv := st(s).nMem(ctx);
          -- A.5
          if (rc >= maximum(ra, rb)) then
            px := minimum(ra, rb);
          elsif (rc <= minimum(ra, rb)) then
            px := maximum(ra, rb);
          else
            px := ra + rb - rc;
          end if;
          -- A.6
          px := clampv(px + sg * cv);
          if (biased) then
            ix  := clampv(px + sg * st(s).bias);
            iIx <= to_unsigned(ix, BITNESS);
          elsif (extreme /= 0) then
            -- Errval*D directly or after the modulo wrap, whichever fits
            if (extreme > 0) then
              e := EMAX * D;
              raw := -(RANGE_V - EMAX) * D;
            else
              e := EMIN * D;
              raw := (RANGE_V + EMIN) * D;
            end if;
            if (px + sg * e >= 0 and px + sg * e <= MAXVAL) then
              ix := px + sg * e;
            elsif (px + sg * raw >= 0 and px + sg * raw <= MAXVAL) then
              ix := px + sg * raw;
            end if;
            iIx <= to_unsigned(ix, BITNESS);
          end if;
          -- A.7
          e  := ix - px;
          if (sg = -1) then
            e := -e;
          end if;
          -- A.8
          eq  := quantize(e);
          raw := px + sg * eq * D;
          rx  := clampv(raw);
          -- A.9
          e   := mod_range(eq);
          ex.px   := px;
          ex.wrap := e /= eq;
          cover_values(e, ix, px, rx);
          if (nv = 1) then
            ICover(covRange, R_N1);
          end if;
          ICover(covErr, (0, sign3(e)));
          ICover(covWrap, (0, boolean'pos(e /= eq)));
          ICover(covClamp, (0, clamp_bin(raw)));
          ex.b0 := bv;
          ex.n0 := nv;
          -- A.12
          bv := bv + e * D;
          if (nv = RESET) then
            ICover(covEdge, 4);
            if (bv >= 0) then
              bv := bv / 2;
            else
              bv := -((1 - bv) / 2);
            end if;
            nv := nv / 2;
          end if;
          nv := nv + 1;
          if (nv = RESET) then
            ICover(covRange, R_NMAX);
          end if;
          -- A.13
          if (bv <= -nv) then
            bv := bv + nv;
            if (cv > MIN_C) then
              cv := cv - 1;
            else
              ICover(covRange, R_CMINH);
            end if;
            if (bv <= -nv) then
              bv := -nv + 1;
              ICover(covRange, R_BCLN);
            end if;
          elsif (bv > 0) then
            bv := bv - nv;
            if (cv < MAX_C) then
              cv := cv + 1;
            else
              ICover(covRange, R_CMAXH);
            end if;
            if (bv > 0) then
              bv := 0;
              ICover(covRange, R_BCL0);
            end if;
          end if;
          if (cv = MIN_C) then
            ICover(covRange, R_CMIN);
          elsif (cv = MAX_C) then
            ICover(covRange, R_CMAX);
          end if;
          if (bv = 1 - RESET) then
            ICover(covRange, R_BMIN);
          elsif (bv = 0) then
            ICover(covRange, R_BMAX);
          end if;
          st(s).pend := true;
          st(s).pCtx := ctx;
          st(s).pC   := cv;
          st(s).pB   := bv;
          st(s).pN   := nv;
          ex.ctx := ctx;
          ex.c   := cv;
          ex.b   := bv;
          ex.n   := nv;
        end if;
        ICover(covMode, (s, ex.mode));
        ex.rx     := rx;
        ex.errval := e;
        ex.ix     := ix;
        st(s).ra  := rx;
        st(s).lastCyc := cyc;

        if (st(s).col = st(s).width - 1) then
          st(s).col := 0;
          if (rv.RandInt(0, 29) = 0) then
            st(s).first := true;
          end if;
        else
          st(s).col := st(s).col + 1;
        end if;

        expQ(qWr mod 16) := ex;
        qWr := qWr + 1;
        iValid <= '1';
      end if;

      wait until rising_edge(clk);
      check_out;
      cyc := cyc + 1;
      if (cyc mod 256 = 0 and cyc >= CYCLES) then
        closed := all_covered or cyc >= MAX_CYC;
      end if;
    end loop;

    iValid <= '0';
    for i in 0 to 9 loop
      wait until rising_edge(clk);
      check_out;
    end loop;

    AffirmIfEqual(protoId, qRd, qWr, "every pixel produced one output");
    Log("slots " & to_string(cyc) & ", pixels " & to_string(qRd), ALWAYS);
    WriteBin(covMode);
    WriteBin(covFwd);
    WriteBin(covErr);
    WriteBin(covWrap);
    WriteBin(covClamp);
    WriteBin(covEdge);
    WriteBin(covRange);
    AffirmIf(closeId, all_covered, "coverage closed within " & to_string(MAX_CYC) & " slots");

    done <= true;
    end_of_test("tb_nl_loop_osvvm");
    wait;

  end process stim;

end architecture sim;
