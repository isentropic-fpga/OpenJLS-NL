-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--

--------------------------------------------------------------------------------
-- OSVVM testbench: nl_frontend (raster -> 4 stripe lanes -> loop inputs).
--
-- Sessions: reset with a random config (W = 4*Ws, H, NEAR, T1..T3), then 1..3
-- images through the raster port with random gaps, iHold bursts and the
-- row-buffer backpressure. The TB stands in for nl_loop at the exit: every
-- slot leaves E - L = 7 cycles after L with a random mode, Rx and, for a
-- regular exit, one of its candidate contexts with random new (C, B, N).
-- Exits with no slot at E (the loop has no reset) carry junk and must be
-- dropped. The model keeps the Rx lines and the per-lane context store fed
-- by those exits and checks every slot at L against it.
--
-- Requirements (IDs as in the README registry):
--   NL.LaneIssue    slot order: column j of lanes 0..3 on 4 consecutive
--                   cycles, raster order per stripe, Ix, first/eol flags
--   NL.Neighbours   Rb, Rc (and Rd through Q1) from the Rx lines with the
--                   T.87 edge rules: line 0 all 0, Rc = Rb of the previous
--                   line at column 0, Rd = Rb at the stripe end
--   T87.A4          Q1, Q2 (decoded back from candidate 4 and its sign) and
--                   Q3 through the bounds, against an A.4 transcription
--   T87.A4.1        SIGN of each of the 9 candidates (Q3 = t - 4)
--   T87.A4.2        one-to-one vector -> context map, ctx - 4 in [0, 364]
--   NL.CtxNumbering ctx = 9*(9*Q1 + Q2) + Q3 + 4 after the flip (D5)
--   NL.Q3Bounds     Q3 = -4 + #{k : Ra < bound(k)} equals A.4(Rc - Ra)
--   NL.CtxStore     candidate C, B, N = latest regular exit of that lane
--                   and context (RAM, write register forwarding D6, clear)
--   NL.ClearSweep   512-cycle oClr sweep, addresses 0..511, before each image
--   NL.ExitTags     oExLane / oExFirstPix / oExLastPix / oExEol at E, oExValid
--                   only for slots issued since reset
--   NL.Reset        reset in any phase: outputs cleared, next session scored
--
-- Guideline notes:
--   G1  quantA4 is A.4 line by line on the TB's own NEAR/T; sign merge and
--       the D5 numbering are written from the spec text, not from the RTL.
--   G2  covRange: Ws 8 / max, H 1, NEAR 0 / max, T1 = NEAR+1, T3 = MAXVAL,
--       T1 = T2 = T3, D1 / D2 at +-MAXVAL, bounds at 1-MAXVAL / 2*MAXVAL,
--       candidate C / B / N at their min and max. RTL asserts: D1 / D2,
--       bounds, row, ctx, bank ranges; config and loop-contract assumptions.
--   G3  D1, D2 at +-MAXVAL; Q bins at -4; B at 1-RESET.
--   G4  covQ (Q1 x Q2, all 81), covQ3, covEdge (lane x stripe edge),
--       covFwd (candidate x write age: write register / RAM write-to-read
--       turnaround / RAM), covMode (regular / run / RI / junk exit).
--   G5  covRst: reset in CFG / CLEAR / RUN busy / RUN idle / DRAIN, then a
--       scored session; every reset is followed by up to 7 junk exits.
--   G6  see the code coverage report.
--------------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use work.nl_pkg.all;

library osvvm;
  context osvvm.OsvvmContext;

library tb_support;
  use tb_support.tb_support_pkg.all;

entity tb_nl_frontend_osvvm is
  generic (
    BITNESS : natural range 8 to 16 := 12
  );
end entity tb_nl_frontend_osvvm;

architecture sim of tb_nl_frontend_osvvm is

  constant MAX_W      : natural := 256;
  constant MAX_WS     : natural := MAX_W / 4;
  constant MAX_H      : natural := 6;
  constant MAX_IMG    : natural := 3;
  constant MAXVAL     : natural := 2 ** BITNESS - 1;
  constant NEAR_MAX   : natural := minimum(255, MAXVAL / 2);
  constant RESET_N    : natural := 64;                                   -- T.87 default RESET
  constant LAT        : natural := 7;                                    -- E - L
  constant CLK_PERIOD : time    := CLK_PERIOD_DEFAULT;

  constant N_SESS     : natural := 12;
  constant N_RST      : natural := 20;
  constant RP_CFG     : integer := 0;                                    -- waiting for iCfgDone
  constant RP_CLEAR   : integer := 1;                                    -- clear sweep
  constant RP_BUSY    : integer := 2;                                    -- slots in flight
  constant RP_IDLE    : integer := 3;                                    -- image open, nothing in flight
  constant RP_DRAIN   : integer := 4;                                    -- last slot issued, before the sweep

  -- covRange bins
  constant R_WS_MIN   : integer := 0;
  constant R_WS_MAX   : integer := 1;
  constant R_H1       : integer := 2;
  constant R_NEAR0    : integer := 3;
  constant R_NEARMAX  : integer := 4;
  constant R_T1MIN    : integer := 5;
  constant R_T3MAX    : integer := 6;
  constant R_TEQ      : integer := 7;
  constant R_D1MIN    : integer := 8;
  constant R_D1MAX    : integer := 9;
  constant R_D2MIN    : integer := 10;
  constant R_D2MAX    : integer := 11;
  constant R_BNDMIN   : integer := 12;
  constant R_BNDMAX   : integer := 13;
  constant R_CMIN     : integer := 14;
  constant R_CMAX     : integer := 15;
  constant R_BMIN     : integer := 16;
  constant R_BMAX     : integer := 17;
  constant R_NMIN     : integer := 18;
  constant R_NMAX     : integer := 19;
  constant R_IMGS     : integer := 20;

  signal clk       : std_logic := '0';
  signal rst       : std_logic := '1';
  signal iWidth    : unsigned(15 downto 0) := to_unsigned(32, 16);
  signal iHeight   : unsigned(15 downto 0) := to_unsigned(1, 16);
  signal iCfgDone  : std_logic := '0';
  signal iNear     : unsigned(7 downto 0) := (others => '0');
  signal iT1       : unsigned(BITNESS - 1 downto 0) := to_unsigned(1, BITNESS);
  signal iT2       : unsigned(BITNESS - 1 downto 0) := to_unsigned(1, BITNESS);
  signal iT3       : unsigned(BITNESS - 1 downto 0) := to_unsigned(1, BITNESS);
  signal iValid    : std_logic := '0';
  signal iPixel    : std_logic_vector(BITNESS - 1 downto 0) := (others => '0');
  signal oReady    : std_logic;
  signal iHold     : std_logic := '0';
  signal oClr      : std_logic;
  signal oClrAddr  : unsigned(8 downto 0);
  signal oValid    : std_logic;
  signal oFirstPix : std_logic;
  signal oFirstCol : std_logic;
  signal oEol      : std_logic;
  signal oIx       : unsigned(BITNESS - 1 downto 0);
  signal oRb       : unsigned(BITNESS - 1 downto 0);
  signal oRc       : unsigned(BITNESS - 1 downto 0);
  signal oQ12Zero  : std_logic;
  signal oQ3Bound  : t_signed_array(0 to 7)(BITNESS + 1 downto 0);
  signal oCandCtx  : t_unsigned_array(0 to 8)(CO_CTX_WIDTH - 1 downto 0);
  signal oCandSign : std_logic_vector(0 to 8);
  signal oCandC    : t_signed_array(0 to 8)(CO_C_WIDTH - 1 downto 0);
  signal oCandB    : t_signed_array(0 to 8)(6 downto 0);
  signal oCandN    : t_unsigned_array(0 to 8)(6 downto 0);
  signal iExValid  : std_logic := '0';
  signal iExMode   : std_logic_vector(1 downto 0) := "00";
  signal iExRx     : unsigned(BITNESS - 1 downto 0) := (others => '0');
  signal iExCtx    : unsigned(CO_CTX_WIDTH - 1 downto 0) := (others => '0');
  signal iExCq     : signed(CO_C_WIDTH - 1 downto 0) := (others => '0');
  signal iExBq     : signed(6 downto 0) := (others => '0');
  signal iExNq     : unsigned(6 downto 0) := (others => '0');
  signal oExValid  : std_logic;
  signal oExLane   : unsigned(1 downto 0);
  signal oExFirstPix : std_logic;
  signal oExLastPix  : std_logic;
  signal oExEol    : std_logic;

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

  dut : entity work.nl_frontend(behavioral)
    generic map (
      BITNESS         => BITNESS,
      MAX_IMAGE_WIDTH => MAX_W
    )
    port map (
      iClk        => clk,
      iRst        => rst,
      iWidth      => iWidth,
      iHeight     => iHeight,
      iCfgDone    => iCfgDone,
      iNear       => iNear,
      iT1         => iT1,
      iT2         => iT2,
      iT3         => iT3,
      iValid      => iValid,
      iPixel      => iPixel,
      oReady      => oReady,
      iHold       => iHold,
      oClr        => oClr,
      oClrAddr    => oClrAddr,
      oValid      => oValid,
      oFirstPix   => oFirstPix,
      oFirstCol   => oFirstCol,
      oEol        => oEol,
      oIx         => oIx,
      oRb         => oRb,
      oRc         => oRc,
      oQ12Zero    => oQ12Zero,
      oQ3Bound    => oQ3Bound,
      oCandCtx    => oCandCtx,
      oCandSign   => oCandSign,
      oCandC      => oCandC,
      oCandB      => oCandB,
      oCandN      => oCandN,
      iExValid    => iExValid,
      iExMode     => iExMode,
      iExRx       => iExRx,
      iExCtx      => iExCtx,
      iExCq       => iExCq,
      iExBq       => iExBq,
      iExNq       => iExNq,
      oExValid    => oExValid,
      oExLane     => oExLane,
      oExFirstPix => oExFirstPix,
      oExLastPix  => oExLastPix,
      oExEol      => oExEol
    );

  stim : process is

    type t_img is array (0 to MAX_IMG - 1, 0 to MAX_H - 1, 0 to MAX_W - 1) of natural;
    type t_rx is array (0 to 3, 0 to MAX_H - 1, 0 to MAX_WS - 1) of natural;
    type t_ctxs is array (0 to 3, 0 to 511) of integer;
    type t_map is array (0 to 511) of integer;
    type t_int9 is array (0 to 8) of integer;
    type t_int8 is array (0 to 7) of integer;

    type t_slot is record
      valid    : boolean;
      lane     : natural;
      row      : natural;
      col      : natural;
      firstPix : boolean;
      lastPix  : boolean;
      eol      : boolean;
      ix       : natural;
      ctx      : t_int9;
    end record t_slot;

    type t_hist is array (0 to 7) of t_slot;

    variable rv       : RandomPType;
    variable reqIssue : AlertLogIDType;
    variable reqNbr   : AlertLogIDType;
    variable reqA4    : AlertLogIDType;
    variable reqA41   : AlertLogIDType;
    variable reqA42   : AlertLogIDType;
    variable reqNum   : AlertLogIDType;
    variable reqQ3    : AlertLogIDType;
    variable reqStore : AlertLogIDType;
    variable reqClr   : AlertLogIDType;
    variable reqTag   : AlertLogIDType;
    variable reqRst   : AlertLogIDType;
    variable covQ     : CoverageIDType;
    variable covQ3    : CoverageIDType;
    variable covEdge  : CoverageIDType;
    variable covFwd   : CoverageIDType;
    variable covMode  : CoverageIDType;
    variable covRst   : CoverageIDType;
    variable covRange : CoverageIDType;

    -- Session config
    variable cfgWs    : natural := 8;
    variable cfgH     : natural := 1;
    variable cfgNear  : natural := 0;
    variable cfgT1    : natural := 1;
    variable cfgT2    : natural := 1;
    variable cfgT3    : natural := 1;
    variable nImg     : natural := 0;
    variable img      : t_img;

    -- Model
    variable rxs      : t_rx;
    variable stC      : t_ctxs;
    variable stB      : t_ctxs;
    variable stN      : t_ctxs;
    variable lastWr   : t_ctxs;
    variable ctxMap   : t_map := (others => -1);
    variable hist     : t_hist;

    -- Pointers and state
    variable cycles   : integer := 0;
    variable inRst    : boolean := true;
    variable postRst  : boolean := false;
    variable cfgLeft  : natural := 0;
    variable staleLeft : natural := 0;
    variable issImg   : natural := 0;
    variable issRow   : natural := 0;
    variable issCol   : natural := 0;
    variable issLane  : natural := 0;
    variable lastL    : integer := 0;
    variable rasImg   : natural := 0;
    variable rasRow   : natural := 0;
    variable rasCol   : natural := 0;
    variable gapLeft  : natural := 0;
    variable holdLeft : natural := 0;
    variable clrCnt   : natural := 0;
    variable clrDone  : boolean := false;
    variable inRun    : boolean := false;
    variable draining : boolean := false;
    variable imgsDone : natural := 0;
    variable expExV   : std_logic := '0';
    variable ph       : integer;
    variable hit      : boolean;

    -- A.4, line by line.
    impure function quant_a4 (
      d : integer
    ) return integer is
    begin
      if (d <= -cfgT3) then
        return -4;
      elsif (d <= -cfgT2) then
        return -3;
      elsif (d <= -cfgT1) then
        return -2;
      elsif (d < -cfgNear) then
        return -1;
      elsif (d <= cfgNear) then
        return 0;
      elsif (d < cfgT1) then
        return 1;
      elsif (d < cfgT2) then
        return 2;
      elsif (d < cfgT3) then
        return 3;
      else
        return 4;
      end if;
    end function quant_a4;

    -- Difference landing in quantization region q (q in -4..4), 0 if empty.
    impure function diff_for (
      q : integer
    ) return integer is
      variable lo, hi : integer;
    begin
      case abs(q) is
        when 0 => lo := 0;           hi := cfgNear;
        when 1 => lo := cfgNear + 1; hi := cfgT1 - 1;
        when 2 => lo := cfgT1;       hi := cfgT2 - 1;
        when 3 => lo := cfgT2;       hi := cfgT3 - 1;
        when others => lo := cfgT3;  hi := MAXVAL;
      end case;
      if (hi < lo) then
        return 0;
      elsif (q < 0) then
        return -rv.RandInt(lo, hi);
      elsif (q = 0) then
        return rv.RandInt(-hi, hi);
      else
        return rv.RandInt(lo, hi);
      end if;
    end function diff_for;

    impure function clamp (
      v : integer
    ) return natural is
    begin
      return maximum(0, minimum(MAXVAL, v));
    end function clamp;

    procedure gen_images is
      variable v : integer;
      variable d : integer;
      variable kind : integer;
    begin
      for i in 0 to nImg - 1 loop
        for r in 0 to cfgH - 1 loop
          kind := rv.DistValInt(((0, 6), (1, 1), (2, 1)));               -- walk / flat / extremes
          v    := rv.DistValInt(((0, 1), (MAXVAL, 1), (rv.RandInt(0, MAXVAL), 4)));
          for c in 0 to 4 * cfgWs - 1 loop
            case kind is
              when 1 =>
                null;
              when 2 =>
                v := MAXVAL * rv.RandInt(0, 1);
              when others =>
                d := diff_for(rv.RandInt(-4, 4));
                if (v + d < 0 or v + d > MAXVAL) then
                  d := -d;
                end if;
                v := clamp(v + d);
            end case;
            img(i, r, c) := v;
          end loop;
        end loop;
      end loop;
    end procedure gen_images;

    procedure clear_model is
    begin
      for k in 0 to 7 loop
        hist(k).valid := false;
      end loop;
      issImg   := 0;
      issRow   := 0;
      issCol   := 0;
      issLane  := 0;
      rasImg   := 0;
      rasRow   := 0;
      rasCol   := 0;
      clrCnt   := 0;
      clrDone  := false;
      inRun    := false;
      draining := false;
      gapLeft  := 0;
      holdLeft := 0;
    end procedure clear_model;

    impure function in_flight return boolean is
    begin
      if (oValid = '1') then
        return true;
      end if;
      for k in 0 to 7 loop
        if (hist(k).valid) then
          return true;
        end if;
      end loop;
      return false;
    end function in_flight;

    impure function session_done return boolean is
    begin
      return issImg >= nImg and not in_flight;
    end function session_done;

    -- Slot at L: everything the loop is given, against the model.
    procedure check_l is
      variable lane, row, col : natural;
      variable rb, rc, rd     : integer;
      variable d1, d2, q1, q2 : integer;
      variable s, a, b, c, q3 : integer;
      variable e, code, dist  : integer;
      variable dc, dsign      : integer;
      variable dq1, dq2       : integer;
      variable bnd            : t_int8;
      variable ra, cnt        : integer;
      variable slot           : t_slot;
      variable vC, vB, vN     : integer;
    begin
      AffirmIf(reqIssue, issImg < nImg, "slot beyond the session's images");
      if (issImg >= nImg) then
        return;
      end if;
      lane := issLane;
      row  := issRow;
      col  := issCol;
      if (lane > 0) then
        AffirmIfEqual(reqIssue, cycles, lastL + 1, "column group lanes on consecutive cycles");
      end if;
      lastL := cycles;

      -- First slot of an image: sweep done, store at its reset value.
      if (row = 0 and col = 0 and lane = 0) then
        AffirmIf(reqClr, clrDone, "image " & to_string(issImg) & " issued after the clear sweep");
        clrDone := false;
        for l in 0 to 3 loop
          for x in 0 to 511 loop
            stC(l, x)    := 0;
            stB(l, x)    := 0;
            stN(l, x)    := 1;
            lastWr(l, x) := -100;
          end loop;
        end loop;
      end if;

      AffirmIfEqual(reqIssue, checked_integer(oIx), img(issImg, row, lane * cfgWs + col),
                    "Ix lane " & to_string(lane) & " row " & to_string(row) & " col " & to_string(col));
      AffirmIfEqual(reqIssue, oFirstPix, sl(row = 0 and col = 0), "first pixel flag");
      AffirmIfEqual(reqIssue, oFirstCol, sl(col = 0), "first column flag");
      AffirmIfEqual(reqIssue, oEol, sl(col = cfgWs - 1), "end of line flag");

      -- Neighbours, T.87 edge rules within the stripe.
      if (row = 0) then
        rb := 0;
        rc := 0;
        rd := 0;
      else
        rb := rxs(lane, row - 1, col);
        if (col = 0) then
          if (row >= 2) then
            rc := rxs(lane, row - 2, 0);
          else
            rc := 0;
          end if;
        else
          rc := rxs(lane, row - 1, col - 1);
        end if;
        if (col = cfgWs - 1) then
          rd := rb;
        else
          rd := rxs(lane, row - 1, col + 1);
        end if;
      end if;
      AffirmIfEqual(reqNbr, checked_integer(oRb), rb, "Rb");
      AffirmIfEqual(reqNbr, checked_integer(oRc), rc, "Rc");

      -- A.1 gradients, A.4 on D1, D2.
      d1 := rd - rb;
      d2 := rb - rc;
      q1 := quant_a4(d1);
      q2 := quant_a4(d2);
      AffirmIfEqual(reqA4, oQ12Zero, sl(q1 = 0 and q2 = 0), "Q1 = Q2 = 0 flag");
      -- Candidate 4 (Q3 = 0) decoded back to (Q1, Q2).
      dc    := checked_integer(oCandCtx(4));
      dsign := 1 - 2 * checked_bit(oCandSign(4));
      dq1   := (dc / 9 + 4) / 9;
      dq2   := dc / 9 - 9 * dq1;
      AffirmIf(reqA4, dsign * dq1 = q1 and dsign * dq2 = q2,
               "Q1, Q2 = " & to_string(dsign * dq1) & ", " & to_string(dsign * dq2) &
               " expected " & to_string(q1) & ", " & to_string(q2));

      for t in 0 to 8 loop
        q3 := t - 4;
        -- A.4.1: first non-zero element negative -> reverse, SIGN = -1.
        if (q1 < 0 or (q1 = 0 and q2 < 0) or (q1 = 0 and q2 = 0 and q3 < 0)) then
          s := -1;
        else
          s := 1;
        end if;
        a := s * q1;
        b := s * q2;
        c := s * q3;
        AffirmIfEqual(reqA41, oCandSign(t), sl(s = -1), "SIGN of candidate " & to_string(t));
        -- D5 numbering.
        e := 9 * (9 * a + b) + c + 4;
        AffirmIfEqual(reqNum, checked_integer(oCandCtx(t)), e, "ctx of candidate " & to_string(t));
        -- A.4.2: one-to-one, Q = ctx - 4 in [0, 364].
        code := (a + 4) * 81 + (b + 4) * 9 + (c + 4);
        AffirmIf(reqA42, e - 4 >= 0 and e - 4 <= 364, "Q = " & to_string(e - 4) & " outside [0, 364]");
        if (e >= 0 and e <= 511) then
          if (ctxMap(e) = -1) then
            ctxMap(e) := code;
          end if;
          AffirmIfEqual(reqA42, ctxMap(e), code, "context " & to_string(e) & " shared by two vectors");
        end if;
        -- Store: latest regular exit of this lane and context.
        if (e >= 0 and e <= 511) then
          vC := checked_integer(oCandC(t));
          vB := checked_integer(oCandB(t));
          vN := checked_integer(oCandN(t));
          AffirmIf(reqStore, vC = stC(lane, e) and vB = stB(lane, e) and vN = stN(lane, e),
                   "lane " & to_string(lane) & " ctx " & to_string(e) & ": C, B, N = " &
                   to_string(vC) & ", " & to_string(vB) & ", " & to_string(vN) & " expected " &
                   to_string(stC(lane, e)) & ", " & to_string(stB(lane, e)) & ", " & to_string(stN(lane, e)));
          dist := cycles - lastWr(lane, e);
          if (dist = 1 or dist = 5 or dist = 9) then
            ICover(covFwd, (t, (dist - 1) / 4));
          end if;
          if (vC = -128) then ICover(covRange, R_CMIN); end if;
          if (vC = 127) then ICover(covRange, R_CMAX); end if;
          if (vB = 1 - RESET_N) then ICover(covRange, R_BMIN); end if;
          if (vB = 0) then ICover(covRange, R_BMAX); end if;
          if (vN = 1) then ICover(covRange, R_NMIN); end if;
          if (vN = RESET_N) then ICover(covRange, R_NMAX); end if;
        end if;
        slot.ctx(t) := e;
      end loop;

      -- Q3 through the bounds: -4 + #{k : Ra < bound(k)} = A.4(Rc - Ra).
      for k in 0 to 7 loop
        bnd(k) := checked_integer(oQ3Bound(k));
        if (bnd(k) = 1 - MAXVAL) then ICover(covRange, R_BNDMIN); end if;
        if (bnd(k) = 2 * MAXVAL) then ICover(covRange, R_BNDMAX); end if;
      end loop;
      for p in -1 to 17 loop
        if (p = -1) then
          ra := rv.RandInt(0, MAXVAL);
        elsif (p = 16) then
          ra := 0;
        elsif (p = 17) then
          ra := MAXVAL;
        else
          ra := clamp(bnd(p / 2) - (p mod 2));
        end if;
        cnt := 0;
        for k in 0 to 7 loop
          if (ra < bnd(k)) then
            cnt := cnt + 1;
          end if;
        end loop;
        AffirmIfEqual(reqQ3, cnt - 4, quant_a4(rc - ra), "Q3 for Ra = " & to_string(ra) & ", Rc = " & to_string(rc));
        ICover(covQ3, quant_a4(rc - ra));
      end loop;

      -- Coverage
      ICover(covQ, (q1, q2));
      if (d1 = -MAXVAL) then ICover(covRange, R_D1MIN); end if;
      if (d1 = MAXVAL) then ICover(covRange, R_D1MAX); end if;
      if (d2 = -MAXVAL) then ICover(covRange, R_D2MIN); end if;
      if (d2 = MAXVAL) then ICover(covRange, R_D2MAX); end if;
      if (row = 0) then ICover(covEdge, (lane, 4)); end if;
      if (row > 0 and col = 0) then ICover(covEdge, (lane, 0)); end if;
      if (row > 0 and col = cfgWs - 1) then ICover(covEdge, (lane, 1)); end if;
      if (row = 0 and col = 0) then ICover(covEdge, (lane, 2)); end if;
      if (row = cfgH - 1 and col = cfgWs - 1) then ICover(covEdge, (lane, 3)); end if;
      if (row >= 2 and col = 0) then ICover(covEdge, (lane, 5)); end if;

      slot.valid    := true;
      slot.lane     := lane;
      slot.row      := row;
      slot.col      := col;
      slot.firstPix := row = 0 and col = 0;
      slot.lastPix  := row = cfgH - 1 and col = cfgWs - 1;
      slot.eol      := col = cfgWs - 1;
      slot.ix       := img(issImg, row, lane * cfgWs + col);
      hist(cycles mod 8) := slot;

      -- Next expected slot.
      if (issLane < 3) then
        issLane := issLane + 1;
      else
        issLane := 0;
        if (issCol < cfgWs - 1) then
          issCol := issCol + 1;
        else
          issCol := 0;
          if (issRow < cfgH - 1) then
            issRow := issRow + 1;
          else
            issRow   := 0;
            issImg   := issImg + 1;
            imgsDone := imgsDone + 1;
            inRun    := false;
            draining := true;
          end if;
        end if;
      end if;
    end procedure check_l;

    -- Outputs of this cycle.
    procedure monitor is
    begin
      hist(cycles mod 8).valid := false;
      if (postRst) then
        postRst := false;
        AffirmIf(reqRst, oValid = '0' and oClr = '0' and oReady = '1',
                 "after reset: no slot, no sweep, ready");
      end if;

      if (oClr = '1') then
        AffirmIfEqual(reqClr, checked_integer(oClrAddr), clrCnt, "clear address");
        AffirmIf(reqClr, oValid = '0', "no slot during the clear sweep");
        clrCnt   := clrCnt + 1;
        draining := false;
      elsif (clrCnt > 0) then
        AffirmIfEqual(reqClr, clrCnt, 512, "clear sweep length");
        clrCnt  := 0;
        clrDone := true;
        inRun   := issImg < nImg;
      end if;

      if (oValid = '1') then
        check_l;
      end if;
    end procedure monitor;

    -- Inputs for this cycle.
    procedure drive is
      variable e    : t_slot;
      variable mode : integer;
      variable rx   : integer;
      variable ctx  : integer;
      variable vC, vB, vN : integer;
      variable junk : boolean;
    begin
      -- Loop exit: the slot that was at L 7 cycles ago, or junk.
      e := hist((cycles + 1) mod 8);
      if (not inRst and e.valid) then
        AffirmIfEqual(reqTag, checked_integer(oExLane), e.lane, "oExLane");
        AffirmIfEqual(reqTag, oExFirstPix, sl(e.firstPix), "oExFirstPix");
        AffirmIfEqual(reqTag, oExLastPix, sl(e.lastPix), "oExLastPix");
        AffirmIfEqual(reqTag, oExEol, sl(e.eol), "oExEol");
        hist((cycles + 1) mod 8).valid := false;
        mode := rv.DistValInt(((0, 7), (1, 1), (2, 1)));
        case rv.DistValInt(((0, 16), (1, 3), (2, 1))) is
          when 0 => rx := e.ix;
          when 1 => rx := clamp(e.ix + rv.RandInt(-cfgNear, cfgNear));
          when others => rx := MAXVAL * rv.RandInt(0, 1);
        end case;
        rxs(e.lane, e.row, e.col) := rx;
        vN := rv.DistValInt(((1, 1), (RESET_N, 1), (rv.RandInt(1, RESET_N), 2)));
        vB := rv.DistValInt(((1 - vN, 1), (0, 1), (rv.RandInt(1 - vN, 0), 2)));
        vC := rv.DistValInt(((-128, 1), (127, 1), (rv.RandInt(-128, 127), 2)));
        if (mode = 0) then
          ctx := e.ctx(rv.RandInt(0, 8));
          stC(e.lane, ctx)    := vC;
          stB(e.lane, ctx)    := vB;
          stN(e.lane, ctx)    := vN;
          lastWr(e.lane, ctx) := cycles;
        elsif (rv.RandInt(0, 1) = 0) then
          ctx := e.ctx(rv.RandInt(0, 8));                              -- not written: mode gate
        else
          ctx := mode - 1;                                             -- RI contexts 0 / 1
        end if;
        ICover(covMode, mode);
        iExValid <= '1';
        expExV   := '1';
      else
        -- No slot at E: junk, dropped (always right after a reset, as the
        -- loop's in-flight slots would be).
        junk     := staleLeft > 0 or inRst or rv.DistValInt(((1, 1), (0, 3))) = 1;
        mode     := rv.DistValInt(((0, 2), (1, 1), (2, 1)));
        rx       := rv.RandInt(0, MAXVAL);
        ctx      := rv.RandInt(0, 368);
        vC       := rv.RandInt(-128, 127);
        vN       := rv.RandInt(1, RESET_N);
        vB       := rv.RandInt(1 - vN, 0);
        iExValid <= sl(junk);
        expExV   := '0';
        if (junk and not inRst) then
          ICover(covMode, 3);
        end if;
      end if;
      if (staleLeft > 0) then
        staleLeft := staleLeft - 1;
      end if;
      iExMode <= std_logic_vector(to_unsigned(mode, 2));
      iExRx   <= to_unsigned(rx, BITNESS);
      iExCtx  <= to_unsigned(ctx, CO_CTX_WIDTH);
      iExCq   <= to_signed(vC, CO_C_WIDTH);
      iExBq   <= to_signed(vB, 7);
      iExNq   <= to_unsigned(vN, 7);

      if (inRst) then
        iValid <= '0';
        iHold  <= '0';
        return;
      end if;

      -- Config handshake.
      if (cfgLeft > 0) then
        cfgLeft := cfgLeft - 1;
      end if;
      iCfgDone <= sl(cfgLeft = 0);

      -- Hold bursts.
      if (holdLeft > 0) then
        holdLeft := holdLeft - 1;
      elsif (rv.DistValInt(((1, 1), (0, 60))) = 1) then
        holdLeft := rv.RandInt(1, 40);
      end if;
      iHold <= sl(holdLeft > 0);

      -- Raster, with gaps (oReady is settled: registered state, iRst low).
      if (gapLeft > 0) then
        gapLeft := gapLeft - 1;
        iValid  <= '0';
      elsif (rv.DistValInt(((1, 1), (0, 80))) = 1) then
        gapLeft := rv.RandInt(5, 120);
        iValid  <= '0';
      elsif (rasImg < nImg and rv.DistValInt(((1, 4), (0, 1))) = 1) then
        iValid <= '1';
        iPixel <= std_logic_vector(to_unsigned(img(rasImg, rasRow, rasCol), BITNESS));
        if (oReady = '1') then
          if (rasCol < 4 * cfgWs - 1) then
            rasCol := rasCol + 1;
          else
            rasCol := 0;
            if (rasRow < cfgH - 1) then
              rasRow := rasRow + 1;
            else
              rasRow := 0;
              rasImg := rasImg + 1;
            end if;
          end if;
        end if;
      else
        iValid <= '0';
        iPixel <= rv.RandSlv(BITNESS);
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
      wait for 1 ns;
      if (not inRst) then
        AffirmIfEqual(reqTag, oExValid, expExV, "oExValid only for slots issued since reset");
      end if;
    end procedure step;

    -- Reset with a new config and images; the loop's in-flight exits keep
    -- coming during and after it.
    procedure begin_session (n : positive) is
      variable g : natural;
    begin
      cfgWs   := rv.DistValInt(((8, 1), (MAX_WS, 1), (rv.RandInt(9, MAX_WS - 1), 2)));
      cfgH    := rv.DistValInt(((1, 1), (MAX_H, 1), (rv.RandInt(2, MAX_H - 1), 2)));
      cfgNear := rv.DistValInt(((0, 1), (NEAR_MAX, 1), (rv.RandInt(1, 7), 1), (rv.RandInt(0, NEAR_MAX), 1)));
      g       := rv.RandInt(1, maximum(1, MAXVAL / 8));
      cfgT1   := minimum(MAXVAL, cfgNear + rv.RandInt(1, g));
      cfgT2   := minimum(MAXVAL, cfgT1 + rv.RandInt(0, g));
      cfgT3   := minimum(MAXVAL, cfgT2 + rv.RandInt(0, g));
      case rv.DistValInt(((0, 3), (1, 1), (2, 1), (3, 1))) is
        when 1 => cfgT1 := cfgNear + 1;
        when 2 => cfgT3 := MAXVAL;
        when 3 => cfgT2 := cfgT1; cfgT3 := cfgT1;
        when others => null;
      end case;
      nImg := rv.RandInt(1, MAX_IMG);
      gen_images;

      if (cfgWs = 8) then ICover(covRange, R_WS_MIN); end if;
      if (cfgWs = MAX_WS) then ICover(covRange, R_WS_MAX); end if;
      if (cfgH = 1) then ICover(covRange, R_H1); end if;
      if (cfgNear = 0) then ICover(covRange, R_NEAR0); end if;
      if (cfgNear = NEAR_MAX) then ICover(covRange, R_NEARMAX); end if;
      if (cfgT1 = cfgNear + 1) then ICover(covRange, R_T1MIN); end if;
      if (cfgT3 = MAXVAL) then ICover(covRange, R_T3MAX); end if;
      if (cfgT1 = cfgT3) then ICover(covRange, R_TEQ); end if;
      if (nImg = MAX_IMG) then ICover(covRange, R_IMGS); end if;

      inRst    := true;
      rst      <= '1';
      iCfgDone <= '0';
      iWidth   <= to_unsigned(4 * cfgWs, 16);
      iHeight  <= to_unsigned(cfgH, 16);
      iNear    <= to_unsigned(cfgNear, 8);
      iT1      <= to_unsigned(cfgT1, BITNESS);
      iT2      <= to_unsigned(cfgT2, BITNESS);
      iT3      <= to_unsigned(cfgT3, BITNESS);
      clear_model;
      for i in 1 to n loop
        step;
      end loop;
      rst       <= '0';
      inRst     := false;
      postRst   := true;
      cfgLeft   := rv.RandInt(1, 12);
      staleLeft := LAT;
      clear_model;
    end procedure begin_session;

    impure function phase_hit (p : integer) return boolean is
    begin
      case p is
        when RP_CFG   => return cfgLeft > 0;
        when RP_CLEAR => return oClr = '1';
        when RP_BUSY  => return in_flight;
        when RP_IDLE  => return inRun and not in_flight;
        when others   => return draining and oClr = '0';
      end case;
    end function phase_hit;

    procedure run_session is
    begin
      while not session_done loop
        step;
        exit when cycles > 5_000_000;
      end loop;
      AffirmIf(GetAlertLogID("Completion"), session_done, "session issued every image");
    end procedure run_session;

  begin

    SetAlertLogName("tb_nl_frontend_osvvm");
    SetLogEnable(PASSED, FALSE);
    rv.InitSeed(rv'instance_name);
    reqIssue := GetReqID("NL.LaneIssue", 1000);
    reqNbr   := GetReqID("NL.Neighbours", 1000);
    reqA4    := GetReqID("T87.A4", 1000);
    reqA41   := GetReqID("T87.A4.1", 1000);
    reqA42   := GetReqID("T87.A4.2", 1000);
    reqNum   := GetReqID("NL.CtxNumbering", 1000);
    reqQ3    := GetReqID("NL.Q3Bounds", 1000);
    reqStore := GetReqID("NL.CtxStore", 1000);
    reqClr   := GetReqID("NL.ClearSweep", 100);
    reqTag   := GetReqID("NL.ExitTags", 1000);
    reqRst   := GetReqID("NL.Reset", N_RST);

    covQ := NewID("Q1 x Q2");
    SetFieldName(covQ, "Q1", "Q2");
    AddCross(covQ, GenBin(-4, 4), GenBin(-4, 4));

    covQ3 := NewID("Q3 from the bounds");
    AddBins(covQ3, GenBin(-4, 4));

    covEdge := NewID("lane x stripe edge");
    SetFieldName(covEdge, "lane", "edge");
    for l in 0 to 3 loop
      AddCross(covEdge, "lane=" & to_string(l) & " / first column", GenBin(l), GenBin(0));
      AddCross(covEdge, "lane=" & to_string(l) & " / end of line", GenBin(l), GenBin(1));
      AddCross(covEdge, "lane=" & to_string(l) & " / first pixel", GenBin(l), GenBin(2));
      AddCross(covEdge, "lane=" & to_string(l) & " / last pixel", GenBin(l), GenBin(3));
      AddCross(covEdge, "lane=" & to_string(l) & " / line 0", GenBin(l), GenBin(4));
      AddCross(covEdge, "lane=" & to_string(l) & " / Rc from column 0 two lines up", GenBin(l), GenBin(5));
    end loop;

    covFwd := NewID("candidate x write age");
    SetFieldName(covFwd, "candidate", "age");
    for t in 0 to 8 loop
      AddCross(covFwd, "t=" & to_string(t) & " / previous lane pixel (write register)", GenBin(t), GenBin(0));
      AddCross(covFwd, "t=" & to_string(t) & " / 2 pixels back (RAM write-to-read)", GenBin(t), GenBin(1));
      AddCross(covFwd, "t=" & to_string(t) & " / 3 pixels back (RAM)", GenBin(t), GenBin(2));
    end loop;

    covMode := NewID("loop exit kind");
    AddBins(covMode, "regular, written", GenBin(0));
    AddBins(covMode, "run hit, not written", GenBin(1));
    AddBins(covMode, "run interruption, not written", GenBin(2));
    AddBins(covMode, "no slot at E, dropped", GenBin(3));

    covRst := NewID("reset phase");
    AddBins(covRst, "reset in CFG", 3, GenBin(RP_CFG));
    AddBins(covRst, "reset during the clear sweep", 3, GenBin(RP_CLEAR));
    AddBins(covRst, "reset with slots in flight", 3, GenBin(RP_BUSY));
    AddBins(covRst, "reset with the image open, pipeline empty", 3, GenBin(RP_IDLE));
    AddBins(covRst, "reset in DRAIN", 3, GenBin(RP_DRAIN));

    covRange := NewID("value ranges");
    AddBins(covRange, "Ws = 8 (min)", GenBin(R_WS_MIN));
    AddBins(covRange, "Ws = max", GenBin(R_WS_MAX));
    AddBins(covRange, "H = 1", GenBin(R_H1));
    AddBins(covRange, "NEAR = 0", GenBin(R_NEAR0));
    AddBins(covRange, "NEAR = max", GenBin(R_NEARMAX));
    AddBins(covRange, "T1 = NEAR+1", GenBin(R_T1MIN));
    AddBins(covRange, "T3 = MAXVAL", GenBin(R_T3MAX));
    AddBins(covRange, "T1 = T2 = T3", GenBin(R_TEQ));
    AddBins(covRange, "D1 = -MAXVAL", GenBin(R_D1MIN));
    AddBins(covRange, "D1 = MAXVAL", GenBin(R_D1MAX));
    AddBins(covRange, "D2 = -MAXVAL", GenBin(R_D2MIN));
    AddBins(covRange, "D2 = MAXVAL", GenBin(R_D2MAX));
    AddBins(covRange, "bound = 1-MAXVAL", GenBin(R_BNDMIN));
    AddBins(covRange, "bound = 2*MAXVAL", GenBin(R_BNDMAX));
    AddBins(covRange, "C = -128", GenBin(R_CMIN));
    AddBins(covRange, "C = 127", GenBin(R_CMAX));
    AddBins(covRange, "B = 1-RESET", GenBin(R_BMIN));
    AddBins(covRange, "B = 0", GenBin(R_BMAX));
    AddBins(covRange, "N = 1", GenBin(R_NMIN));
    AddBins(covRange, "N = RESET", GenBin(R_NMAX));
    AddBins(covRange, "3 images in a session", GenBin(R_IMGS));

    for s in 0 to N_SESS - 1 loop
      begin_session(rv.RandInt(1, 6));
      run_session;
    end loop;

    -- Reset rounds: traffic until the target phase, reset, scored session.
    for r in 0 to N_RST - 1 loop
      if (r < 5 * 3) then
        ph := r mod 5;
      else
        ph := rv.RandInt(RP_CFG, RP_DRAIN);
      end if;
      begin_session(rv.RandInt(1, 6));
      hit := false;
      while not hit loop
        step;
        hit := phase_hit(ph) and (ph = RP_CFG or rv.DistValInt(((1, 1), (0, 7))) = 1);
        if (session_done and not hit) then
          begin_session(rv.RandInt(1, 6));                             -- phase not reached, retry
        end if;
        exit when cycles > 5_000_000;
      end loop;
      AffirmIf(reqRst, hit, "reset round " & to_string(r) & ": phase " & to_string(ph) & " reached");
      ICover(covRst, ph);
      begin_session(rv.RandInt(1, 6));
      run_session;
    end loop;

    AffirmIf(GetAlertLogID("Completion"), imgsDone > 0, "images issued");

    WriteBin(covQ);
    WriteBin(covQ3);
    WriteBin(covEdge);
    WriteBin(covFwd);
    WriteBin(covMode);
    WriteBin(covRst);
    WriteBin(covRange);
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covQ), "Q1 x Q2 coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covQ3), "Q3 coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covEdge), "edge coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covFwd), "write age coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covMode), "exit kind coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covRst), "reset-phase coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covRange), "value range coverage closed");

    end_of_test("tb_nl_frontend_osvvm");
    wait;

  end process stim;

  watchdog : process is
  begin
    wait for 100 ms;
    Alert(GetAlertLogID("Watchdog"), "tb_nl_frontend_osvvm: watchdog timeout", FAILURE);
    std.env.stop;
  end process watchdog;

end architecture sim;
