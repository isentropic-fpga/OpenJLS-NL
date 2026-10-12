-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--

----------------------------------------------------------------------------------
-- Engineer:    Vitor Mendes Camilo
--
-- Module Name: tb_nl_loop
--
-- Notes:
--              Random-stimulus check of nl_loop against a per-stripe reference
--              written from T.87 A.2-A.19 (integer division, full context memory).
--              The context candidates fed to the DUT come from a memory that lacks
--              each stripe's most recent regular update, which the DUT forwards.
--              Slot k feeds lane k mod 4; STRIPES lanes are used (1 = stalled stream).
--
----------------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.math_real.all;
  use work.nl_pkg.all;

entity tb_nl_loop is
  generic (
    BITNESS : natural  := 12;
    STRIPES : positive range 1 to 4 := 1;
    NEAR    : natural  := 3;
    CYCLES  : positive := 200000;
    SEED    : positive := 1
  );
end entity tb_nl_loop;

architecture sim of tb_nl_loop is

  constant RESET   : natural := 64;
  constant NW      : natural := 7;
  constant BW      : natural := 7;
  constant MAXVAL  : natural := 2 ** BITNESS - 1;
  constant D       : natural := 2 * NEAR + 1;
  constant RANGE_V : natural := (MAXVAL + 2 * NEAR) / D + 1;
  constant RECIP   : natural := (2 ** (BITNESS + 10) + D - 1) / D;

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
    cMem    : t_ctx;
    bMem    : t_ctx;
    nMem    : t_ctx;
    pend    : boolean;
    pCtx    : integer;
    pC      : integer;
    pB      : integer;
    pN      : integer;
  end record t_stripe;

  type t_stripes is array (0 to 3) of t_stripe;

  type t_exp is record
    mode   : integer;
    riType : boolean;
    rx     : integer;
    errval : integer;
    ctx    : integer;
    c      : integer;
    b      : integer;
    n      : integer;
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

begin

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
      oNq       => oNq
    );

  stim_proc : process is

    variable s1, s2   : positive := SEED;
    variable st       : t_stripes;
    variable slot     : natural := 0;
    variable s        : natural;
    variable ix, rb, rc, ra : integer;
    variable q1, q2, q3     : integer;
    variable v1, v2, v3     : integer;
    variable sg       : integer;
    variable ctx      : integer;
    variable cv, bv, nv : integer;
    variable px, e, rx : integer;
    variable ex       : t_exp;
    variable ri       : boolean;
    variable nRun, nRi, nReg, nFwd : natural := 0;
    variable expQ     : t_exp_q;
    variable qWr      : natural := 0;
    variable qRd      : natural := 0;

    impure function rnd (
      n : positive
    ) return natural is
      variable r : real;
    begin

      uniform(s1, s2, r);
      return natural(floor(r * real(n)));

    end function rnd;

    impure function near_val (
      center, spread : integer
    ) return integer is
    begin

      return clampv(center + rnd(2 * spread + 1) - spread);

    end function near_val;

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

    -- Outputs sampled on the clock edge, before this edge's update
    procedure check_out is
      variable ex : t_exp;
    begin

      if (oValid = '1') then
        ex := expQ(qRd mod 16);
        assert to_integer(unsigned(oMode)) = ex.mode
          report "mode mismatch at " & integer'image(qRd) & ": got " &
                 integer'image(to_integer(unsigned(oMode))) & " exp " & integer'image(ex.mode)
          severity failure;
        assert to_integer(oRx) = ex.rx
          report "Rx mismatch at " & integer'image(qRd) & ": got " &
                 integer'image(to_integer(oRx)) & " exp " & integer'image(ex.rx)
          severity failure;
        if (ex.mode /= 1) then
          assert to_integer(oErrval) = ex.errval
            report "Errval mismatch at " & integer'image(qRd) & ": got " &
                   integer'image(to_integer(oErrval)) & " exp " & integer'image(ex.errval)
            severity failure;
        end if;
        if (ex.mode = 0) then
          assert to_integer(oCtx) = ex.ctx and to_integer(oCq) = ex.c and
                 to_integer(oBq) = ex.b and to_integer(oNq) = ex.n
            report "context mismatch at " & integer'image(qRd) & ": got ctx/C/B/N " &
                   integer'image(to_integer(oCtx)) & "/" & integer'image(to_integer(oCq)) & "/" &
                   integer'image(to_integer(oBq)) & "/" & integer'image(to_integer(oNq)) & " exp " &
                   integer'image(ex.ctx) & "/" & integer'image(ex.c) & "/" &
                   integer'image(ex.b) & "/" & integer'image(ex.n)
            severity failure;
        end if;
        if (ex.mode = 2) then
          assert (oRiType = '1') = ex.riType report "RItype mismatch" severity failure;
        end if;
        qRd := qRd + 1;
      end if;

    end procedure check_out;

  begin

    for i in 0 to 3 loop
      st(i).first := true;
      st(i).ra    := 0;
      st(i).inRun := false;
    end loop;

    wait until rising_edge(clk);

    for cyc in 0 to CYCLES - 1 loop
      s := slot mod 4;
      slot := slot + 1;
      iValid <= '0';

      if (s < STRIPES and rnd(10) /= 0) then
        -- New stripe image
        if (st(s).first) then
          st(s).first := false;
          st(s).width := 4 + rnd(40);
          st(s).col   := 0;
          st(s).inRun := false;
          st(s).pend  := false;
          st(s).cMem  := (others => 0);
          st(s).bMem  := (others => 0);
          st(s).nMem  := (others => 1);
          iFirstPix   <= '1';
        else
          iFirstPix <= '0';
        end if;

        -- Pixels: mostly smooth, sometimes random, to exercise runs and all Q3
        if (rnd(4) = 0) then
          ix := rnd(MAXVAL + 1);
          rb := rnd(MAXVAL + 1);
          rc := rnd(MAXVAL + 1);
        else
          ix := near_val(st(s).ra, 2 * NEAR + 3);
          rb := near_val(ix, NEAR + 2);
          rc := near_val(rb, T2);
        end if;
        if (rnd(2) = 0) then
          q1 := 0; q2 := 0;
        else
          q1 := rnd(9) - 4; q2 := rnd(9) - 4;
        end if;

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

        if (st(s).inRun or (q1 = 0 and q2 = 0 and q3 = 0)) then
          if (abs(ix - ra) <= NEAR) then
            ex.mode := 1;
            rx      := ra;
            e       := 0;
            nRun    := nRun + 1;
            st(s).inRun := st(s).col /= st(s).width - 1;
          else
            ex.mode := 2;
            ri := abs(ra - rb) <= NEAR;
            ex.riType := ri;
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
            e  := quantize(e);
            rx := clampv(px + sg * e * D);
            e  := mod_range(e);
            nRi := nRi + 1;
            st(s).inRun := false;
          end if;
        else
          ex.mode := 0;
          nReg    := nReg + 1;
          ctx_of(q1, q2, q3, ctx, sg);
          if (st(s).pend and st(s).pCtx = ctx) then
            nFwd := nFwd + 1;
          end if;
          cv := st(s).cMem(ctx);
          bv := st(s).bMem(ctx);
          nv := st(s).nMem(ctx);
          if (rc >= maximum(ra, rb)) then
            px := minimum(ra, rb);
          elsif (rc <= minimum(ra, rb)) then
            px := maximum(ra, rb);
          else
            px := ra + rb - rc;
          end if;
          px := clampv(px + sg * cv);
          e  := ix - px;
          if (sg = -1) then
            e := -e;
          end if;
          e  := quantize(e);
          rx := clampv(px + sg * e * D);
          e  := mod_range(e);
          -- A.12
          bv := bv + e * D;
          if (nv = RESET) then
            if (bv >= 0) then
              bv := bv / 2;
            else
              bv := -((1 - bv) / 2);
            end if;
            nv := nv / 2;
          end if;
          nv := nv + 1;
          -- A.13
          if (bv <= -nv) then
            bv := bv + nv;
            if (cv > CO_MIN_C) then
              cv := cv - 1;
            end if;
            if (bv <= -nv) then
              bv := -nv + 1;
            end if;
          elsif (bv > 0) then
            bv := bv - nv;
            if (cv < CO_MAX_C) then
              cv := cv + 1;
            end if;
            if (bv > 0) then
              bv := 0;
            end if;
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
        ex.rx     := rx;
        ex.errval := e;
        st(s).ra  := rx;

        if (st(s).col = st(s).width - 1) then
          st(s).col := 0;
          if (rnd(30) = 0) then
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
    end loop;

    iValid <= '0';
    for i in 0 to 9 loop
      wait until rising_edge(clk);
      check_out;
    end loop;

    assert qRd = qWr report "missing outputs" severity failure;
    report "PASS: " & integer'image(qRd) & " pixels (regular " & integer'image(nReg) &
           ", forwarded " & integer'image(nFwd) & ", run " & integer'image(nRun) &
           ", RI " & integer'image(nRi) & ") NEAR=" & integer'image(NEAR) &
           " STRIPES=" & integer'image(STRIPES) & " BITNESS=" & integer'image(BITNESS);
    done <= true;
    wait;

  end process stim_proc;


end architecture sim;
