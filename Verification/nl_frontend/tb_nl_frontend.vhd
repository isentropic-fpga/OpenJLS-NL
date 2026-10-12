-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--

----------------------------------------------------------------------------------
-- Engineer:    Vitor Mendes Camilo
--
-- Module Name: tb_nl_frontend
--
-- Notes:
--              nl_config + nl_frontend + nl_loop on synthetic images, checked per
--              lane against a T.87 near-lossless model of each Ws x H stripe
--              (mode, Rx, Errval and, for regular pixels, updated C/B/N).
--              IMAGES back-to-back images, random input gaps and issue holds.
--
----------------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.math_real.all;
  use work.nl_pkg.all;

entity tb_nl_frontend is
  generic (
    BITNESS : natural  := 12;
    NEAR    : natural  := 3;
    WIDTH   : positive := 64;
    HEIGHT  : positive := 16;
    IMAGES  : positive := 2;
    SEED    : positive := 1
  );
end entity tb_nl_frontend;

architecture sim of tb_nl_frontend is

  constant RESET   : natural := 64;
  constant MAXVAL  : natural := 2 ** BITNESS - 1;
  constant D       : natural := 2 * NEAR + 1;
  constant RANGE_V : natural := (MAXVAL + 2 * NEAR) / D + 1;
  constant WS      : natural := WIDTH / 4;
  constant NPIX    : natural := WS * HEIGHT;

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

  constant FACTOR : natural := (minimum(MAXVAL, 4095) + 128) / 256;
  constant T1     : integer := clampt(FACTOR + 2 + 3 * NEAR, NEAR + 1);
  constant T2     : integer := clampt(4 * FACTOR + 3 + 5 * NEAR, T1);
  constant T3     : integer := clampt(17 * FACTOR + 4 + 7 * NEAR, T2);

  type t_exp is record
    mode   : integer;
    rx     : integer;
    errval : integer;
    c      : integer;
    b      : integer;
    n      : integer;
  end record t_exp;

  type t_exp_array is array (0 to NPIX - 1) of t_exp;
  type t_exp_lanes is array (0 to 3) of t_exp_array;
  type t_img is array (0 to HEIGHT - 1, 0 to WIDTH - 1) of integer;
  type t_ctx is array (0 to 364) of integer;
  type t_cnt is array (0 to 3) of natural;

  signal clk       : std_logic := '0';
  signal rst       : std_logic := '1';
  signal done      : boolean   := false;

  signal sCfgDone  : std_logic;
  signal sNear     : unsigned(7 downto 0);
  signal sRecip    : unsigned(BITNESS + 10 downto 0);
  signal sRange    : unsigned(BITNESS downto 0);
  signal sRangeD   : unsigned(BITNESS + 1 downto 0);
  signal sT1       : unsigned(BITNESS - 1 downto 0);
  signal sT2       : unsigned(BITNESS - 1 downto 0);
  signal sT3       : unsigned(BITNESS - 1 downto 0);

  signal iValid    : std_logic := '0';
  signal iPixel    : std_logic_vector(BITNESS - 1 downto 0) := (others => '0');
  signal oReady    : std_logic;
  signal iHold     : std_logic := '0';

  signal fValid    : std_logic;
  signal fFirstPix : std_logic;
  signal fFirstCol : std_logic;
  signal fEol      : std_logic;
  signal fIx       : unsigned(BITNESS - 1 downto 0);
  signal fRb       : unsigned(BITNESS - 1 downto 0);
  signal fRc       : unsigned(BITNESS - 1 downto 0);
  signal fQ12Zero  : std_logic;
  signal fQ3Bound  : t_signed_array(0 to 7)(BITNESS + 1 downto 0);
  signal fCandCtx  : t_unsigned_array(0 to 8)(CO_CTX_WIDTH - 1 downto 0);
  signal fCandSign : std_logic_vector(0 to 8);
  signal fCandC    : t_signed_array(0 to 8)(CO_C_WIDTH - 1 downto 0);
  signal fCandB    : t_signed_array(0 to 8)(6 downto 0);
  signal fCandN    : t_unsigned_array(0 to 8)(6 downto 0);

  signal xValid    : std_logic;
  signal xMode     : std_logic_vector(1 downto 0);
  signal xRiType   : std_logic;
  signal xRx       : unsigned(BITNESS - 1 downto 0);
  signal xErrval   : signed(BITNESS downto 0);
  signal xCtx      : unsigned(CO_CTX_WIDTH - 1 downto 0);
  signal xCq       : signed(CO_C_WIDTH - 1 downto 0);
  signal xBq       : signed(6 downto 0);
  signal xNq       : unsigned(6 downto 0);
  signal xBpre     : signed(6 downto 0);
  signal xNpre     : unsigned(6 downto 0);
  signal xNRes     : std_logic;
  signal xLane     : unsigned(1 downto 0);
  signal xFirstPix : std_logic;
  signal xLastPix  : std_logic;
  signal xEol      : std_logic;

  shared variable img : t_img;
  shared variable exp : t_exp_lanes;

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

  -- T.87 near-lossless model of stripe s of img (A.2-A.19, context updates)
  procedure model_stripe (
    s : in natural
  ) is
    type t_rec is array (-1 to HEIGHT - 1, -1 to WS) of integer;
    variable rec        : t_rec := (others => (others => 0));
    variable cMem, bMem, nMem : t_ctx;
    variable ra, rb, rc, rd, ix : integer;
    variable q1, q2, q3, ctx, sg, cv, bv, nv, px, e, rx : integer;
    variable inRun, ri  : boolean;
    variable k          : natural := 0;
    variable ex         : t_exp;
  begin

    cMem := (others => 0);
    bMem := (others => 0);
    nMem := (others => 1);
    for r in 0 to HEIGHT - 1 loop
      inRun := false;
      for j in 0 to WS - 1 loop
        ix := img(r, s * WS + j);
        if (r = 0) then
          rb := 0; rc := 0; rd := 0;
        else
          rb := rec(r - 1, j);
          if (j = 0) then
            rc := rec(r - 2, 0) when r >= 2 else 0;
          else
            rc := rec(r - 1, j - 1);
          end if;
          rd := rec(r - 1, j + 1) when j < WS - 1 else rb;
        end if;
        ra := rec(r, j - 1) when j > 0 else rb;

        q1 := quant(rd - rb);
        q2 := quant(rb - rc);
        q3 := quant(rc - ra);
        ex.c := 0; ex.b := 0; ex.n := 0;

        if (inRun or (q1 = 0 and q2 = 0 and q3 = 0)) then
          if (abs(ix - ra) <= NEAR) then
            ex.mode := 1;
            rx      := ra;
            e       := 0;
            inRun   := j /= WS - 1;
          else
            ex.mode := 2;
            ri := abs(ra - rb) <= NEAR;
            px := ra when ri else rb;
            e  := ix - px;
            if (not ri and ra > rb) then
              e  := -e;
              sg := -1;
            else
              sg := 1;
            end if;
            e     := quantize(e);
            rx    := clampv(px + sg * e * D);
            e     := mod_range(e);
            inRun := false;
          end if;
        else
          ex.mode := 0;
          if (q1 < 0 or (q1 = 0 and q2 < 0) or (q1 = 0 and q2 = 0 and q3 < 0)) then
            q1 := -q1; q2 := -q2; q3 := -q3; sg := -1;
          else
            sg := 1;
          end if;
          ctx := 81 * q1 + 9 * q2 + q3;
          cv  := cMem(ctx);
          bv  := bMem(ctx);
          nv  := nMem(ctx);
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
          bv := bv + e * D;
          if (nv = RESET) then
            bv := bv / 2 when bv >= 0 else -((1 - bv) / 2);
            nv := nv / 2;
          end if;
          nv := nv + 1;
          if (bv <= -nv) then
            bv := bv + nv;
            cv := cv - 1 when cv > CO_MIN_C else cv;
            bv := -nv + 1 when bv <= -nv else bv;
          elsif (bv > 0) then
            bv := bv - nv;
            cv := cv + 1 when cv < CO_MAX_C else cv;
            bv := 0 when bv > 0 else bv;
          end if;
          cMem(ctx) := cv;
          bMem(ctx) := bv;
          nMem(ctx) := nv;
          ex.c := cv; ex.b := bv; ex.n := nv;
        end if;
        rec(r, j) := rx;
        ex.rx     := rx;
        ex.errval := e;
        exp(s)(k) := ex;
        k         := k + 1;
      end loop;
    end loop;

  end procedure model_stripe;

begin

  clk <= not clk after 2 ns when not done;

  u_cfg : entity work.nl_config
    generic map (
      BITNESS => BITNESS
    )
    port map (
      iClk    => clk,
      iRst    => rst,
      iNear   => to_unsigned(NEAR, 8),
      oDone   => sCfgDone,
      oNear   => sNear,
      oRecip  => sRecip,
      oRange  => sRange,
      oRangeD => sRangeD,
      oQbpp   => open,
      oAinit  => open,
      oT1     => sT1,
      oT2     => sT2,
      oT3     => sT3
    );

  u_fe : entity work.nl_frontend
    generic map (
      BITNESS         => BITNESS,
      MAX_IMAGE_WIDTH => 256
    )
    port map (
      iClk        => clk,
      iRst        => rst,
      iWidth      => to_unsigned(WIDTH, 16),
      iHeight     => to_unsigned(HEIGHT, 16),
      iCfgDone    => sCfgDone,
      iNear       => sNear,
      iT1         => sT1,
      iT2         => sT2,
      iT3         => sT3,
      iValid      => iValid,
      iPixel      => iPixel,
      oReady      => oReady,
      iHold       => iHold,
      oClr        => open,
      oClrAddr    => open,
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
      iClk      => clk,
      iNear     => sNear,
      iMaxVal   => to_unsigned(MAXVAL, BITNESS),
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

  -- Stimulus: image generation, model, raster input with gaps -------------------------------------
  stim_proc : process is

    variable s1, s2 : positive := SEED;
    variable r      : real;
    variable v      : integer;

    impure function rnd (
      n : positive
    ) return natural is
    begin

      uniform(s1, s2, r);
      return natural(floor(r * real(n)));

    end function rnd;

  begin

    for im in 0 to IMAGES - 1 loop
      -- Smooth ramps, flat patches and noise
      for y in 0 to HEIGHT - 1 loop
        for x in 0 to WIDTH - 1 loop
          case (x / 8 + y / 4 + im) mod 4 is
            when 0 =>
              v := 100 + 3 * x + 2 * y;
            when 1 =>
              v := 1000;
            when 2 =>
              v := 500 + rnd(4 * NEAR + 8);
            when others =>
              v := rnd(MAXVAL + 1);
          end case;
          img(y, x) := clampv(v mod (MAXVAL + 1));
        end loop;
      end loop;
      for s in 0 to 3 loop
        model_stripe(s);
      end loop;

      if (im = 0) then
        rst <= '1';
        for i in 0 to 4 loop
          wait until rising_edge(clk);
        end loop;
        rst <= '0';
      end if;

      for y in 0 to HEIGHT - 1 loop
        for x in 0 to WIDTH - 1 loop
          while (rnd(8) = 0) loop
            iValid <= '0';
            wait until rising_edge(clk);
          end loop;
          iValid <= '1';
          iPixel <= std_logic_vector(to_unsigned(img(y, x), BITNESS));
          wait until rising_edge(clk) and oReady = '1';
        end loop;
      end loop;
      iValid <= '0';
      -- Wait for this image to leave before the model is overwritten
      wait until rising_edge(clk) and xValid = '1' and xLastPix = '1' and xLane = 3;
    end loop;

    for i in 0 to 20 loop
      wait until rising_edge(clk);
    end loop;
    done <= true;
    wait;

  end process stim_proc;

  hold_proc : process (clk) is

    variable s1, s2 : positive := SEED + 7;
    variable r      : real;

  begin

    if rising_edge(clk) then
      uniform(s1, s2, r);
      iHold <= '1' when r < 0.2 else '0';
    end if;

  end process hold_proc;

  -- Check ----------------------------------------------------------------------------------------
  check_proc : process (clk, done) is

    variable cnt   : t_cnt := (others => 0);
    variable ln    : natural;
    variable ex    : t_exp;
    variable total : natural := 0;

  begin

    if rising_edge(clk) then
      if (xValid = '1') then
        ln := to_integer(xLane);
        if (xFirstPix = '1') then
          cnt(ln) := 0;
        end if;
        ex := exp(ln)(cnt(ln));
        assert to_integer(unsigned(xMode)) = ex.mode and to_integer(xRx) = ex.rx
          report "lane " & integer'image(ln) & " pixel " & integer'image(cnt(ln)) &
                 ": mode/Rx got " & integer'image(to_integer(unsigned(xMode))) & "/" &
                 integer'image(to_integer(xRx)) & " exp " & integer'image(ex.mode) & "/" &
                 integer'image(ex.rx)
          severity failure;
        if (ex.mode /= 1) then
          assert to_integer(xErrval) = ex.errval
            report "lane " & integer'image(ln) & " pixel " & integer'image(cnt(ln)) & ": Errval"
            severity failure;
        end if;
        if (ex.mode = 0) then
          assert to_integer(xCq) = ex.c and to_integer(xBq) = ex.b and to_integer(xNq) = ex.n
            report "lane " & integer'image(ln) & " pixel " & integer'image(cnt(ln)) & ": C/B/N"
            severity failure;
        end if;
        assert (xLastPix = '1') = (cnt(ln) = NPIX - 1)
          report "lane " & integer'image(ln) & ": last pixel flag" severity failure;
        cnt(ln) := cnt(ln) + 1;
        total   := total + 1;
      end if;
    end if;
    if (done'event and done) then
      assert total = IMAGES * WIDTH * HEIGHT report "missing pixels" severity failure;
      report "PASS: " & integer'image(total) & " pixels NEAR=" & integer'image(NEAR) &
             " BITNESS=" & integer'image(BITNESS) & " W=" & integer'image(WIDTH) &
             " H=" & integer'image(HEIGHT);
    end if;

  end process check_proc;

end architecture sim;
