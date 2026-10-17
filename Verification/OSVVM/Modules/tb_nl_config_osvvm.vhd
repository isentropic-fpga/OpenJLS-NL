-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- tb_nl_config_osvvm: nl_config, every NEAR code 0..255 applied through iRst and checked after oDone
--
-- A.2.1 and C.2.4.1.1 are outside Docs/Requirements.md (per-sample procedure only) but are T.87
-- text, so they get T87 IDs.
--
-- Requirements:
--   T87.A2.1       RANGE, qbpp, A init
--   T87.C2.4.1.1   default T1 / T2 / T3
--   NL.NearClamp   NEAR above min(255, MAXVAL/2) clamped
--   NL.Config      RECIP, RANGE*(2*NEAR+1), oDone timing, outputs hold
--   NL.Reset       oDone low in reset, reset in every phase, recovery
--
-- Guidelines:
--   G1.Req      T87.A2.1, T87.C2.4.1.1
--   G1.Property A.2.1 / C.2.4.1.1 text in integers (no C code in T.87)
--   G1.Nl       NL.NearClamp, NL.Config, NL.Reset
--   G2.Range    nl_config.vhd range comments and asserts
--   G2.MinMax   every NEAR code: G4.Config NEAR, G4.Config qbpp, G2.MinMax node (T1 / T3 CLAMP)
--   G2.Clamp    NL.NearClamp (in range / clamped); no accumulator
--   G2.Checked  checked_* on all outputs
--   G3.Stim     G2.MinMax node (NEAR code MSB set); no signed nodes
--   G4.Config   G4.Config NEAR, G4.Config qbpp; NEAR change: every run_near re-resets with a new
--               NEAR; BITNESS 8 / 12 / 16 by the suite generics
--   G4 other    N/A: per-image constants only
--   G5          G5.States resetPhase; only oDone is reset, data is valid with oDone; next NEAR
--               replayed against golden
--   G6.Code     fully covered

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use work.openjls_pkg.all;
  use work.olo_base_pkg_math.log2ceil;

library osvvm;
  context osvvm.OsvvmContext;

library tb_support;
  use tb_support.tb_support_pkg.all;

entity tb_nl_config_osvvm is
  generic (BITNESS : natural range 8 to 16 := 12);
end entity tb_nl_config_osvvm;

architecture sim of tb_nl_config_osvvm is

  constant CLK_PERIOD : time    := CLK_PERIOD_DEFAULT;
  constant MAXVAL     : natural := 2 ** BITNESS - 1;
  constant NMAX       : natural := minimum(255, MAXVAL / 2);
  constant FACTOR     : natural := (minimum(MAXVAL, 4095) + 128) / 256;
  -- Two serial divisions of BITNESS+11 quotient bits (+1 load each) + post.
  constant DONE_MAX   : natural := 2 * (BITNESS + 12) + 4;
  -- Edges after release at which the FSM enters each phase (DW = BITNESS+11):
  -- RECIP load + DW bits, RANGE load + DW bits, POST, DONE.
  constant DW         : natural := BITNESS + 11;
  constant PH_RANGE   : natural := DW + 1;
  constant PH_POST    : natural := 2 * DW + 2;

  signal clk     : std_logic := '0';
  signal rst     : std_logic := '1';
  signal sNearIn : unsigned(7 downto 0) := (others => '0');
  signal oDone   : std_logic;
  signal oNear   : unsigned(7 downto 0);
  signal oRecip  : unsigned(BITNESS + 10 downto 0);
  signal oRange  : unsigned(BITNESS downto 0);
  signal oRangeD : unsigned(BITNESS + 1 downto 0);
  signal oQbpp   : unsigned(4 downto 0);
  signal oAinit  : unsigned(BITNESS - 1 downto 0);
  signal oT1     : unsigned(BITNESS - 1 downto 0);
  signal oT2     : unsigned(BITNESS - 1 downto 0);
  signal oT3     : unsigned(BITNESS - 1 downto 0);

  function clamp_t (
    t, lo : natural
  ) return natural is
  begin

    if (t > MAXVAL or t < lo) then
      return lo;
    end if;
    return t;

  end function clamp_t;

begin

  clk <= not clk after CLK_PERIOD / 2;

  dut : entity work.nl_config(behavioral)
    generic map (
      BITNESS => BITNESS
    )
    port map (
      iClk    => clk,
      iRst    => rst,
      iNear   => sNearIn,
      oDone   => oDone,
      oNear   => oNear,
      oRecip  => oRecip,
      oRange  => oRange,
      oRangeD => oRangeD,
      oQbpp   => oQbpp,
      oAinit  => oAinit,
      oT1     => oT1,
      oT2     => oT2,
      oT3     => oT3
    );

  stim : process is

    variable rv       : RandomPType;
    variable reqInit  : AlertLogIDType;
    variable reqThr   : AlertLogIDType;
    variable reqClamp : AlertLogIDType;
    variable reqCfg   : AlertLogIDType;
    variable reqRst   : AlertLogIDType;
    variable covRst   : CoverageIDType;
    variable covRange : CoverageIDType;
    variable covNear  : CoverageIDType;
    variable covClamp : CoverageIDType;
    variable covQbpp  : CoverageIDType;
    variable k        : natural;

    -- Reset with NEAR code nc held, release, wait for oDone, check everything.
    procedure run_near (nc : natural; msg : string) is

      variable n      : natural;
      variable rng    : natural;
      variable cyc    : natural;
      variable t1     : natural;
      variable t2     : natural;
      variable t3     : natural;
      variable recip  : natural;

    begin

      rst     <= '1';
      sNearIn <= to_unsigned(nc, 8);
      wait until rising_edge(clk);
      wait until rising_edge(clk);
      wait for 1 ns;
      AffirmIf(reqRst, oDone = '0', msg & ": oDone low in reset");
      rst <= '0';

      cyc := 0;
      loop
        wait until rising_edge(clk);
        cyc := cyc + 1;
        wait for 1 ns;
        exit when oDone = '1' or cyc > 4 * DONE_MAX;
      end loop;
      AffirmIf(reqCfg, oDone = '1' and cyc <= DONE_MAX,
               msg & ": oDone after " & to_string(cyc) & " cycles (max " & to_string(DONE_MAX) & ")");

      n     := minimum(nc, NMAX);
      rng   := (MAXVAL + 2 * n) / (2 * n + 1) + 1;
      recip := (2 ** (BITNESS + 10) + 2 * n) / (2 * n + 1);
      t1    := clamp_t(FACTOR + 2 + 3 * n, n + 1);
      t2    := clamp_t(4 * FACTOR + 3 + 5 * n, t1);
      t3    := clamp_t(17 * FACTOR + 4 + 7 * n, t2);

      if (nc > NMAX) then
        AffirmIfEqual(reqClamp, checked_integer(oNear), n, msg & ": NEAR clamped to " & to_string(NMAX));
      else
        AffirmIfEqual(reqCfg, checked_integer(oNear), n, msg & ": NEAR");
      end if;
      AffirmIfEqual(reqCfg, checked_integer(oRecip), recip, msg & ": RECIP");
      AffirmIfEqual(reqCfg, checked_integer(oRangeD), rng * (2 * n + 1), msg & ": RANGE*(2*NEAR+1)");
      AffirmIfEqual(reqInit, checked_integer(oRange), rng, msg & ": RANGE");
      AffirmIfEqual(reqInit, checked_integer(oQbpp), log2ceil(rng), msg & ": qbpp");
      AffirmIfEqual(reqInit, checked_integer(oAinit), maximum(2, (rng + 32) / 64), msg & ": A init");
      AffirmIfEqual(reqThr, checked_integer(oT1), t1, msg & ": T1");
      AffirmIfEqual(reqThr, checked_integer(oT2), t2, msg & ": T2");
      AffirmIfEqual(reqThr, checked_integer(oT3), t3, msg & ": T3");

      -- Outputs hold once done, whatever iNear does.
      sNearIn <= to_unsigned(rv.RandInt(0, 255), 8);
      for i in 1 to 5 loop
        wait until rising_edge(clk);
      end loop;
      wait for 1 ns;
      AffirmIf(reqCfg, oDone = '1' and checked_integer(oNear) = n and checked_integer(oRange) = rng and
               checked_integer(oT3) = t3, msg & ": outputs hold after done");

      ICover(covNear, n);
      ICover(covClamp, boolean'pos(nc > NMAX));
      ICover(covQbpp, log2ceil(rng));
      if (nc >= 128) then
        ICover(covRange, 0);
      end if;
      if (FACTOR + 2 + 3 * n > MAXVAL) then
        ICover(covRange, 1);
      end if;
      if (17 * FACTOR + 4 + 7 * n > MAXVAL) then
        ICover(covRange, 2);
      else
        ICover(covRange, 3);
      end if;

    end procedure run_near;

  begin

    SetAlertLogName("tb_nl_config_osvvm");
    SetLogEnable(PASSED, FALSE);
    rv.InitSeed(rv'instance_name);
    reqInit  := GetReqID("T87.A2.1", 3 * 256);
    reqThr   := GetReqID("T87.C2.4.1.1", 3 * 256);
    if (NMAX < 255) then
      reqClamp := GetReqID("NL.NearClamp", 255 - NMAX);
    end if;
    reqCfg   := GetReqID("NL.Config", 4 * 256);
    reqRst   := GetReqID("NL.Reset", 256 + 4);

    covRst := NewID("G5.States resetPhase");
    SetFieldName(covRst, "phase");
    AddBins(covRst, "RECIP divide", GenBin(0));
    AddBins(covRst, "RANGE divide", GenBin(1));
    AddBins(covRst, "POST", GenBin(2));
    AddBins(covRst, "DONE", GenBin(3));
    covRange := NewID("G2.MinMax node");
    SetFieldName(covRange, "node");
    AddBins(covRange, "NEAR code MSB set", GenBin(0));
    -- CLAMP above MAXVAL only exists where some legal NEAR overflows it.
    if (FACTOR + 2 + 3 * NMAX > MAXVAL) then
      AddBins(covRange, "T1 CLAMP > MAXVAL", GenBin(1));
    end if;
    if (17 * FACTOR + 4 + 7 * NMAX > MAXVAL) then
      AddBins(covRange, "T3 CLAMP > MAXVAL", GenBin(2));
    end if;
    AddBins(covRange, "T3 no clamp", GenBin(3));

    covNear := NewID("G4.Config NEAR");
    SetFieldName(covNear, "NEAR");
    AddBins(covNear, "lossless", GenBin(0));
    AddBins(covNear, "NEAR=1..NMAX-1", GenBin(1, NMAX - 1, 1));
    AddBins(covNear, "NMAX", GenBin(NMAX));
    covClamp := NewID("G1.Nl NEAR clamp");
    SetFieldName(covClamp, "clamped");
    AddBins(covClamp, "in range", GenBin(0));
    if (NMAX < 255) then
      AddBins(covClamp, "clamped", GenBin(1));
    end if;
    covQbpp := NewID("G4.Config qbpp");
    SetFieldName(covQbpp, "qbpp");
    for qb in log2ceil((MAXVAL + 2 * NMAX) / (2 * NMAX + 1) + 1) to BITNESS loop
      AddBins(covQbpp, "qbpp=" & to_string(qb), GenBin(qb));
    end loop;

    -- Every NEAR code, ascending, then a random order pass of a few.
    for nc in 0 to 255 loop
      run_near(nc, "NEAR code " & to_string(nc));
    end loop;

    -- run_near leaves the FSM in DONE, so every call above resets in DONE.
    ICover(covRst, 3);

    -- Re-reset in each phase, at its first and last edge, then random: the
    -- new NEAR must win. k = release edges before reset is reasserted.
    for i in 1 to 40 loop
      case i is
        when 1 => k := 0;
        when 2 => k := PH_RANGE - 1;
        when 3 => k := PH_RANGE;
        when 4 => k := PH_POST - 1;
        when 5 => k := PH_POST;
        when others => k := rv.RandInt(0, PH_POST);
      end case;
      rst     <= '1';
      sNearIn <= to_unsigned(rv.RandInt(0, 255), 8);
      wait until rising_edge(clk);
      rst     <= '0';
      for c in 1 to k loop
        wait until rising_edge(clk);
      end loop;
      wait for 1 ns;
      AffirmIf(reqRst, oDone = '0', "re-reset " & to_string(i) & ": oDone low before done");
      if (k < PH_RANGE) then
        ICover(covRst, 0);
      elsif (k < PH_POST) then
        ICover(covRst, 1);
      else
        ICover(covRst, 2);
      end if;
      run_near(rv.RandInt(0, 255), "re-reset " & to_string(i));
    end loop;

    WriteBin(covNear);
    WriteBin(covClamp);
    WriteBin(covQbpp);
    WriteBin(covRst);
    WriteBin(covRange);
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covRst), "reset phase coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covRange), "range coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covNear), "NEAR coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covClamp), "NEAR clamp coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covQbpp), "qbpp coverage closed");

    end_of_test("tb_nl_config_osvvm");
    wait;

  end process stim;

end architecture sim;
