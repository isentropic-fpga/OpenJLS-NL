-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- tb_nl_top_osvvm: nl_top, tb_openjls_top_osvvm ported to the 4 stripe lanes
--
-- Payload correctness on real images is the golden suite's job. Here the oracle is small seeded
-- images coded per stripe by CharLS (tb_nl_top_golden_pkg), and the TB checks the 4 streams stay
-- byte-exact under stress:
--   A  every image alone, clean
--   B  4 images back to back, no reset between them
--   C  every image under random per-lane iReady, input gaps, and both
--   D  back to back under both
--   E  long iReady hold on each lane and on all 4: oReady must drop, nothing is lost
--   F  reset while idle, in the config divider, in the clear sweep, mid feed, while draining and
--      with a beat held; the recovery image uses a random NEAR / W / H and is scored
-- Every lane also checks oKeep (contiguous from the MSB, >= 1 byte), oLast on FF D9 and the AXIS
-- hold.
--
-- Guidelines:
--   G2.Clamp    N/A: no accumulator at this level
--   G2.Checked  N/A: bytes are scoreboarded against CharLS, X / U never matches
--   G3.Stim     N/A: no arithmetic in the TB beyond byte counts
--   G4.Config   W, H, NEAR min/max in G2.MinMax config ranges
--   G4.Lane     G4.Stream lane backpressure hold; every image scored on all 4 lanes
--   G4 other    N/A: owned by the module TBs

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use work.openjls_pkg.all;
  use work.tb_nl_top_golden_pkg.all;

library osvvm;
  context osvvm.OsvvmContext;
  use osvvm.ScoreboardPkg_slv.all;

library tb_support;
  use tb_support.tb_support_pkg.all;

entity tb_nl_top_osvvm is
  generic (
    BITNESS   : natural range 8 to 16 := 12;
    OUT_WIDTH : natural := 64;
    -- Defaults equal the largest golden image (64 x 64), so W and H reach max.
    MAX_W     : natural := 64;
    MAX_H     : natural := 64;
    -- Bind to the funcsim netlist of PostSynth/build_run_osvvm.sh (no generics).
    POST_SYNTH : boolean := false
  );
end entity tb_nl_top_osvvm;

architecture sim of tb_nl_top_osvvm is

  constant LANES      : natural := 4;
  constant NB         : natural := OUT_WIDTH / 8;
  constant CLK_PERIOD : time    := CLK_PERIOD_DEFAULT;
  constant NMAX       : natural := minimum(255, (2 ** BITNESS - 1) / 2);

  constant N_KIND     : natural := 6;                                -- cases per BITNESS in the golden pkg
  constant N_RST      : natural := 24;
  constant HOLD_CYC   : natural := 3500;

  -- Stress modes.
  constant M_CLEAN    : natural := 0;
  constant M_BP       : natural := 1;                                -- random per-lane iReady
  constant M_GAP      : natural := 2;                                -- random input gaps
  constant M_BOTH     : natural := 3;

  -- Reset phases.
  constant RP_IDLE    : natural := 0;
  constant RP_CFG     : natural := 1;                                -- config divider running
  constant RP_CLEAR   : natural := 2;                                -- context clear sweep
  constant RP_FEED    : natural := 3;                                -- image partly fed
  constant RP_DRAIN   : natural := 4;                                -- all fed, oLast pending
  constant RP_HOLD    : natural := 5;                                -- output beat held

  signal clk          : std_logic := '0';
  signal rst          : std_logic := '1';
  signal iNear        : std_logic_vector(7 downto 0) := (others => '0');
  signal iImageWidth  : std_logic_vector(15 downto 0) := x"0020";      -- legal while the first reset runs
  signal iImageHeight : std_logic_vector(15 downto 0) := x"0001";
  signal iValid       : std_logic := '0';
  signal iPixel       : std_logic_vector(BITNESS - 1 downto 0) := (others => '0');
  signal oReady       : std_logic;
  signal oData        : std_logic_vector(LANES * OUT_WIDTH - 1 downto 0);
  signal oValid       : std_logic_vector(LANES - 1 downto 0);
  signal oKeep        : std_logic_vector(LANES * NB - 1 downto 0);
  signal oLast        : std_logic_vector(LANES - 1 downto 0);
  signal iReady       : std_logic_vector(LANES - 1 downto 0) := (others => '1');

  -- First golden case of this BITNESS.
  function first_case return natural is
  begin
    for c in CASES'range loop
      if (CASES(c).bits = BITNESS) then
        return c;
      end if;
    end loop;
    report "tb_nl_top_osvvm: no golden cases for BITNESS " & integer'image(BITNESS) severity failure;
    return 0;
  end function first_case;

  constant BASE : natural := first_case;

begin

  clk <= not clk after CLK_PERIOD / 2;

  -- Components, not entity instantiation: the netlist nl_top has no generics,
  -- and only the elaborated branch is bound.
  g_dut : if POST_SYNTH generate

    component nl_top is
      port (
        iClk         : in  std_logic;
        iRst         : in  std_logic;
        iNear        : in  std_logic_vector(7 downto 0);
        iImageWidth  : in  std_logic_vector(15 downto 0);
        iImageHeight : in  std_logic_vector(15 downto 0);
        iValid       : in  std_logic;
        iPixel       : in  std_logic_vector(BITNESS - 1 downto 0);
        oReady       : out std_logic;
        oData        : out std_logic_vector(LANES * OUT_WIDTH - 1 downto 0);
        oValid       : out std_logic_vector(LANES - 1 downto 0);
        oKeep        : out std_logic_vector(LANES * NB - 1 downto 0);
        oLast        : out std_logic_vector(LANES - 1 downto 0);
        iReady       : in  std_logic_vector(LANES - 1 downto 0)
      );
    end component nl_top;

  begin

    dut : component nl_top
      port map (
        iClk         => clk,
        iRst         => rst,
        iNear        => iNear,
        iImageWidth  => iImageWidth,
        iImageHeight => iImageHeight,
        iValid       => iValid,
        iPixel       => iPixel,
        oReady       => oReady,
        oData        => oData,
        oValid       => oValid,
        oKeep        => oKeep,
        oLast        => oLast,
        iReady       => iReady
      );

  else generate

    component nl_top is
      generic (
        BITNESS          : natural;
        MAX_IMAGE_WIDTH  : natural;
        MAX_IMAGE_HEIGHT : natural;
        OUT_WIDTH        : natural
      );
      port (
        iClk         : in  std_logic;
        iRst         : in  std_logic;
        iNear        : in  std_logic_vector(7 downto 0);
        iImageWidth  : in  std_logic_vector(15 downto 0);
        iImageHeight : in  std_logic_vector(15 downto 0);
        iValid       : in  std_logic;
        iPixel       : in  std_logic_vector(BITNESS - 1 downto 0);
        oReady       : out std_logic;
        oData        : out std_logic_vector(LANES * OUT_WIDTH - 1 downto 0);
        oValid       : out std_logic_vector(LANES - 1 downto 0);
        oKeep        : out std_logic_vector(LANES * NB - 1 downto 0);
        oLast        : out std_logic_vector(LANES - 1 downto 0);
        iReady       : in  std_logic_vector(LANES - 1 downto 0)
      );
    end component nl_top;

  begin

    dut : component nl_top
      generic map (
        BITNESS          => BITNESS,
        MAX_IMAGE_WIDTH  => MAX_W,
        MAX_IMAGE_HEIGHT => MAX_H,
        OUT_WIDTH        => OUT_WIDTH
      )
      port map (
        iClk         => clk,
        iRst         => rst,
        iNear        => iNear,
        iImageWidth  => iImageWidth,
        iImageHeight => iImageHeight,
        iValid       => iValid,
        iPixel       => iPixel,
        oReady       => oReady,
        oData        => oData,
        oValid       => oValid,
        oKeep        => oKeep,
        oLast        => oLast,
        iReady       => iReady
      );

  end generate g_dut;

  stim : process is

    -- Phase probes (G5): config divider done, context clear sweep.
    alias sCfgDone is << signal .tb_nl_top_osvvm.g_dut.dut.sCfgDone : std_logic >>;
    alias sClr is << signal .tb_nl_top_osvvm.g_dut.dut.sClr : std_logic >>;

    type t_int is array (0 to LANES - 1) of integer;
    type t_bool is array (0 to LANES - 1) of boolean;
    type t_data is array (0 to LANES - 1) of std_logic_vector(OUT_WIDTH - 1 downto 0);
    type t_keep is array (0 to LANES - 1) of std_logic_vector(NB - 1 downto 0);

    constant SB : ScoreboardIDArrayType := NewID("lane SB", LANES);

    variable rv        : RandomPType;
    variable reqGold   : AlertLogIDType;
    variable reqB2B    : AlertLogIDType;
    variable reqAxis   : AlertLogIDType;
    variable reqStall  : AlertLogIDType;
    variable reqNoStl  : AlertLogIDType;
    variable reqCfg    : AlertLogIDType;
    variable reqRst    : AlertLogIDType;
    variable covStress : CoverageIDType;
    variable covB2B    : CoverageIDType;
    variable covStall  : CoverageIDType;
    variable covHold   : CoverageIDType;
    variable covRst    : CoverageIDType;
    variable covCfg    : CoverageIDType;
    variable covRange  : CoverageIDType;

    -- Session: images queued since the last reset (all share its config).
    variable cfgCase   : natural := BASE;
    variable fq        : integer_vector(0 to 15);
    variable nFq       : natural := 0;                                -- images queued
    variable fedImg    : natural := 0;                                -- images fully fed
    variable pixIdx    : natural := 0;                                -- next pixel of image fedImg
    variable vld       : boolean := false;                            -- pixel offered, not taken
    variable gapLeft   : natural := 0;
    variable mode      : natural := M_CLEAN;
    variable holdMask  : std_logic_vector(LANES - 1 downto 0) := (others => '0');
    variable newCfg    : boolean := false;                            -- config changed at the last reset

    -- Per lane output state.
    variable imgDone   : t_int := (others => 0);
    variable laneBytes : t_int := (others => 0);
    variable sbErr     : t_int := (others => 0);
    variable notRdy    : t_int := (others => 0);
    variable held      : t_bool := (others => false);
    variable heldData  : t_data;
    variable heldKeep  : t_keep;
    variable heldLast  : t_bool;

    -- oReady low while a pixel is offered, outside config / clear sweep.
    variable stallCyc  : natural := 0;
    variable cycles    : natural := 0;
    variable dropped   : std_logic_vector(8 downto 0);
    variable stallSeen : boolean;
    variable hit       : boolean;
    variable ph        : natural;
    variable c         : natural;

    impure function npix (cs : natural) return natural is
    begin
      return CASES(cs).w * CASES(cs).h;
    end function npix;

    -- Queue image cs; its 4 lane streams go to the scoreboards.
    procedure enqueue (cs : natural) is
    begin
      assert CASES(cs).w = CASES(cfgCase).w and CASES(cs).h = CASES(cfgCase).h and
             CASES(cs).near = CASES(cfgCase).near
        report "tb: enqueued image does not match the reset config" severity failure;
      fq(nFq) := cs;
      nFq     := nFq + 1;
      for l in 0 to LANES - 1 loop
        for i in 0 to CASES(cs).lane_len(l) - 1 loop
          Push(SB(SB'low + l), bool2bit(i = CASES(cs).lane_len(l) - 1) &
                               std_logic_vector(to_unsigned(BYTES(CASES(cs).lane_off(l) + i), 8)));
        end loop;
      end loop;
    end procedure enqueue;

    -- Reset with the config of case cs, dropping everything in flight.
    procedure do_reset (cs : natural; n : positive) is
      constant OLD : natural := cfgCase;
    begin
      iNear        <= std_logic_vector(to_unsigned(CASES(cs).near, 8));
      iImageWidth  <= std_logic_vector(to_unsigned(CASES(cs).w, 16));
      iImageHeight <= std_logic_vector(to_unsigned(CASES(cs).h, 16));
      iValid       <= '0';
      rst          <= '1';
      for i in 1 to n loop
        wait until rising_edge(clk);
      end loop;
      rst <= '0';
      wait for 1 ns;
      -- oLast / oData are FIFO data (not reset); only oValid qualifies them.
      AffirmIf(reqRst, oValid = "0000", "reset: output streams cleared");
      for l in 0 to LANES - 1 loop
        while not IsEmpty(SB(SB'low + l)) loop
          Pop(SB(SB'low + l), dropped);
        end loop;
        imgDone(l)   := 0;
        laneBytes(l) := 0;
        held(l)      := false;
        notRdy(l)    := 0;
        sbErr(l)     := GetErrorCount(SB(SB'low + l));
      end loop;
      cfgCase  := cs;
      nFq      := 0;
      fedImg   := 0;
      pixIdx   := 0;
      vld      := false;
      gapLeft  := 0;
      holdMask := (others => '0');
      newCfg   := CASES(cs).near /= CASES(OLD).near or CASES(cs).w /= CASES(OLD).w or
                  CASES(cs).h /= CASES(OLD).h;
      if (CASES(cs).near /= CASES(OLD).near) then
        ICover(covCfg, 0);
      end if;
      if (CASES(cs).w /= CASES(OLD).w or CASES(cs).h /= CASES(OLD).h) then
        ICover(covCfg, 1);
      end if;
      if (not newCfg) then
        ICover(covCfg, 2);
      end if;
      ICover(covRange, (0, CASES(cs).w));
      ICover(covRange, (1, CASES(cs).h));
      ICover(covRange, (2, CASES(cs).near));
    end procedure do_reset;

    -- Lane l's iReady this cycle.
    procedure lane_ready (l : natural; r : out std_logic) is
    begin
      if (holdMask(l) = '1') then
        r := '0';
      elsif (mode /= M_BP and mode /= M_BOTH) then
        r := '1';
      elsif (notRdy(l) > 0) then
        notRdy(l) := notRdy(l) - 1;
        r         := '0';
      elsif (rv.DistValInt(((1, 1), (0, 15))) = 1) then
        notRdy(l) := rv.RandInt(1, 12);
        r         := '0';
      else
        r := bool2bit(rv.DistValInt(((1, 3), (0, 1))) = 1);
      end if;
    end procedure lane_ready;

    -- Check lane l's output at this edge.
    procedure check_lane (l : natural) is
      variable data : std_logic_vector(OUT_WIDTH - 1 downto 0);
      variable keep : std_logic_vector(NB - 1 downto 0);
      variable n    : natural;
      variable ok   : boolean;
      variable cs   : natural;
    begin
      data := oData((l + 1) * OUT_WIDTH - 1 downto l * OUT_WIDTH);
      keep := oKeep((l + 1) * NB - 1 downto l * NB);
      if (held(l)) then
        AffirmIf(reqAxis, oValid(l) = '1' and data = heldData(l) and keep = heldKeep(l) and
                          (oLast(l) = '1') = heldLast(l),
                 "lane " & to_string(l) & ": beat held while iReady is low");
      end if;
      held(l) := oValid(l) = '1' and iReady(l) = '0';
      if (held(l)) then
        heldData(l) := data;
        heldKeep(l) := keep;
        heldLast(l) := oLast(l) = '1';
        ICover(covHold, l);
      end if;

      if (oValid(l) = '1' and iReady(l) = '1') then
        -- oKeep: n ones from the MSB, then zeros.
        n := 0;
        while n < NB and keep(NB - 1 - n) = '1' loop
          n := n + 1;
        end loop;
        ok := n >= 1;
        for i in n to NB - 1 loop
          ok := ok and keep(NB - 1 - i) = '0';
        end loop;
        AffirmIf(reqAxis, ok, "lane " & to_string(l) & ": oKeep contiguous from the MSB, " & to_hstring(keep));
        for i in 0 to n - 1 loop
          Check(SB(SB'low + l), bool2bit(oLast(l) = '1' and i = n - 1) &
                                data(OUT_WIDTH - 1 - 8 * i downto OUT_WIDTH - 8 * (i + 1)));
          laneBytes(l) := laneBytes(l) + 1;
        end loop;
        if (oLast(l) = '1') then
          AffirmIf(reqAxis, imgDone(l) < nFq, "lane " & to_string(l) & ": oLast only for a queued image");
          if (imgDone(l) < nFq) then
            cs := fq(imgDone(l));
            AffirmIf(reqGold, laneBytes(l) = CASES(cs).lane_len(l) and GetErrorCount(SB(SB'low + l)) = sbErr(l),
                     "lane " & to_string(l) & ": image " & to_string(imgDone(l)) & " (case " & to_string(cs) &
                     ") byte-exact, " & to_string(laneBytes(l)) & " of " & to_string(CASES(cs).lane_len(l)) & " bytes");
            if (imgDone(l) > 0) then
              AffirmIf(reqB2B, GetErrorCount(SB(SB'low + l)) = sbErr(l),
                       "lane " & to_string(l) & ": back-to-back image " & to_string(imgDone(l)) & " byte-exact");
            elsif (newCfg) then
              AffirmIf(reqCfg, GetErrorCount(SB(SB'low + l)) = sbErr(l),
                       "lane " & to_string(l) & ": first image after a config change byte-exact");
            end if;
          end if;
          sbErr(l)     := GetErrorCount(SB(SB'low + l));
          imgDone(l)   := imgDone(l) + 1;
          laneBytes(l) := 0;
        end if;
      end if;
    end procedure check_lane;

    -- One clock: drive just after an edge, check at the next.
    procedure step is
      variable rdy     : std_logic_vector(LANES - 1 downto 0);
      variable w       : natural;
      variable expWait : boolean;
    begin
      if (not vld and fedImg < nFq) then
        if (gapLeft > 0) then
          gapLeft := gapLeft - 1;
        elsif ((mode = M_GAP or mode = M_BOTH) and rv.DistValInt(((1, 1), (0, 7))) = 1) then
          gapLeft := rv.RandInt(0, 20);
        else
          vld := true;
          iPixel <= std_logic_vector(to_unsigned(PIX(CASES(fq(fedImg)).pix_off + pixIdx), BITNESS));
        end if;
      end if;
      iValid <= bool2bit(vld);
      for l in 0 to LANES - 1 loop
        lane_ready(l, rdy(l));
      end loop;
      iReady <= rdy;

      wait until rising_edge(clk);
      cycles := cycles + 1;

      -- Not a stall: overwriting lane 0's stripe of the first row after a clear
      -- sweep. That row was buffered during the sweep, so lane 0 frees it at
      -- 1/4 px per clock (D3).
      expWait := false;
      if (vld) then
        w := CASES(fq(fedImg)).w;
        if (pixIdx >= w) then
          expWait := pixIdx < 2 * w;
        else
          expWait := fedImg > 0 and CASES(fq(fedImg - 1)).h = 1;
        end if;
        expWait := expWait and pixIdx mod w < w / 4;
      end if;
      if (vld and oReady = '0' and sCfgDone = '1' and sClr = '0' and not expWait) then
        stallCyc := stallCyc + 1;
      end if;
      if (vld and oReady = '1') then
        vld    := false;
        pixIdx := pixIdx + 1;
        if (pixIdx = npix(fq(fedImg))) then
          pixIdx := 0;
          fedImg := fedImg + 1;
        end if;
      end if;
      for l in 0 to LANES - 1 loop
        check_lane(l);
      end loop;
      wait for 1 ns;
    end procedure step;

    impure function all_done return boolean is
    begin
      for l in 0 to LANES - 1 loop
        if (imgDone(l) < nFq) then
          return false;
        end if;
      end loop;
      return fedImg = nFq;
    end function all_done;

    procedure run_done (what : string) is
      constant START : natural := cycles;
    begin
      while not all_done loop
        step;
        exit when cycles - START > 200_000;
      end loop;
      AffirmIf(GetAlertLogID("Completion"), all_done, what & ": every lane framed every image");
    end procedure run_done;

    -- One session: reset into case cs, queue it (and partners), run.
    procedure session (cs : natural; m : natural; n : positive := 1) is
    begin
      do_reset(cs, rv.RandInt(1, 4));
      mode := m;
      for i in 0 to n - 1 loop
        -- Back to back: the struct / smooth pair alternates (same dims, NEAR 0).
        if (n > 1) then
          enqueue(BASE + i mod 2);
        else
          enqueue(cs);
        end if;
      end loop;
      run_done("case " & to_string(cs) & " mode " & to_string(m));
    end procedure session;

    impure function phase_hit (p : natural) return boolean is
    begin
      case p is
        when RP_IDLE  => return all_done;
        when RP_CFG   => return sCfgDone = '0';
        when RP_CLEAR => return sClr = '1';
        when RP_FEED  => return fedImg < nFq and pixIdx > 0 and sClr = '0';
        when RP_DRAIN => return fedImg = nFq and not all_done;
        when others   =>
          for l in 0 to LANES - 1 loop
            if (held(l)) then
              return true;
            end if;
          end loop;
          return false;
      end case;
    end function phase_hit;

  begin

    SetAlertLogName("tb_nl_top_osvvm");
    SetLogEnable(PASSED, FALSE);
    rv.InitSeed(rv'instance_name);

    reqGold  := GetReqID("NL.StripeGolden", LANES * (N_KIND * 4 + 4 + 3 + 5 + N_RST));
    reqB2B   := GetReqID("NL.BackToBack", LANES * (3 + 2));
    reqAxis  := GetReqID("NL.AxisOut", 1000);
    reqStall := GetReqID("NL.StallPropagation", 5);
    reqNoStl := GetReqID("NL.NoStall", N_KIND + 1);
    reqCfg   := GetReqID("NL.ConfigInReset", LANES * N_KIND);
    reqRst   := GetReqID("NL.Reset", 2 * N_RST);

    covStress := NewID("G4.Stream image kind x stress mode");
    SetFieldName(covStress, "case", "mode");
    for k in 0 to N_KIND - 1 loop
      AddCross(covStress, "case " & to_string(k) & " / clean", GenBin(k), GenBin(M_CLEAN));
      AddCross(covStress, "case " & to_string(k) & " / backpressure", GenBin(k), GenBin(M_BP));
      AddCross(covStress, "case " & to_string(k) & " / input gaps", GenBin(k), GenBin(M_GAP));
      AddCross(covStress, "case " & to_string(k) & " / both", GenBin(k), GenBin(M_BOTH));
    end loop;
    covB2B := NewID("G4.Stream back to back");
    AddBins(covB2B, "clean", GenBin(0));
    AddBins(covB2B, "stressed", GenBin(1));
    covStall := NewID("G4.Stream stall propagation");
    covHold  := NewID("G4.Stream lane backpressure hold");
    for l in 0 to LANES - 1 loop
      AddBins(covStall, "hold lane " & to_string(l), GenBin(l));
      AddBins(covHold, "lane " & to_string(l), GenBin(l));
    end loop;
    AddBins(covStall, "hold all lanes", GenBin(LANES));
    covRst := NewID("G5.States resetPhase");
    AddBins(covRst, "reset while idle", 2, GenBin(RP_IDLE));
    AddBins(covRst, "reset in the config divider", 2, GenBin(RP_CFG));
    AddBins(covRst, "reset in the clear sweep", 2, GenBin(RP_CLEAR));
    AddBins(covRst, "reset mid feed", 2, GenBin(RP_FEED));
    AddBins(covRst, "reset while draining", 2, GenBin(RP_DRAIN));
    AddBins(covRst, "reset with output held", 2, GenBin(RP_HOLD));
    covCfg := NewID("G4.Config config across reset");
    AddBins(covCfg, "NEAR changed", 3, GenBin(0));
    AddBins(covCfg, "W/H changed", 3, GenBin(1));
    AddBins(covCfg, "same config", 3, GenBin(2));
    covRange := NewID("G2.MinMax config ranges");
    SetFieldName(covRange, "field", "value");
    AddCross(covRange, "W = 32", GenBin(0), GenBin(32));
    if (MAX_W = 64) then
      AddCross(covRange, "W = MAX_W", GenBin(0), GenBin(MAX_W));
    end if;
    AddCross(covRange, "H = 1", GenBin(1), GenBin(1));
    if (MAX_H = 64) then
      AddCross(covRange, "H = MAX_H", GenBin(1), GenBin(MAX_H));
    end if;
    AddCross(covRange, "NEAR = 0", GenBin(2), GenBin(0));
    AddCross(covRange, "NEAR = max", GenBin(2), GenBin(NMAX));

    -- A: every image alone, clean. oReady may only drop in the config
    -- divider, the clear sweep, and while row 0 is issued (see step).
    for k in 0 to N_KIND - 1 loop
      stallCyc := 0;
      session(BASE + k, M_CLEAN);
      ICover(covStress, (k, M_CLEAN));
      AffirmIfEqual(reqNoStl, stallCyc, 0, "case " & to_string(k) & ": oReady held high outside the clear sweep");
    end loop;

    -- B: back to back, clean.
    stallCyc := 0;
    session(BASE, M_CLEAN, 4);
    ICover(covB2B, 0);
    AffirmIfEqual(reqNoStl, stallCyc, 0, "back to back: oReady held high outside the clear sweep");

    -- C: every image stressed.
    for m in M_BP to M_BOTH loop
      for k in 0 to N_KIND - 1 loop
        session(BASE + k, m);
        ICover(covStress, (k, m));
      end loop;
    end loop;

    -- D: back to back, stressed.
    session(BASE, M_BOTH, 3);
    ICover(covB2B, 1);

    -- E: long iReady hold on the noise image: one lane at a time, then all.
    for t in 0 to LANES loop
      do_reset(BASE + N_KIND - 1, 2);
      mode := M_CLEAN;
      enqueue(BASE + N_KIND - 1);
      while not (fedImg = 0 and pixIdx > 2 * CASES(BASE + N_KIND - 1).w and sClr = '0') loop
        step;
      end loop;
      if (t = LANES) then
        holdMask := (others => '1');
      else
        holdMask    := (others => '0');
        holdMask(t) := '1';
      end if;
      stallCyc := 0;
      -- Until oReady has dropped for a while (OUT_WIDTH 1024 needs ~2.5k
      -- cycles to fill the output FIFOs), or HOLD_CYC.
      for i in 1 to HOLD_CYC loop
        step;
        exit when stallCyc >= 16;
      end loop;
      stallSeen := stallCyc > 0;
      AffirmIf(reqStall, stallSeen, "hold " & to_hstring(holdMask) & ": oReady dropped (" &
                                    to_string(stallCyc) & " stalled cycles)");
      if (stallSeen) then
        ICover(covStall, t);
      end if;
      holdMask := (others => '0');
      run_done("stall propagation " & to_string(t));
    end loop;

    -- F: reset rounds. Traffic, reset in the target phase, then a scored
    -- recovery image at a random config.
    for r in 0 to N_RST - 1 loop
      if (r < 2 * 6) then
        ph := r mod 6;
      else
        ph := rv.RandInt(RP_IDLE, RP_HOLD);
      end if;
      if (ph = RP_DRAIN or ph = RP_HOLD) then
        c := BASE + N_KIND - 1;
      else
        c := BASE + rv.RandInt(0, N_KIND - 1);
      end if;
      do_reset(c, rv.RandInt(1, 4));
      if (ph = RP_HOLD) then
        mode := M_BOTH;
      else
        mode := rv.RandInt(M_CLEAN, M_BOTH);
      end if;
      enqueue(c);
      if (c - BASE < 2 and rv.DistValInt(((1, 1), (0, 1))) = 1) then
        enqueue(BASE + 1 - (c - BASE));
      end if;
      hit := phase_hit(ph);
      while not hit loop
        step;
        hit := phase_hit(ph) and (ph = RP_IDLE or ph = RP_CFG or rv.DistValInt(((1, 1), (0, 3))) = 1);
        exit when all_done and ph /= RP_IDLE;
      end loop;
      AffirmIf(reqRst, hit, "reset round " & to_string(r) & ": phase " & to_string(ph) & " reached");
      ICover(covRst, ph);
      c := BASE + rv.RandInt(0, N_KIND - 1);
      session(c, rv.RandInt(M_CLEAN, M_BOTH));
    end loop;

    for l in 0 to LANES - 1 loop
      AffirmIf(GetAlertLogID("ScoreboardCompletion"), IsEmpty(SB(SB'low + l)),
               "lane " & to_string(l) & " scoreboard drained");
      AffirmIfEqual(GetAlertLogID("ScoreboardCompletion"), GetErrorCount(SB(SB'low + l)), 0,
                    "lane " & to_string(l) & " scoreboard mismatches");
    end loop;

    WriteBin(covStress);
    WriteBin(covB2B);
    WriteBin(covStall);
    WriteBin(covHold);
    WriteBin(covRst);
    WriteBin(covCfg);
    WriteBin(covRange);
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covStress), "image x stress coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covB2B), "back-to-back coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covStall), "stall propagation coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covHold), "lane hold coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covRst), "reset-phase coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covCfg), "config change coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covRange), "config range coverage closed");

    end_of_test("tb_nl_top_osvvm");
    wait;

  end process stim;

  watchdog : process is
  begin
    wait for 100 ms;
    Alert(GetAlertLogID("Watchdog"), "tb_nl_top_osvvm: watchdog timeout", FAILURE);
    std.env.stop;
  end process watchdog;

end architecture sim;
