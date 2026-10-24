-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- tb_nl_top_osvvm: nl_top, tb_openjls_top_osvvm ported to the 4 stripe lanes
--
-- Payload correctness on real images is the golden suite's job. Here the oracle is small seeded
-- images coded per stripe by CharLS (tb_nl_top_golden_pkg). The TB parses the chunk stream
-- (Docs/output_format.md) and checks each slice stays byte-exact under stress:
--   A  every image alone, clean
--   B  4 images back to back, no reset between them
--   C  every image under random iReady, input gaps, and both
--   D  back to back under both
--   E  long iReady hold, from the start and mid image: oReady must drop, nothing is lost
--   F  minimal images (8 px per lane) flooded back to back, output ready then held: the
--      per-image queues must stall the input, not overflow
--   G  reset while idle, in the config divider, in the clear sweep, mid feed, while draining and
--      with a beat held; the recovery image uses a random NEAR / W / H and is scored
-- Every header is checked (marker, version, band, image_count, length), as are oLast at each
-- chunk end, oKeep (all ones but on oLast, contiguous from the MSB) and the AXIS hold.
--
-- Guidelines:
--   G2.Clamp    N/A: no accumulator at this level
--   G2.Checked  N/A: bytes are scoreboarded against CharLS, X / U never matches
--   G3.Stim     N/A: no arithmetic in the TB beyond byte counts
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
    CHUNK_BEATS : positive := 4;          -- header beat included, >= 2
    BAND_ID   : natural range 0 to 255 := 16#5A#;
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
  constant CHUNK      : natural := CHUNK_BEATS * NB - 8;   -- payload per chunk
  constant CLK_PERIOD : time    := CLK_PERIOD_DEFAULT;
  constant NMAX       : natural := minimum(255, (2 ** BITNESS - 1) / 2);

  constant N_KIND     : natural := 6;                                -- cases per BITNESS in the golden pkg
  constant N_RST      : natural := 24;
  constant HOLD_CYC   : natural := 3500;
  -- Past every per-image queue (upstream jls_framer start queue, 4)
  constant FLOOD      : natural := 8;
  -- Flood cases in the golden pkg: 32 x 1 (8 px per lane), 32 x 6 smooth at NEAR max (least payload)
  constant C_FLOOD    : integer_vector := (4, 3);

  -- Stress modes.
  constant M_CLEAN    : natural := 0;
  constant M_BP       : natural := 1;                                -- random iReady
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
  signal oData        : std_logic_vector(OUT_WIDTH - 1 downto 0);
  signal oValid       : std_logic;
  signal oKeep        : std_logic_vector(NB - 1 downto 0);
  signal oLast        : std_logic;
  signal iReady       : std_logic := '1';

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

  -- Some slice of this BITNESS ends exactly on a chunk boundary.
  function has_exact_chunk return boolean is
  begin
    for c in CASES'range loop
      for l in 0 to LANES - 1 loop
        if (CASES(c).bits = BITNESS and CASES(c).lane_len(l) mod CHUNK = 0) then
          return true;
        end if;
      end loop;
    end loop;
    return false;
  end function has_exact_chunk;

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
        oData        : out std_logic_vector(OUT_WIDTH - 1 downto 0);
        oValid       : out std_logic;
        oKeep        : out std_logic_vector(NB - 1 downto 0);
        oLast        : out std_logic;
        iReady       : in  std_logic
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
        OUT_WIDTH        : natural;
        CHUNK_BYTES      : natural;
        BAND_ID          : natural
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
        oData        : out std_logic_vector(OUT_WIDTH - 1 downto 0);
        oValid       : out std_logic;
        oKeep        : out std_logic_vector(NB - 1 downto 0);
        oLast        : out std_logic;
        iReady       : in  std_logic
      );
    end component nl_top;

  begin

    dut : component nl_top
      generic map (
        BITNESS          => BITNESS,
        MAX_IMAGE_WIDTH  => MAX_W,
        MAX_IMAGE_HEIGHT => MAX_H,
        OUT_WIDTH        => OUT_WIDTH,
        CHUNK_BYTES      => CHUNK + 8,
        BAND_ID          => BAND_ID
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
    type t_hdr is array (0 to 7) of natural range 0 to 255;

    constant SB : ScoreboardIDArrayType := NewID("slice SB", LANES);

    variable rv        : RandomPType;
    variable reqGold   : AlertLogIDType;
    variable reqB2B    : AlertLogIDType;
    variable reqAxis   : AlertLogIDType;
    variable reqStall  : AlertLogIDType;
    variable reqNoStl  : AlertLogIDType;
    variable reqCfg    : AlertLogIDType;
    variable reqRst    : AlertLogIDType;
    variable reqFlood  : AlertLogIDType;
    variable reqChunk  : AlertLogIDType;
    variable covStress : CoverageIDType;
    variable covB2B    : CoverageIDType;
    variable covStall  : CoverageIDType;
    variable covRst    : CoverageIDType;
    variable covCfg    : CoverageIDType;
    variable covRange  : CoverageIDType;
    variable covChunk  : CoverageIDType;

    -- Session: images queued since the last reset (all share its config).
    variable cfgCase   : natural := BASE;
    variable fq        : integer_vector(0 to 15);
    variable nFq       : natural := 0;                                -- images queued
    variable fedImg    : natural := 0;                                -- images fully fed
    variable pixIdx    : natural := 0;                                -- next pixel of image fedImg
    variable vld       : boolean := false;                            -- pixel offered, not taken
    variable gapLeft   : natural := 0;
    variable mode      : natural := M_CLEAN;
    variable holdOut   : boolean := false;
    variable newCfg    : boolean := false;                            -- config changed at the last reset

    -- Per slice output state.
    variable imgDone   : t_int := (others => 0);
    variable laneBytes : t_int := (others => 0);
    variable sbErr     : t_int := (others => 0);

    -- Stream state: header bytes seen, payload left in the current chunk.
    variable hdr       : t_hdr;
    variable hIdx      : natural := 0;
    variable payLeft   : natural := 0;
    variable chSlice   : natural := 0;
    variable chLast    : boolean := false;
    variable notRdy    : natural := 0;
    variable held      : boolean := false;
    variable heldData  : std_logic_vector(OUT_WIDTH - 1 downto 0);
    variable heldKeep  : std_logic_vector(NB - 1 downto 0);
    variable heldLast  : std_logic;

    -- oReady low while a pixel is offered, outside config / clear sweep.
    variable stallCyc  : natural := 0;
    variable cycles    : natural := 0;
    variable dropped   : std_logic_vector(8 downto 0);
    variable stallSeen : boolean;
    variable hit       : boolean;
    variable ph        : natural;
    variable c         : natural;
    variable floodErr  : t_int;

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
      AffirmIf(reqRst, oValid = '0', "reset: output stream cleared");
      for l in 0 to LANES - 1 loop
        while not IsEmpty(SB(SB'low + l)) loop
          Pop(SB(SB'low + l), dropped);
        end loop;
        imgDone(l)   := 0;
        laneBytes(l) := 0;
        sbErr(l)     := GetErrorCount(SB(SB'low + l));
      end loop;
      hIdx    := 0;
      payLeft := 0;
      held    := false;
      notRdy  := 0;
      cfgCase  := cs;
      nFq      := 0;
      fedImg   := 0;
      pixIdx   := 0;
      vld      := false;
      gapLeft  := 0;
      holdOut  := false;
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

    -- iReady this cycle.
    procedure out_ready (r : out std_logic) is
    begin
      if (holdOut) then
        r := '0';
      elsif (mode /= M_BP and mode /= M_BOTH) then
        r := '1';
      elsif (notRdy > 0) then
        notRdy := notRdy - 1;
        r      := '0';
      elsif (rv.DistValInt(((1, 1), (0, 15))) = 1) then
        notRdy := rv.RandInt(1, 12);
        r      := '0';
      else
        r := bool2bit(rv.DistValInt(((1, 3), (0, 1))) = 1);
      end if;
    end procedure out_ready;

    -- Header complete in hdr: check it and open its chunk.
    procedure open_chunk is
      variable len : natural;
      variable img : natural;
    begin
      chSlice := hdr(2) / 4 mod 4;
      chLast  := hdr(2) / 2 mod 2 = 1;
      img     := hdr(4) * 256 + hdr(5);
      len     := hdr(6) * 256 + hdr(7);
      AffirmIf(reqChunk, hdr(0) = 16#FF# and hdr(1) = 16#B0# and hdr(2) / 16 = 1 and hdr(2) mod 2 = 0,
               "header: FF B0, version 1, reserved 0; got " & to_hstring(to_unsigned(hdr(0), 8)) &
               to_hstring(to_unsigned(hdr(1), 8)) & to_hstring(to_unsigned(hdr(2), 8)));
      AffirmIfEqual(reqChunk, hdr(3), BAND_ID, "header: band");
      AffirmIfEqual(reqChunk, img, imgDone(chSlice) mod 65536, "header slice " & to_string(chSlice) & ": image_count");
      AffirmIf(reqChunk, len >= 1 and len <= CHUNK and (chLast or len = CHUNK),
               "header slice " & to_string(chSlice) & ": length " & to_string(len) & ", last " & boolean'image(chLast));
      payLeft := len;
      if (not chLast) then
        ICover(covChunk, 0);
      elsif (len < CHUNK) then
        ICover(covChunk, 1);
      else
        ICover(covChunk, 2);
      end if;
    end procedure open_chunk;

    -- Last chunk of slice l's image done: score the image.
    procedure slice_done (l : natural) is
      variable cs : natural;
    begin
      AffirmIf(reqAxis, imgDone(l) < nFq, "slice " & to_string(l) & ": last chunk only for a queued image");
      if (imgDone(l) < nFq) then
        cs := fq(imgDone(l));
        AffirmIf(reqGold, laneBytes(l) = CASES(cs).lane_len(l) and GetErrorCount(SB(SB'low + l)) = sbErr(l),
                 "slice " & to_string(l) & ": image " & to_string(imgDone(l)) & " (case " & to_string(cs) &
                 ") byte-exact, " & to_string(laneBytes(l)) & " of " & to_string(CASES(cs).lane_len(l)) & " bytes");
        if (imgDone(l) > 0) then
          AffirmIf(reqB2B, GetErrorCount(SB(SB'low + l)) = sbErr(l),
                   "slice " & to_string(l) & ": back-to-back image " & to_string(imgDone(l)) & " byte-exact");
        elsif (newCfg) then
          AffirmIf(reqCfg, GetErrorCount(SB(SB'low + l)) = sbErr(l),
                   "slice " & to_string(l) & ": first image after a config change byte-exact");
        end if;
      end if;
      sbErr(l)     := GetErrorCount(SB(SB'low + l));
      imgDone(l)   := imgDone(l) + 1;
      laneBytes(l) := 0;
    end procedure slice_done;

    -- Check the output at this edge.
    procedure check_out is
      variable n     : natural;
      variable ok    : boolean;
      variable b     : natural;
      variable ended : integer;                                      -- byte index a chunk ended at
      variable early : boolean;                                      -- a chunk ended before the beat's last byte
    begin
      if (held) then
        AffirmIf(reqAxis, oValid = '1' and oData = heldData and oKeep = heldKeep and oLast = heldLast,
                 "beat held while iReady is low");
      end if;
      held := oValid = '1' and iReady = '0';
      if (held) then
        heldData := oData;
        heldKeep := oKeep;
        heldLast := oLast;
        ICover(covChunk, 3);
      end if;

      if (oValid = '1' and iReady = '1') then
        -- oKeep: n ones from the MSB, then zeros; all ones unless oLast.
        n := 0;
        while n < NB and oKeep(NB - 1 - n) = '1' loop
          n := n + 1;
        end loop;
        ok := n >= 1 and (oLast = '1' or n = NB);
        for i in n to NB - 1 loop
          ok := ok and oKeep(NB - 1 - i) = '0';
        end loop;
        AffirmIf(reqAxis, ok, "oKeep contiguous from the MSB, all ones but on oLast: " & to_hstring(oKeep) &
                              " oLast " & to_string(oLast));
        ended := -1;
        early := false;
        for i in 0 to n - 1 loop
          b := to_integer(unsigned(oData(OUT_WIDTH - 1 - 8 * i downto OUT_WIDTH - 8 * (i + 1))));
          if (payLeft = 0) then
            early   := early or ended >= 0;
            hdr(hIdx) := b;
            hIdx    := hIdx + 1;
            if (hIdx = 8) then
              hIdx := 0;
              open_chunk;
            end if;
          else
            Check(SB(SB'low + chSlice), bool2bit(chLast and payLeft = 1) & std_logic_vector(to_unsigned(b, 8)));
            laneBytes(chSlice) := laneBytes(chSlice) + 1;
            payLeft := payLeft - 1;
            if (payLeft = 0) then
              ended := i;
              if (chLast) then
                slice_done(chSlice);
              end if;
            end if;
          end if;
        end loop;
        AffirmIf(reqChunk, not early and (oLast = '1') = (ended = n - 1),
                 "oLast exactly at each chunk end (oLast " & to_string(oLast) & ", chunk end at byte " &
                 to_string(ended) & " of " & to_string(n) & ")");
      end if;
    end procedure check_out;

    -- One clock: drive just after an edge, check at the next.
    procedure step is
      variable rdy     : std_logic;
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
      out_ready(rdy);
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
      check_out;
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
      AffirmIf(GetAlertLogID("Completion"), all_done, what & ": every slice of every image out");
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
        when others   => return held;
      end case;
    end function phase_hit;

  begin

    SetAlertLogName("tb_nl_top_osvvm");
    SetLogEnable(PASSED, FALSE);
    rv.InitSeed(rv'instance_name);

    reqGold  := GetReqID("NL.StripeGolden", LANES * (N_KIND * 4 + 4 + 3 + 5 + N_RST));
    reqB2B   := GetReqID("NL.BackToBack", LANES * (3 + 2));
    reqAxis  := GetReqID("NL.AxisOut", 1000);
    reqStall := GetReqID("NL.StallPropagation", 2);
    reqNoStl := GetReqID("NL.NoStall", N_KIND + 1);
    reqCfg   := GetReqID("NL.ConfigInReset", LANES * N_KIND);
    reqRst   := GetReqID("NL.Reset", 2 * N_RST);
    reqFlood := GetReqID("NL.ImageFlood", C_FLOOD'length * (2 * LANES + 1));
    reqChunk := GetReqID("NL.Chunks", 1000);

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
    AddBins(covStall, "hold from the start", GenBin(0));
    AddBins(covStall, "hold mid image", GenBin(1));
    covChunk := NewID("G4.Stream chunks");
    AddBins(covChunk, "full chunk", GenBin(0));
    AddBins(covChunk, "last chunk, partial", GenBin(1));
    if (has_exact_chunk) then
      AddBins(covChunk, "last chunk, full payload", GenBin(2));
    end if;
    AddBins(covChunk, "beat held", GenBin(3));
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
    covRange := NewID("G2.MinMax G4.Config config ranges");
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

    -- E: long iReady hold on the noise image, from the start, then mid image.
    -- Enough copies back to back to overflow the 2 chunks per slice.
    for t in 0 to 1 loop
      do_reset(BASE + N_KIND - 1, 2);
      mode := M_CLEAN;
      for i in 0 to minimum(fq'high, 1 + 2 * CHUNK / minimum(CASES(BASE + N_KIND - 1).lane_len)) loop
        enqueue(BASE + N_KIND - 1);
      end loop;
      while t = 1 and not (fedImg = 0 and pixIdx > 2 * CASES(BASE + N_KIND - 1).w and sClr = '0') loop
        step;
      end loop;
      holdOut  := true;
      stallCyc := 0;
      -- Until oReady has dropped for a while, or the buffers had time to fill.
      for i in 1 to HOLD_CYC + 8 * CHUNK loop
        step;
        exit when stallCyc >= 16;
      end loop;
      stallSeen := stallCyc > 0;
      AffirmIf(reqStall, stallSeen, "hold " & to_string(t) & ": oReady dropped (" &
                                    to_string(stallCyc) & " stalled cycles)");
      if (stallSeen) then
        ICover(covStall, t);
      end if;
      holdOut := false;
      run_done("stall propagation " & to_string(t));
    end loop;

    -- F: flood, outputs ready then all held.
    for f in C_FLOOD'range loop
    for hld in boolean loop
      do_reset(BASE + C_FLOOD(f), 2);
      mode := M_CLEAN;
      for n in 1 to FLOOD loop
        enqueue(BASE + C_FLOOD(f));
      end loop;
      for l in 0 to LANES - 1 loop
        floodErr(l) := GetErrorCount(SB(SB'low + l));
      end loop;
      if (hld) then
        holdOut := true;
        for i in 1 to HOLD_CYC loop
          step;
        end loop;
        AffirmIf(reqFlood, fedImg < nFq, "flood case " & to_string(C_FLOOD(f)) & " held: input stalled at image " & to_string(fedImg) &
                                         " of " & to_string(nFq));
        holdOut := false;
      end if;
      run_done("flood case " & to_string(C_FLOOD(f)) & " held=" & boolean'image(hld));
      for l in 0 to LANES - 1 loop
        AffirmIf(reqFlood, imgDone(l) = FLOOD and GetErrorCount(SB(SB'low + l)) = floodErr(l),
                 "flood case " & to_string(C_FLOOD(f)) & " held=" & boolean'image(hld) & " slice " & to_string(l) & ": " & to_string(imgDone(l)) &
                 " images byte-exact");
      end loop;
    end loop;
    end loop;

    -- G: reset rounds. Traffic, reset in the target phase, then a scored
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
               "slice " & to_string(l) & " scoreboard drained");
      AffirmIfEqual(GetAlertLogID("ScoreboardCompletion"), GetErrorCount(SB(SB'low + l)), 0,
                    "slice " & to_string(l) & " scoreboard mismatches");
    end loop;

    WriteBin(covStress);
    WriteBin(covB2B);
    WriteBin(covStall);
    WriteBin(covChunk);
    WriteBin(covRst);
    WriteBin(covCfg);
    WriteBin(covRange);
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covStress), "image x stress coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covB2B), "back-to-back coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covStall), "stall propagation coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covChunk), "chunk coverage closed");
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
