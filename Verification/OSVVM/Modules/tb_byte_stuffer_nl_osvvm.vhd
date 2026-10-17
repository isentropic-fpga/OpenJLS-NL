-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--

--------------------------------------------------------------------------------
-- OSVVM testbench: byte_stuffer_nl (OpenJLS byte_stuffer shared by 4 lanes).
--
-- Port of OpenJLS tb_byte_stuffer_osvvm to the lane ring: one clock = one
-- lane's turn (iSlot counts 0,1,2,3 from reset, as nl_top's sSlot), every
-- port belongs to that lane except the per-lane oAlmostFull. Each lane runs
-- its own image stream with its own bit-stream FF-stuffer reference (a '0'
-- stuffed after every 0xFF, residue padded after stuffing) and its own
-- scoreboard, so a byte landing on the wrong lane or lane state leaking into
-- another lane fails the check.
--
-- Input side mirrors nl_top: a lane only presents a word on its turn when its
-- registered oAlmostFull is low; iStall is additionally pulsed at random.
-- iReady is random per lane with not-ready bursts. Each lane plays the
-- OpenJLS directed corners (rotated per lane), then random images; then
-- N_RST reset rounds, each reset hitting one phase (all idle / mid-image /
-- lane awaiting oFlushDone / FIFO backed up past oAlmostFull), followed by a
-- scored recovery image per lane.
--
-- Requirements: NL.LaneStuff (per-lane T.87 FF stuffing + flush, owner of
-- the lane ring), NL.Reset (reset in any phase clears the outputs and the
-- next image per lane matches the reference).
--
-- Guideline notes:
--   G1  Reference is a bit-level transcription of the T.87 stuffing rule
--       (byte 0xFF -> next byte carries a '0' MSB), not of the RTL layouts.
--   G2  Widths are bounded by construction (natural range on every RTL
--       variable, upstream accumulator/FIFO asserts); emitBytes 0..4 and
--       every vlen 1..IN_WIDTH (all-ones/all-zeros sweep) are the min/max
--       bins. NL contract iSlot +1/clock asserted in the RTL.
--   G3  N/A: no signed arithmetic.
--   G4  covFlush (clean / residue / dangling FF), covStuff (first / run),
--       covCross (data beat vs almostFull), all crossed with lane.
--   G5  covRst: reset per phase, aborted output ignored (scoreboards
--       drained), recovery image scored.
--   G6  Statements not hit: stage1 "FIFO full" assert branch (protected by
--       the AlmFull cushion, unreachable in a legal stream).
--------------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use work.openjls_pkg.all;

library osvvm;
  context osvvm.OsvvmContext;
  use osvvm.ScoreboardPkg_slv.all;

library work;
  use work.olo_base_pkg_math.log2ceil;

library tb_support;
  use tb_support.tb_support_pkg.all;

entity tb_byte_stuffer_nl_osvvm is
  generic (BITNESS : natural range 8 to 16 := 12);
end entity tb_byte_stuffer_nl_osvvm;

architecture sim of tb_byte_stuffer_nl_osvvm is

  -- DUT config as nl_top.
  constant IN_WIDTH    : natural := 2 * (BITNESS + maximum(8, BITNESS));
  constant OUT_BYTES   : natural := 4;
  constant OUT_WIDTH   : natural := OUT_BYTES * 8;
  constant BURST_DEPTH : natural := 16;
  constant VLEN_W      : natural := log2ceil(IN_WIDTH + 1);
  constant OBYTES_W    : natural := log2ceil(OUT_BYTES + 1);
  constant LANES       : natural := 4;

  constant CLK_PERIOD  : time    := CLK_PERIOD_DEFAULT;
  constant N_RANDOM    : natural := 40;                              -- random images per lane
  constant MAX_BEATS   : natural := 40;
  -- Directed images: 5 corners + every length of all-ones / all-zeros.
  constant N_DIR       : natural := 5 + 2 * IN_WIDTH;
  constant N_IMAGES    : natural := N_DIR + N_RANDOM;

  constant FT_CLEAN    : integer := 1;
  constant FT_RESIDUE  : integer := 2;
  constant FT_DANGLING : integer := 3;
  constant SE_NONE     : integer := 0;
  constant SE_FIRST    : integer := 1;
  constant SE_RUN      : integer := 2;

  -- Reset rounds: phase hit when rst rises.
  constant N_RST       : natural := 24;
  constant RP_IDLE     : integer := 0;                               -- nothing in flight
  constant RP_DATA     : integer := 1;                               -- mid-image, bits in flight
  constant RP_FLUSH    : integer := 2;                               -- a lane awaiting oFlushDone
  constant RP_BACKUP   : integer := 3;                               -- iReady held low, oAlmostFull set

  subtype t_word is std_logic_vector(IN_WIDTH - 1 downto 0);

  signal clk         : std_logic := '0';
  signal rst         : std_logic := '1';
  signal iSlot       : unsigned(1 downto 0) := (others => '0');
  signal iStall      : std_logic := '0';
  signal iWord       : t_word := (others => '0');
  signal iWordValid  : std_logic := '0';
  signal iWordVLen   : unsigned(VLEN_W - 1 downto 0) := (others => '0');
  signal iFlush      : std_logic := '0';
  signal oWord       : std_logic_vector(OUT_WIDTH - 1 downto 0);
  signal oWordValid  : std_logic;
  signal oValidBytes : unsigned(OBYTES_W - 1 downto 0);
  signal iReady      : std_logic := '1';
  signal oAlmostFull : std_logic_vector(3 downto 0);
  signal oFlushDone  : std_logic;

begin

  clk <= not clk after CLK_PERIOD / 2;

  dut : entity work.byte_stuffer_nl(behavioral)
    generic map (
      IN_WIDTH            => IN_WIDTH,
      OUT_BYTES_PER_CYCLE => OUT_BYTES,
      OUT_WIDTH           => OUT_WIDTH,
      BURST_DEPTH         => BURST_DEPTH
    )
    port map (
      iClk          => clk,
      iRst          => rst,
      iSlot         => iSlot,
      iStall        => iStall,
      iWord         => iWord,
      iWordValid    => iWordValid,
      iWordValidLen => iWordVLen,
      iFlush        => iFlush,
      oWord         => oWord,
      oWordValid    => oWordValid,
      oValidBytes   => oValidBytes,
      iReady        => iReady,
      oAlmostFull   => oAlmostFull,
      oFlushDone    => oFlushDone
    );

  stim : process is

    type t_int is array (0 to LANES - 1) of integer;
    type t_bool is array (0 to LANES - 1) of boolean;
    type t_byte is array (0 to LANES - 1) of std_logic_vector(7 downto 0);
    type t_words is array (0 to LANES - 1) of t_word;

    constant SB      : ScoreboardIDArrayType := NewID("lane SB", LANES);
    constant ZERO_W  : t_word := (others => '0');
    constant ONES_W  : t_word := (others => '1');

    variable rv       : RandomPType;
    variable req      : AlertLogIDType;
    variable reqRst   : AlertLogIDType;
    variable covRst   : CoverageIDType;
    variable covEmit  : CoverageIDType;
    variable covFlush : CoverageIDType;
    variable covCross : CoverageIDType;
    variable covStuff : CoverageIDType;

    -- Reference FF-stuffer state, per lane.
    variable curByte    : t_byte;
    variable bitsInByte : t_int;
    variable anyBits    : t_bool;
    variable lastEmitFF : t_bool;
    variable realSince  : t_bool;
    variable stuffSeen  : t_bool;

    -- Image driver state, per lane.
    variable imgIdx    : t_int := (others => 0);
    variable imgEnd    : t_int := (others => N_IMAGES);              -- images to play, per lane
    variable beat      : t_int := (others => 0);                     -- beat within the image
    variable nBeats    : t_int := (others => 0);
    variable waitDone  : t_bool := (others => false);                -- flushed, awaiting oFlushDone
    variable imgsDone  : t_int := (others => 0);
    variable curW      : t_words;
    variable curLen    : t_int;
    variable curLast   : t_bool;
    variable haveBeat  : t_bool := (others => false);
    variable notRdy    : t_int := (others => 0);                     -- not-ready burst left

    variable almReg    : std_logic_vector(3 downto 0) := (others => '0');
    variable s         : natural := 0;
    variable presented : boolean;
    variable nb        : natural;
    variable byte      : std_logic_vector(7 downto 0);
    variable holdOff   : boolean := false;                           -- force iReady low (backup phase)
    variable longImg   : boolean := false;                           -- full-width, MAX_BEATS images
    variable cycles    : natural := 0;
    variable w         : t_word;
    variable ph        : integer;
    variable hit       : boolean;
    variable dropped   : std_logic_vector(7 downto 0);

    procedure push_bit (l : natural; b : std_logic) is
    begin
      curByte(l)(7 - bitsInByte(l)) := b;
      bitsInByte(l) := bitsInByte(l) + 1;
      anyBits(l)    := true;
      realSince(l)  := true;
      if (bitsInByte(l) = 8) then
        Push(SB(SB'low + l), curByte(l));
        if (curByte(l) = x"FF") then
          if (stuffSeen(l)) then
            ICover(covStuff, (l, SE_RUN));
          else
            ICover(covStuff, (l, SE_FIRST));
          end if;
          stuffSeen(l)  := true;
          lastEmitFF(l) := true;
          realSince(l)  := false;
          curByte(l)    := (others => '0');
          bitsInByte(l) := 1;                                          -- stuffed '0' MSB
        else
          ICover(covStuff, (l, SE_NONE));
          lastEmitFF(l) := false;
          curByte(l)    := (others => '0');
          bitsInByte(l) := 0;
        end if;
      end if;
    end procedure push_bit;

    procedure reset_ref (l : natural) is
    begin
      curByte(l)    := (others => '0');
      bitsInByte(l) := 0;
      anyBits(l)    := false;
      lastEmitFF(l) := false;
      realSince(l)  := false;
      stuffSeen(l)  := false;
    end procedure reset_ref;

    procedure finish_image (l : natural) is
      variable ft : integer;
    begin
      if (bitsInByte(l) = 0) then
        ft := FT_CLEAN;
      elsif (bitsInByte(l) = 1 and lastEmitFF(l) and not realSince(l)) then
        ft := FT_DANGLING;
      else
        ft := FT_RESIDUE;
      end if;
      if (bitsInByte(l) > 0) then
        Push(SB(SB'low + l), curByte(l));
      end if;
      ICover(covFlush, (l, ft));
      reset_ref(l);
    end procedure finish_image;

    impure function rand_word return t_word is
      variable rw : t_word;
    begin
      if (rv.DistValInt(((1, 3), (0, 7))) = 1) then
        return ONES_W;
      end if;
      rw := (others => '0');
      for k in 0 to IN_WIDTH / 8 - 1 loop
        if (rv.DistValInt(((1, 1), (0, 7))) = 1) then
          rw(IN_WIDTH - 1 - k * 8 downto IN_WIDTH - (k + 1) * 8) := x"FF";
        else
          rw(IN_WIDTH - 1 - k * 8 downto IN_WIDTH - (k + 1) * 8) := rv.RandSlv(0, 255, 8);
        end if;
      end loop;
      return rw;
    end function rand_word;

    -- Next beat of lane l's current image (directed list rotated per lane,
    -- then random images).
    procedure next_beat (l : natural) is
      variable d : natural;
    begin
      if (beat(l) = 0) then
        if (imgIdx(l) < N_DIR) then
          d := (imgIdx(l) + 7 * l) mod N_DIR;
          if (d = 4) then
            nBeats(l) := 3;
          else
            nBeats(l) := 1;
          end if;
        elsif (longImg) then
          nBeats(l) := MAX_BEATS;
        else
          nBeats(l) := rv.RandInt(1, MAX_BEATS);
        end if;
      end if;
      d := (imgIdx(l) + 7 * l) mod N_DIR;
      w := ZERO_W;
      if (imgIdx(l) >= N_DIR) then
        w          := rand_word;
        if (longImg) then
          curLen(l) := IN_WIDTH;
        else
          curLen(l) := rv.RandInt(1, IN_WIDTH);
        end if;
      elsif (d = 0) then
        w(IN_WIDTH - 1 downto IN_WIDTH - 8) := x"5A";
        curLen(l) := 8;                                                -- clean byte end
      elsif (d = 1) then
        w(IN_WIDTH - 1 downto IN_WIDTH - 8) := x"FF";
        curLen(l) := 8;                                                -- dangling FF
      elsif (d = 2) then
        w(IN_WIDTH - 1 downto IN_WIDTH - 5) := "10101";
        curLen(l) := 5;                                                -- residue
      elsif (d = 3) then
        w         := ONES_W;
        curLen(l) := IN_WIDTH;                                         -- long FF run
      elsif (d = 4) then
        if (beat(l) < 2) then                                          -- multi-word flush
          w         := ONES_W;
          curLen(l) := IN_WIDTH;
        else
          w(IN_WIDTH - 1 downto IN_WIDTH - 12) := x"FF" & "0101";
          curLen(l) := 12;
        end if;
      else
        if ((d - 5) mod 2 = 0) then
          w := ONES_W;
        end if;
        curLen(l) := (d - 5) / 2 + 1;
      end if;
      curW(l)     := w;
      curLast(l)  := beat(l) = nBeats(l) - 1;
      haveBeat(l) := true;
    end procedure next_beat;

    -- Drive lane l's turn.
    procedure drive_slot (l : natural) is
    begin
      iSlot <= to_unsigned(l, 2);
      if (not haveBeat(l) and not waitDone(l) and imgIdx(l) < imgEnd(l)) then
        next_beat(l);
      end if;
      presented := haveBeat(l) and almReg(l) = '0';
      iWordValid <= bool2bit(presented);
      iFlush     <= bool2bit(presented and curLast(l));
      iWord      <= curW(l);
      iWordVLen  <= to_unsigned(curLen(l), VLEN_W);
      iStall     <= bool2bit(rv.DistValInt(((1, 1), (0, 9))) = 1);
      -- Per-lane downstream readiness with not-ready bursts.
      if (holdOff) then
        iReady <= '0';
      elsif (notRdy(l) > 0) then
        notRdy(l) := notRdy(l) - 1;
        iReady    <= '0';
      elsif (rv.DistValInt(((1, 1), (0, 11))) = 1) then
        notRdy(l) := rv.RandInt(2, 2 * BURST_DEPTH);
        iReady    <= '0';
      else
        iReady <= bool2bit(rv.DistValInt(((1, 3), (0, 1))) = 1);
      end if;
    end procedure drive_slot;

    -- One clock: drive lane s, sample at the edge, account.
    procedure step is
      variable consumed : boolean;
    begin
      drive_slot(s);
      wait until rising_edge(clk);
      cycles := cycles + 1;

      -- Output beat belongs to lane s.
      if (oWordValid = '1') then
        nb := checked_integer(oValidBytes);
        ICover(covEmit, (s, nb));
        if (nb > 0) then
          ICover(covCross, (s, std_to_int(oAlmostFull(s))));
        end if;
        for i in 0 to nb - 1 loop
          byte := oWord(OUT_WIDTH - 1 - i * 8 downto OUT_WIDTH - (i + 1) * 8);
          Check(SB(SB'low + s), byte);
        end loop;
      end if;
      if (oFlushDone = '1') then
        AffirmIf(req, waitDone(s), "lane " & to_string(s) & ": oFlushDone only after the lane's flush");
        AffirmIf(req, IsEmpty(SB(SB'low + s)), "lane " & to_string(s) & ": image " & to_string(imgsDone(s)) &
                 " complete at oFlushDone");
        waitDone(s) := false;
        imgsDone(s) := imgsDone(s) + 1;
      end if;

      consumed := presented and iStall = '0';
      if (consumed) then
        for i in 0 to curLen(s) - 1 loop
          push_bit(s, curW(s)(IN_WIDTH - 1 - i));
        end loop;
        haveBeat(s) := false;
        if (curLast(s)) then
          finish_image(s);
          waitDone(s)  := true;
          beat(s)      := 0;
          imgIdx(s)    := imgIdx(s) + 1;
        else
          beat(s) := beat(s) + 1;
        end if;
      end if;

      almReg := oAlmostFull;                                           -- nl_top sBsAlmReg
      s      := (s + 1) mod LANES;
    end procedure step;

    impure function all_idle return boolean is
    begin
      for l in 0 to LANES - 1 loop
        if (waitDone(l) or haveBeat(l) or imgIdx(l) < imgEnd(l)) then
          return false;
        end if;
      end loop;
      return true;
    end function all_idle;

    procedure do_reset (n : positive) is
    begin
      rst   <= '1';
      iSlot <= (others => '0');
      iWordValid <= '0';
      iFlush     <= '0';
      for i in 1 to n loop
        wait until rising_edge(clk);
      end loop;
      rst    <= '0';
      s      := 0;
      almReg := (others => '0');
    end procedure do_reset;

    impure function phase_hit (p : integer) return boolean is
    begin
      for l in 0 to LANES - 1 loop
        case p is
          when RP_DATA   => if (beat(l) > 0 and not waitDone(l)) then return true; end if;
          when RP_FLUSH  => if (waitDone(l)) then return true; end if;
          when RP_BACKUP => if (oAlmostFull(l) = '1') then return true; end if;
          when others    => null;
        end case;
      end loop;
      return p = RP_IDLE and all_idle;
    end function phase_hit;

    -- Reset now: the aborted images' expected bytes are dropped, every lane
    -- restarts with one fresh random image.
    procedure abort_and_reset (p : integer) is
    begin
      do_reset(rv.RandInt(1, 6));
      wait for 1 ns;
      AffirmIf(reqRst, oWordValid = '0' and oFlushDone = '0' and oAlmostFull = "0000",
               "reset in phase " & to_string(p) & ": outputs cleared");
      ICover(covRst, p);
      for l in 0 to LANES - 1 loop
        while not IsEmpty(SB(SB'low + l)) loop
          Pop(SB(SB'low + l), dropped);
        end loop;
        reset_ref(l);
        haveBeat(l) := false;
        waitDone(l) := false;
        beat(l)     := 0;
        notRdy(l)   := 0;
        if (imgIdx(l) < N_DIR) then
          imgIdx(l) := N_DIR;
        end if;
        imgEnd(l)   := imgIdx(l) + 1;
      end loop;
    end procedure abort_and_reset;

  begin

    SetAlertLogName("tb_byte_stuffer_nl_osvvm");
    SetLogEnable(PASSED, FALSE);
    rv.InitSeed(rv'instance_name);
    req    := GetReqID("NL.LaneStuff", 2 * LANES * (N_IMAGES + 1));
    reqRst := GetReqID("NL.Reset", 2 * N_RST);

    covRst := NewID("reset phase");
    AddBins(covRst, "reset while idle", 3, GenBin(RP_IDLE));
    AddBins(covRst, "reset mid-image", 3, GenBin(RP_DATA));
    AddBins(covRst, "reset awaiting oFlushDone", 3, GenBin(RP_FLUSH));
    AddBins(covRst, "reset with FIFO backed up (almostFull)", 3, GenBin(RP_BACKUP));

    covEmit := NewID("lane x emitBytes");
    SetFieldName(covEmit, "lane", "emitBytes");
    covFlush := NewID("lane x flushType");
    SetFieldName(covFlush, "lane", "flushType");
    covCross := NewID("lane x data x almostFull");
    SetFieldName(covCross, "lane", "almostFull");
    covStuff := NewID("lane x stuffEvent");
    SetFieldName(covStuff, "lane", "stuffEvent");
    for l in 0 to LANES - 1 loop
      for c in 0 to OUT_BYTES loop
        AddCross(covEmit, "lane=" & to_string(l) & " / emitted bytes=" & to_string(c), GenBin(l), GenBin(c));
      end loop;
      AddCross(covFlush, "lane=" & to_string(l) & " / byte aligned", GenBin(l), GenBin(FT_CLEAN));
      AddCross(covFlush, "lane=" & to_string(l) & " / padded residue", GenBin(l), GenBin(FT_RESIDUE));
      AddCross(covFlush, "lane=" & to_string(l) & " / dangling FF", GenBin(l), GenBin(FT_DANGLING));
      for af in 0 to 1 loop
        AddCross(covCross, "lane=" & to_string(l) & " / data beat / almostFull=" & to_string(af), GenBin(l), GenBin(af));
      end loop;
      AddCross(covStuff, "lane=" & to_string(l) & " / noStuff", GenBin(l), GenBin(SE_NONE));
      AddCross(covStuff, "lane=" & to_string(l) & " / firstStuff", GenBin(l), GenBin(SE_FIRST));
      AddCross(covStuff, "lane=" & to_string(l) & " / runStuff", GenBin(l), GenBin(SE_RUN));
      reset_ref(l);
    end loop;

    do_reset(6);

    -- Directed + random images, the 4 lanes interleaved.
    while not all_idle loop
      step;
      exit when cycles > 2_000_000;
    end loop;

    -- Reset rounds: run scored traffic until the target phase, reset, then a
    -- scored recovery image per lane.
    for r in 0 to N_RST - 1 loop
      if (r < 4 * 3) then
        ph := r mod 4;
      else
        ph := rv.RandInt(RP_IDLE, RP_BACKUP);
      end if;
      for l in 0 to LANES - 1 loop
        imgEnd(l) := imgIdx(l) + 2;
      end loop;
      holdOff := ph = RP_BACKUP;
      longImg := ph = RP_BACKUP;
      hit     := phase_hit(ph);
      while not hit loop
        step;
        hit := phase_hit(ph) and (ph = RP_IDLE or rv.DistValInt(((1, 1), (0, 3))) = 1);
        exit when cycles > 4_000_000;
      end loop;
      AffirmIf(reqRst, hit, "reset round " & to_string(r) & ": phase " & to_string(ph) & " reached");
      holdOff := false;
      longImg := false;
      abort_and_reset(ph);
      while not all_idle loop
        step;
        exit when cycles > 4_000_000;
      end loop;
    end loop;

    for l in 0 to LANES - 1 loop
      AffirmIf(GetAlertLogID("ScoreboardCompletion"), IsEmpty(SB(SB'low + l)),
               "lane " & to_string(l) & " scoreboard drained");
      AffirmIfEqual(GetAlertLogID("ScoreboardCompletion"), GetErrorCount(SB(SB'low + l)), 0,
                    "lane " & to_string(l) & " scoreboard mismatches");
    end loop;

    WriteBin(covEmit);
    WriteBin(covFlush);
    WriteBin(covCross);
    WriteBin(covStuff);
    WriteBin(covRst);
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covEmit), "lane x emitBytes coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covFlush), "lane x flushType coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covCross), "lane x data x almostFull coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covStuff), "lane x stuff-event coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covRst), "reset-phase coverage closed");

    end_of_test("tb_byte_stuffer_nl_osvvm");
    wait;

  end process stim;

  watchdog : process is
  begin
    wait for 50 ms;
    Alert(GetAlertLogID("Watchdog"), "tb_byte_stuffer_nl_osvvm: watchdog timeout", FAILURE);
    std.env.stop;
  end process watchdog;

end architecture sim;
