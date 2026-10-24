-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- tb_nl_merge_osvvm: nl_merge, 4 slice streams into one chunk stream (Docs/output_format.md)
--
-- Each lane sends random slices (1 byte up to 3 chunks, exact chunk multiples included). The lane
-- per cycle is rotating (as nl_top), random or bursty; iReady is ready, random or held long enough
-- to fill every chunk buffer. Per lane, a byte scoreboard and a chunk scoreboard (length, last,
-- image_count) are pushed on input acceptance; each output packet must be one chunk with a valid
-- header, 8 + length bytes, oLast on its end. Then a 65536-slice run per lane wraps image_count,
-- and N_RST reset rounds with a scored recovery.
--
-- Guidelines:
--   G1.Golden   N/A: no T.87 arithmetic; the oracle is the format spec
--   G2.Clamp    N/A: no accumulator
--   G2.Checked  N/A: bytes are scoreboarded, X / U never matches
--   G3.Stim     N/A: no arithmetic on data
--   G4 other    N/A: no Edge / Run / Coding / Config / Branch logic

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

entity tb_nl_merge_osvvm is
  generic (
    OUT_WIDTH   : positive := 64;
    CHUNK_BEATS : positive := 4;     -- header beat included, >= 2
    BAND_ID     : natural  := 165
  );
end entity tb_nl_merge_osvvm;

architecture sim of tb_nl_merge_osvvm is

  constant NB         : natural := OUT_WIDTH / 8;
  constant CHUNK      : natural := CHUNK_BEATS * NB - 8;          -- payload per chunk
  constant BW         : natural := log2ceil(NB + 1);
  constant LANES      : natural := 4;
  constant CLK_PERIOD : time    := CLK_PERIOD_DEFAULT;

  constant N_SLICES   : natural := 40;                                 -- per lane, random phase
  constant N_RST      : natural := 20;
  -- Other-lane packets a closed chunk may see go out first: up to 4 already in the output queue
  -- (header + 1 beat each), the one being read, 3 picked ahead by the round robin.
  constant FAIR_BOUND : natural := 8;

  constant RP_IDLE    : integer := 0;                                  -- nothing in flight
  constant RP_FILL    : integer := 1;                                  -- a chunk partly written
  constant RP_FULL    : integer := 2;                                  -- a lane's both chunks full
  constant RP_PACKET  : integer := 3;                                  -- output mid packet
  constant RP_HOLD    : integer := 4;                                  -- output beat held

  constant LM_ROTATE  : integer := 0;                                  -- lane order of nl_top
  constant LM_RANDOM  : integer := 1;
  constant LM_BURST   : integer := 2;
  constant RM_READY   : integer := 0;
  constant RM_RANDOM  : integer := 1;
  constant RM_HOLD    : integer := 2;                                  -- long iReady = 0 holds

  signal clk    : std_logic := '0';
  signal rst    : std_logic := '1';
  signal iLane  : unsigned(1 downto 0) := (others => '0');
  signal iWord  : std_logic_vector(OUT_WIDTH - 1 downto 0) := (others => '0');
  signal iValid : std_logic := '0';
  signal iBytes : unsigned(BW - 1 downto 0) := (others => '0');
  signal iLast  : std_logic := '0';
  signal oReady : std_logic_vector(3 downto 0);
  signal oData  : std_logic_vector(OUT_WIDTH - 1 downto 0);
  signal oValid : std_logic;
  signal oKeep  : std_logic_vector(NB - 1 downto 0);
  signal oLast  : std_logic;
  signal iReady : std_logic := '1';

begin

  clk <= not clk after CLK_PERIOD / 2;

  dut : entity work.nl_merge
    generic map (
      OUT_WIDTH   => OUT_WIDTH,
      CHUNK_BYTES => CHUNK + 8,
      BAND_ID     => BAND_ID
    )
    port map (
      iClk   => clk,
      iRst   => rst,
      iLane  => iLane,
      iWord  => iWord,
      iValid => iValid,
      iBytes => iBytes,
      iLast  => iLast,
      oReady => oReady,
      oData  => oData,
      oValid => oValid,
      oKeep  => oKeep,
      oLast  => oLast,
      iReady => iReady
    );

  stim : process is

    type t_int is array (0 to LANES - 1) of integer;
    type t_bool is array (0 to LANES - 1) of boolean;
    type t_word is array (0 to LANES - 1) of std_logic_vector(OUT_WIDTH - 1 downto 0);
    type t_pkt is array (0 to CHUNK + 8 + NB - 1) of natural range 0 to 255;

    constant SB_BYTE  : ScoreboardIDArrayType := NewID("lane byte SB", LANES);
    constant SB_CHUNK : ScoreboardIDArrayType := NewID("lane chunk SB", LANES);

    variable rv       : RandomPType;
    variable reqChunk : AlertLogIDType;
    variable reqOrder : AlertLogIDType;
    variable reqFair  : AlertLogIDType;
    variable reqAxis  : AlertLogIDType;
    variable reqRst   : AlertLogIDType;
    variable covLen   : CoverageIDType;
    variable covLane  : CoverageIDType;
    variable covMode  : CoverageIDType;
    variable covImg   : CoverageIDType;
    variable covRst   : CoverageIDType;

    -- Per-lane input
    variable slicesLeft : t_int  := (others => 0);
    variable remBytes   : t_int  := (others => 0);                     -- of the slice being sent
    variable chunkBytes : t_int  := (others => 0);                     -- written to the open chunk
    variable imgCount   : t_int  := (others => 0);
    variable beatReady  : t_bool := (others => false);                 -- a beat is presented
    variable beatWord   : t_word;
    variable beatBytes  : t_int  := (others => 0);
    variable oneByte    : boolean := false;                            -- wrap run: 1-byte slices
    -- Per-lane output
    variable pending    : t_int  := (others => 0);                     -- chunks closed, not out
    variable waitCnt    : t_int  := (others => 0);
    variable seenImg    : t_int  := (others => -1);

    variable pkt        : t_pkt;
    variable pktLen     : natural := 0;
    variable heldValid  : boolean := false;
    variable heldData   : std_logic_vector(OUT_WIDTH - 1 downto 0);
    variable heldKeep   : std_logic_vector(NB - 1 downto 0);
    variable heldLast   : std_logic;

    variable laneMode   : integer := LM_ROTATE;
    variable rdyMode    : integer := RM_READY;
    variable burstLane  : natural := 0;
    variable burstLeft  : natural := 0;
    variable holdLeft   : natural := 0;
    variable s          : natural := 0;
    variable cycles     : natural := 0;
    variable packets    : natural := 0;
    variable dropped    : std_logic_vector(32 downto 0);
    variable droppedB   : std_logic_vector(7 downto 0);

    impure function new_slice_len return positive is
    begin
      if (oneByte) then
        return 1;
      end if;
      case rv.DistValInt(((0, 1), (1, 1), (2, 1), (3, 2), (4, 3))) is
        when 0      => return 1;
        when 1      => return CHUNK;
        when 2      => return CHUNK * rv.RandInt(2, 3);
        when 3      => return CHUNK * rv.RandInt(0, 2) + rv.RandInt(1, CHUNK - 1);
        when others => return rv.RandInt(1, 3 * CHUNK);
      end case;
    end function new_slice_len;

    -- Lane l's next beat, kept until accepted
    procedure make_beat (l : natural) is
    begin
      if (remBytes(l) = 0) then
        remBytes(l)   := new_slice_len;
        slicesLeft(l) := slicesLeft(l) - 1;
      end if;
      beatWord(l)  := rv.RandSlv(OUT_WIDTH);
      beatBytes(l) := minimum(NB, remBytes(l));
      beatReady(l) := true;
    end procedure make_beat;

    impure function has_work (l : natural) return boolean is
    begin
      return beatReady(l) or remBytes(l) > 0 or slicesLeft(l) > 0;
    end function has_work;

    procedure pick_lane (l : out natural) is
    begin
      case laneMode is
        when LM_ROTATE =>
          l := s;
        when LM_BURST =>
          if (burstLeft = 0 or not has_work(burstLane)) then
            burstLane := rv.RandInt(0, LANES - 1);
            burstLeft := rv.RandInt(1, 4 * CHUNK_BEATS);
          end if;
          burstLeft := burstLeft - 1;
          l         := burstLane;
        when others =>
          l := rv.RandInt(0, LANES - 1);
      end case;
    end procedure pick_lane;

    procedure drive is
      variable l : natural;
    begin
      pick_lane(l);
      iLane  <= to_unsigned(l, 2);
      iValid <= '0';
      iLast  <= '0';
      if (has_work(l) and (laneMode = LM_ROTATE or rv.DistValInt(((1, 7), (0, 1))) = 1)) then
        if (not beatReady(l)) then
          make_beat(l);
        end if;
        iValid <= '1';
        iWord  <= beatWord(l);
        iBytes <= to_unsigned(beatBytes(l), BW);
        iLast  <= bool2bit(beatBytes(l) = remBytes(l));
      else
        iWord  <= rv.RandSlv(OUT_WIDTH);
        iBytes <= to_unsigned(rv.RandInt(1, NB), BW);
      end if;

      case rdyMode is
        when RM_READY =>
          iReady <= '1';
        when RM_RANDOM =>
          iReady <= bool2bit(rv.DistValInt(((1, 3), (0, 1))) = 1);
        when others =>
          if (holdLeft > 0) then
            holdLeft := holdLeft - 1;
            iReady   <= '0';
          elsif (rv.DistValInt(((1, 1), (0, 31))) = 1) then
            holdLeft := rv.RandInt(1, 12 * CHUNK_BEATS + 16);
            iReady   <= '0';
          else
            iReady <= '1';
          end if;
      end case;
    end procedure drive;

    -- Input accepted on lane l: bytes and, on a chunk close, the expected chunk
    procedure accept (l : natural) is
      variable n    : natural;
      variable last : boolean;
    begin
      n := beatBytes(l);
      for i in 0 to n - 1 loop
        Push(SB_BYTE(SB_BYTE'low + l), beatWord(l)(OUT_WIDTH - 1 - 8 * i downto OUT_WIDTH - 8 * (i + 1)));
      end loop;
      remBytes(l)   := remBytes(l) - n;
      chunkBytes(l) := chunkBytes(l) + n;
      last          := remBytes(l) = 0;
      if (last or chunkBytes(l) = CHUNK) then
        Push(SB_CHUNK(SB_CHUNK'low + l), bool2bit(last) &
             std_logic_vector(to_unsigned(imgCount(l) mod 65536, 16)) &
             std_logic_vector(to_unsigned(chunkBytes(l), 16)));
        pending(l)    := pending(l) + 1;
        chunkBytes(l) := 0;
        if (last) then
          imgCount(l) := imgCount(l) + 1;
        end if;
      end if;
      beatReady(l) := false;
    end procedure accept;

    procedure check_packet is
      variable slice : natural;
      variable last  : std_logic;
      variable img   : natural;
      variable len   : natural;
    begin
      AffirmIf(reqChunk, pktLen >= 9, "packet holds a header and a payload byte");
      if (pktLen < 9) then
        return;
      end if;
      slice := (pkt(2) / 4) mod 4;
      last  := bool2bit((pkt(2) / 2) mod 2 = 1);
      img   := pkt(4) * 256 + pkt(5);
      len   := pkt(6) * 256 + pkt(7);
      AffirmIf(reqChunk, pkt(0) = 16#FF# and pkt(1) = 16#B0#, "header marker FF B0");
      AffirmIf(reqChunk, pkt(2) / 16 = 1 and pkt(2) mod 2 = 0, "header version 1, reserved 0");
      AffirmIfEqual(reqChunk, pkt(3), BAND_ID, "header band");
      AffirmIfEqual(reqChunk, pktLen, 8 + len, "packet = 8 + length bytes, oLast at the chunk end");
      Check(SB_CHUNK(SB_CHUNK'low + slice), last & std_logic_vector(to_unsigned(img, 16)) &
            std_logic_vector(to_unsigned(len, 16)));
      for i in 8 to pktLen - 1 loop
        Check(SB_BYTE(SB_BYTE'low + slice), std_logic_vector(to_unsigned(pkt(i), 8)));
      end loop;
      AffirmIf(reqOrder, seenImg(slice) = img or (seenImg(slice) + 1) mod 65536 = img or seenImg(slice) < 0,
               "slice " & to_string(slice) & ": image_count in order");
      seenImg(slice) := img;

      if (last = '1') then
        if (len = 1) then
          ICover(covLen, 0);
        elsif (len = CHUNK) then
          ICover(covLen, 1);
        else
          ICover(covLen, 2);
        end if;
        ICover(covLen, 3 + boolean'pos(pktLen mod NB /= 0));
      else
        ICover(covLen, 5);
      end if;
      if (img = 0) then
        ICover(covImg, (slice, 0));
      elsif (img = 65535) then
        ICover(covImg, (slice, 1));
      end if;
      ICover(covLane, (slice, 0));

      for l in 0 to LANES - 1 loop
        if (l /= slice and pending(l) > 0) then
          waitCnt(l) := waitCnt(l) + 1;
          AffirmIf(reqFair, waitCnt(l) <= FAIR_BOUND, "lane " & to_string(l) & ": waited " &
                   to_string(waitCnt(l)) & " packets with a chunk closed");
        end if;
      end loop;
      pending(slice) := pending(slice) - 1;
      waitCnt(slice) := 0;
      packets := packets + 1;
    end procedure check_packet;

    procedure step is
      variable l  : natural;
      variable nk : natural;
      variable ok : boolean;
    begin
      wait for 1 ns;
      drive;
      wait until rising_edge(clk);
      cycles := cycles + 1;
      l      := to_integer(iLane);

      if (iValid = '1') then
        if (oReady(l) = '1') then
          accept(l);
        else
          ICover(covLane, (l, 1));
        end if;
      end if;
      if (oReady = "0000") then
        ICover(covMode, 3);
      end if;

      if (heldValid) then
        AffirmIf(reqAxis, oValid = '1' and oData = heldData and oKeep = heldKeep and oLast = heldLast,
                 "beat held while iReady is low");
      end if;
      heldValid := oValid = '1' and iReady = '0';
      if (heldValid) then
        heldData := oData;
        heldKeep := oKeep;
        heldLast := oLast;
        ICover(covMode, 4);
      end if;

      if (oValid = '1' and iReady = '1') then
        nk := 0;
        ok := true;
        for i in NB - 1 downto 0 loop
          if (oKeep(i) = '1') then
            ok := ok and nk = NB - 1 - i;
            nk := nk + 1;
          elsif (oKeep(i) /= '0') then
            ok := false;
          end if;
        end loop;
        AffirmIf(reqAxis, ok and nk >= 1 and (oLast = '1' or nk = NB),
                 "oKeep contiguous from the MSB, all ones except on oLast");
        for i in 0 to nk - 1 loop
          if (pktLen <= pkt'high) then
            pkt(pktLen) := to_integer(unsigned(oData(OUT_WIDTH - 1 - 8 * i downto OUT_WIDTH - 8 * (i + 1))));
          end if;
          pktLen := pktLen + 1;
        end loop;
        AffirmIf(reqChunk, pktLen <= pkt'high + 1, "packet no longer than a chunk + header");
        if (oLast = '1') then
          check_packet;
          pktLen := 0;
        end if;
      end if;

      s := (s + 1) mod LANES;
    end procedure step;

    impure function all_idle return boolean is
    begin
      for l in 0 to LANES - 1 loop
        if (has_work(l) or pending(l) > 0) then
          return false;
        end if;
      end loop;
      return pktLen = 0 and oValid = '0';
    end function all_idle;

    procedure run_until_idle (limit : natural) is
    begin
      while not all_idle loop
        step;
        exit when cycles > limit;
      end loop;
      AffirmIf(GetAlertLogID("Completion"), all_idle, "every chunk out (cycle " & to_string(cycles) & ")");
    end procedure run_until_idle;

    procedure do_reset (n : positive) is
    begin
      rst    <= '1';
      iValid <= '0';
      for i in 1 to n loop
        wait until rising_edge(clk);
      end loop;
      rst <= '0';
      s   := 0;
    end procedure do_reset;

    impure function phase_hit (p : integer) return boolean is
    begin
      case p is
        when RP_FILL =>
          -- 1-beat payloads are never partly written: then a slice partly sent
          for l in 0 to LANES - 1 loop
            if (chunkBytes(l) > 0 or (CHUNK_BEATS = 2 and remBytes(l) > 0 and not beatReady(l))) then
              return true;
            end if;
          end loop;
          return false;
        when RP_FULL   => return oReady /= "1111";
        when RP_PACKET => return pktLen > 0;
        when RP_HOLD   => return heldValid;
        when others    => return all_idle;
      end case;
    end function phase_hit;

    procedure abort_and_reset (p : integer) is
    begin
      do_reset(rv.RandInt(1, 4));
      wait for 1 ns;
      AffirmIf(reqRst, oValid = '0' and oReady = "1111", "reset in phase " & to_string(p) & ": outputs cleared");
      ICover(covRst, p);
      for l in 0 to LANES - 1 loop
        while not IsEmpty(SB_BYTE(SB_BYTE'low + l)) loop
          Pop(SB_BYTE(SB_BYTE'low + l), droppedB);
        end loop;
        while not IsEmpty(SB_CHUNK(SB_CHUNK'low + l)) loop
          Pop(SB_CHUNK(SB_CHUNK'low + l), dropped);
        end loop;
        slicesLeft(l) := 0;
        remBytes(l)   := 0;
        chunkBytes(l) := 0;
        imgCount(l)   := 0;
        beatReady(l)  := false;
        pending(l)    := 0;
        waitCnt(l)    := 0;
        seenImg(l)    := -1;
      end loop;
      pktLen    := 0;
      heldValid := false;
      holdLeft  := 0;
    end procedure abort_and_reset;

    procedure set_slices (n : natural) is
    begin
      for l in 0 to LANES - 1 loop
        slicesLeft(l) := n;
      end loop;
    end procedure set_slices;

    variable ph    : integer;
    variable hit   : boolean;
    variable limit : natural;

  begin

    SetAlertLogName("tb_nl_merge_osvvm");
    SetLogEnable(PASSED, FALSE);
    rv.InitSeed(rv'instance_name);
    reqChunk := GetReqID("NL.Chunks", 1000);
    reqOrder := GetReqID("NL.MergeOrder", 1000);
    reqFair  := GetReqID("NL.MergeFair", 1000);
    reqAxis  := GetReqID("NL.AxisOut", 1000);
    reqRst   := GetReqID("NL.Reset", N_RST);

    covLen := NewID("G2.MinMax G4.Stream chunk length");
    AddBins(covLen, "last chunk, 1 byte", GenBin(0));
    AddBins(covLen, "last chunk, full payload", GenBin(1));
    AddBins(covLen, "last chunk, in between", GenBin(2));
    AddBins(covLen, "last beat full", GenBin(3));
    AddBins(covLen, "last beat partial", GenBin(4));
    AddBins(covLen, "middle chunk", 10, GenBin(5));

    covLane := NewID("G4.Lane lane x event");
    SetFieldName(covLane, "lane", "event");
    covImg := NewID("G2.MinMax G4.Lane lane x image_count");
    SetFieldName(covImg, "lane", "end");
    for l in 0 to LANES - 1 loop
      AddCross(covLane, "lane=" & to_string(l) & " / chunk out", 10, GenBin(l), GenBin(0));
      AddCross(covLane, "lane=" & to_string(l) & " / input blocked", GenBin(l), GenBin(1));
      AddCross(covImg, "lane=" & to_string(l) & " / image_count 0", GenBin(l), GenBin(0));
      AddCross(covImg, "lane=" & to_string(l) & " / image_count 65535", GenBin(l), GenBin(1));
    end loop;

    covMode := NewID("G4.Stream lane order x output stress");
    AddBins(covMode, "lane order rotate", GenBin(0));
    AddBins(covMode, "lane order random", GenBin(1));
    AddBins(covMode, "lane order burst", GenBin(2));
    AddBins(covMode, "all lanes blocked", GenBin(3));
    AddBins(covMode, "output held", GenBin(4));

    covRst := NewID("G5.States resetPhase");
    AddBins(covRst, "reset while idle", 3, GenBin(RP_IDLE));
    AddBins(covRst, "reset with a chunk (or slice) partly written", 3, GenBin(RP_FILL));
    AddBins(covRst, "reset with a lane blocked", 3, GenBin(RP_FULL));
    AddBins(covRst, "reset mid packet", 3, GenBin(RP_PACKET));
    AddBins(covRst, "reset with output held", 3, GenBin(RP_HOLD));

    do_reset(4);

    -- Random slices under every lane order x iReady mode
    for lm in LM_ROTATE to LM_BURST loop
      for rm in RM_READY to RM_HOLD loop
        laneMode := lm;
        rdyMode  := rm;
        ICover(covMode, lm);
        set_slices(N_SLICES / 3 + 1);
        run_until_idle(cycles + 2_000_000);
      end loop;
    end loop;

    -- image_count 0 .. 65535 and back to 0 on every lane
    laneMode := LM_ROTATE;
    rdyMode  := RM_READY;
    oneByte  := true;
    set_slices(65536 - imgCount(0) + 1);
    run_until_idle(cycles + 4_000_000);
    oneByte  := false;

    -- Reset rounds: traffic until the phase, reset, then a scored recovery
    for r in 0 to N_RST - 1 loop
      if (r < 15) then
        ph := r mod 5;
      else
        ph := rv.RandInt(RP_IDLE, RP_HOLD);
      end if;
      laneMode := rv.RandInt(LM_ROTATE, LM_BURST);
      rdyMode  := rv.RandInt(RM_RANDOM, RM_HOLD);
      set_slices(3);
      hit   := false;
      limit := cycles + 1_000_000;
      while not hit loop
        step;
        hit := phase_hit(ph) and (ph = RP_IDLE or rv.DistValInt(((1, 1), (0, 3))) = 1);
        if (ph /= RP_IDLE and slicesLeft(0) = 0) then
          set_slices(1);                                               -- keep traffic until the phase
        end if;
        exit when cycles > limit;
      end loop;
      AffirmIf(reqRst, hit, "reset round " & to_string(r) & ": phase " & to_string(ph) & " reached");
      abort_and_reset(ph);
      set_slices(2);
      run_until_idle(cycles + 1_000_000);
    end loop;

    for l in 0 to LANES - 1 loop
      AffirmIf(GetAlertLogID("ScoreboardCompletion"), IsEmpty(SB_BYTE(SB_BYTE'low + l)) and
               IsEmpty(SB_CHUNK(SB_CHUNK'low + l)), "lane " & to_string(l) & " scoreboards drained");
      AffirmIfEqual(GetAlertLogID("ScoreboardCompletion"), GetErrorCount(SB_BYTE(SB_BYTE'low + l)) +
                    GetErrorCount(SB_CHUNK(SB_CHUNK'low + l)), 0,
                    "lane " & to_string(l) & " scoreboard mismatches");
    end loop;

    WriteBin(covLen);
    WriteBin(covLane);
    WriteBin(covImg);
    WriteBin(covMode);
    WriteBin(covRst);
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covLen), "chunk length coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covLane), "lane x event coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covImg), "image_count coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covMode), "stress mode coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covRst), "reset-phase coverage closed");

    end_of_test("tb_nl_merge_osvvm");
    wait;

  end process stim;

  watchdog : process is
  begin
    wait for 500 ms;
    Alert(GetAlertLogID("Watchdog"), "tb_nl_merge_osvvm: watchdog timeout", FAILURE);
    std.env.stop;
  end process watchdog;

end architecture sim;
