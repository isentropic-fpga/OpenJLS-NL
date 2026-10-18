-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- tb_nl_framer_osvvm: nl_framer, tb_jls_framer_osvvm ported to the lane ring, plus NEAR
--
-- One clock is one lane's turn. Each lane frames its own images (own size and NEAR) against
-- header(P, Y, X, NEAR) ++ payload ++ FF D9 in its own scoreboard, oLast on the D9. Input honours
-- oReady; per-lane iReady bursts are random, and a held beat must survive until the lane's next
-- turn. Images: a minimal back-to-back pair (FF D9 only), random images, some chained before the
-- previous oLast, then N_RST reset rounds with a scored recovery image per lane.
--
-- Guidelines:
--   G2.Clamp    N/A: no accumulator
--   G3.Stim     G2.MinMax field x value (NEAR = 255)
--   G4.Lane     every G4 bin crossed with the lane
--   G4.Config   G2.MinMax field x value
--   G4 other    N/A: no Edge / Run / Coding / Branch logic

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

entity tb_nl_framer_osvvm is
  generic (
    BITNESS   : natural range 8 to 16 := 12;
    OUT_WIDTH : natural := 64
  );
end entity tb_nl_framer_osvvm;

architecture sim of tb_nl_framer_osvvm is

  -- Config as nl_top.
  constant IN_WIDTH   : natural := 32;
  constant MAX_W      : natural := 4096 / 4;
  constant MAX_H      : natural := 4096;
  constant BYTES_IN   : natural := IN_WIDTH / 8;
  constant BYTES_OUT  : natural := OUT_WIDTH / 8;
  constant WDIM       : natural := log2ceil(MAX_W + 1);
  constant HDIM       : natural := log2ceil(MAX_H + 1);
  constant BE_IN_W    : natural := log2ceil(BYTES_IN + 1);
  constant BE_OUT_W   : natural := log2ceil(BYTES_OUT + 1);
  constant LANES      : natural := 4;
  constant CLK_PERIOD : time    := CLK_PERIOD_DEFAULT;

  constant N_RANDOM   : natural := 30;
  constant N_IMAGES   : natural := 2 + N_RANDOM;                       -- minimal pair + random
  constant MAX_WORDS  : natural := maximum(24, 2 * BYTES_OUT);
  constant HDR_LEN    : natural := 25;

  -- Reset rounds: phase hit when rst rises.
  constant N_RST      : natural := 25;
  constant RP_IDLE    : integer := 0;                                -- nothing in flight
  constant RP_HEADER  : integer := 1;                                -- header bytes being emitted
  constant RP_DATA    : integer := 2;                                -- payload, image still open
  constant RP_EOI     : integer := 3;                                -- EoI sent, oLast pending
  constant RP_HOLD    : integer := 4;                                -- output beat held (iReady low)

  signal clk     : std_logic := '0';
  signal rst     : std_logic := '1';
  signal iStart  : std_logic := '0';
  signal iNear   : unsigned(7 downto 0) := (others => '0');
  signal iWidth  : unsigned(WDIM - 1 downto 0) := (others => '0');
  signal iHeight : unsigned(HDIM - 1 downto 0) := (others => '0');
  signal iEoi    : std_logic := '0';
  signal iWord   : std_logic_vector(IN_WIDTH - 1 downto 0) := (others => '0');
  signal iValid  : std_logic := '0';
  signal iByteEn : unsigned(BE_IN_W - 1 downto 0) := (others => '0');
  signal oReady  : std_logic;
  signal oWord   : std_logic_vector(OUT_WIDTH - 1 downto 0);
  signal oValid  : std_logic;
  signal oByteEn : unsigned(BE_OUT_W - 1 downto 0);
  signal oLast   : std_logic;
  signal iReady  : std_logic := '1';

  -- JPEG-LS frame header (T.87 C.2.2/C.2.3), independent of the RTL ROM.
  function header_byte (
    idx  : natural;
    w    : natural;
    h    : natural;
    near : natural
  ) return std_logic_vector is
  begin

    case idx is

      when 0  => return x"FF";                                           -- SOI
      when 1  => return x"D8";
      when 2  => return x"FF";                                           -- SOF55
      when 3  => return x"F7";
      when 4  => return x"00";                                           -- Lf = 11
      when 5  => return x"0B";
      when 6  => return std_logic_vector(to_unsigned(BITNESS, 8));       -- P
      when 7  => return std_logic_vector(to_unsigned(h / 256, 8));       -- Y hi
      when 8  => return std_logic_vector(to_unsigned(h mod 256, 8));     -- Y lo
      when 9  => return std_logic_vector(to_unsigned(w / 256, 8));       -- X hi
      when 10 => return std_logic_vector(to_unsigned(w mod 256, 8));     -- X lo
      when 11 => return x"01";                                           -- Nf
      when 12 => return x"01";                                           -- C1
      when 13 => return x"11";                                           -- H1V1
      when 14 => return x"00";                                           -- Tq1
      when 15 => return x"FF";                                           -- SOS
      when 16 => return x"DA";
      when 17 => return x"00";                                           -- Ls = 8
      when 18 => return x"08";
      when 19 => return x"01";                                           -- Ns
      when 20 => return x"01";                                           -- Cs1
      when 21 => return x"00";                                           -- Tm1
      when 22 => return std_logic_vector(to_unsigned(near, 8));          -- NEAR
      when 23 => return x"00";                                           -- ILV
      when others => return x"00";                                       -- Al/Ah
    end case;

  end function header_byte;

begin

  clk <= not clk after CLK_PERIOD / 2;

  dut : entity work.nl_framer(behavioral)
    generic map (
      BITNESS          => BITNESS,
      IN_WIDTH         => IN_WIDTH,
      OUT_WIDTH        => OUT_WIDTH,
      MAX_IMAGE_WIDTH  => MAX_W,
      MAX_IMAGE_HEIGHT => MAX_H
    )
    port map (
      iClk         => clk,
      iRst         => rst,
      iStart       => iStart,
      iNear        => iNear,
      iImageWidth  => iWidth,
      iImageHeight => iHeight,
      iEoi         => iEoi,
      iWord        => iWord,
      iValid       => iValid,
      iByteEnable  => iByteEn,
      oReady       => oReady,
      oWord        => oWord,
      oValid       => oValid,
      oByteEnable  => oByteEn,
      oLast        => oLast,
      iReady       => iReady
    );

  stim : process is

    type t_int is array (0 to LANES - 1) of integer;
    type t_bool is array (0 to LANES - 1) of boolean;
    type t_out is array (0 to LANES - 1) of std_logic_vector(OUT_WIDTH - 1 downto 0);

    constant SB : ScoreboardIDArrayType := NewID("lane SB", LANES);

    variable rv       : RandomPType;
    variable req      : AlertLogIDType;
    variable reqRst   : AlertLogIDType;
    variable covRst   : CoverageIDType;
    variable covRange : CoverageIDType;
    variable covBeat  : CoverageIDType;
    variable covChain : CoverageIDType;
    variable covHold  : CoverageIDType;

    -- Per-lane image driver.
    variable dimW      : t_int := (others => 8);
    variable dimH      : t_int := (others => 8);
    variable dimN      : t_int := (others => 0);
    variable imgIdx    : t_int := (others => 0);                     -- images started
    variable imgEnd    : t_int := (others => N_IMAGES);              -- images to play
    variable outBytes  : t_int := (others => 0);                     -- bytes out since last oLast
    variable wordsLeft : t_int := (others => 0);                     -- words of the image being sent
    variable firstWord : t_bool := (others => false);
    variable startSent : t_bool := (others => false);
    variable chain     : t_bool := (others => false);                -- next image may start before oLast
    variable outst     : t_int := (others => 0);                     -- EoI sent, oLast not seen
    variable imgsDone  : t_int := (others => 0);
    variable minimal   : t_bool := (others => false);                -- payload = footer only
    variable notRdy    : t_int := (others => 0);
    -- Per-lane AXI hold check.
    variable held      : t_bool := (others => false);
    variable heldWord  : t_out;
    variable heldBe    : t_int;
    variable heldLast  : t_bool;

    variable s         : natural := 0;
    variable ph        : integer;
    variable hit       : boolean;
    variable dropped   : std_logic_vector(8 downto 0);
    variable cycles    : natural := 0;
    variable nb        : natural;
    variable byte      : std_logic_vector(7 downto 0);
    variable lastByte  : std_logic_vector(7 downto 0);
    variable word      : std_logic_vector(IN_WIDTH - 1 downto 0);
    variable be        : natural;
    variable rdy       : std_logic;
    variable sending   : boolean;

    -- Begin lane l's next image: new dims unless chained onto a pending one.
    procedure begin_image (l : natural) is
    begin
      if (outst(l) = 0) then
        -- Each field at its min / max a quarter of the time each (G2).
        dimW(l) := rv.DistValInt(((1, 1), (MAX_W, 1), (rv.RandInt(1, MAX_W), 2)));
        dimH(l) := rv.DistValInt(((1, 1), (MAX_H, 1), (rv.RandInt(1, MAX_H), 2)));
        dimN(l) := rv.DistValInt(((0, 1), (255, 1), (rv.RandInt(0, 255), 2)));
        ICover(covRange, (0, dimW(l)));
        ICover(covRange, (1, dimH(l)));
        ICover(covRange, (2, dimN(l)));
        ICover(covChain, (l, 0));
      else
        ICover(covChain, (l, 1));
      end if;
      minimal(l) := imgIdx(l) < 2 or (rv.DistValInt(((1, 1), (0, 15))) = 1);
      if (minimal(l)) then
        wordsLeft(l) := 1;
      else
        wordsLeft(l) := rv.RandInt(1, MAX_WORDS);
      end if;
      for k in 0 to HDR_LEN - 1 loop
        Push(SB(SB'low + l), '0' & header_byte(k, dimW(l), dimH(l), dimN(l)));
      end loop;
      firstWord(l) := true;
      startSent(l) := false;
      imgIdx(l)    := imgIdx(l) + 1;
    end procedure begin_image;

    -- Drive lane l's turn (signals sampled by the DUT at the next edge).
    procedure drive_slot (l : natural) is
    begin
      iNear   <= to_unsigned(dimN(l), 8);
      iWidth  <= to_unsigned(dimW(l), WDIM);
      iHeight <= to_unsigned(dimH(l), HDIM);
      iValid  <= '0';
      iEoi    <= '0';
      iStart  <= '0';
      iByteEn <= (others => '0');

      if (wordsLeft(l) = 0 and imgIdx(l) < imgEnd(l)) then
        if (outst(l) = 0 or (outst(l) = 1 and chain(l))) then
          begin_image(l);
        end if;
      end if;

      if (wordsLeft(l) > 0) then
        -- iStart alone a turn ahead, or with the first word.
        if (firstWord(l) and not startSent(l) and rv.DistValInt(((1, 1), (0, 3))) = 1) then
          iStart       <= '1';
          startSent(l) := true;
        elsif (oReady = '1' and rv.DistValInt(((1, 7), (0, 1))) = 1) then
          word := rv.RandSlv(IN_WIDTH);
          if (minimal(l)) then
            be := 0;
          elsif (wordsLeft(l) = 1) then
            be := rv.RandInt(0, BYTES_IN);
          else
            be := rv.RandInt(1, BYTES_IN);
          end if;
          iValid  <= '1';
          iWord   <= word;
          iByteEn <= to_unsigned(be, BE_IN_W);
          iStart  <= bool2bit(firstWord(l) and not startSent(l));
          for i in 0 to be - 1 loop
            Push(SB(SB'low + l), '0' & word(IN_WIDTH - 1 - i * 8 downto IN_WIDTH - (i + 1) * 8));
          end loop;
          firstWord(l) := false;
          wordsLeft(l) := wordsLeft(l) - 1;
          if (wordsLeft(l) = 0) then
            iEoi <= '1';
            Push(SB(SB'low + l), '0' & x"FF");
            Push(SB(SB'low + l), '1' & x"D9");
            outst(l) := outst(l) + 1;
            -- Minimal pair always chains; random images chain ~30%.
            chain(l) := imgIdx(l) = 1 or rv.DistValInt(((1, 3), (0, 7))) = 1;
          end if;
        end if;
      end if;

      -- Per-lane downstream readiness with not-ready bursts.
      if (notRdy(l) > 0) then
        notRdy(l) := notRdy(l) - 1;
        rdy       := '0';
      elsif (rv.DistValInt(((1, 1), (0, 15))) = 1) then
        notRdy(l) := rv.RandInt(1, 6);
        rdy       := '0';
      else
        rdy := bool2bit(rv.DistValInt(((1, 4), (0, 1))) = 1);
      end if;
      iReady <= rdy;
    end procedure drive_slot;

    -- One clock: drive lane s after outputs settle, sample at the edge.
    procedure step is
    begin
      wait for 1 ns;
      drive_slot(s);
      wait until rising_edge(clk);
      cycles := cycles + 1;

      -- Outputs this cycle belong to lane s.
      if (held(s)) then
        AffirmIf(GetAlertLogID("OutputHold"), oValid = '1', "lane " & to_string(s) & ": valid held until accepted");
        AffirmIfEqual(GetAlertLogID("OutputHold"), oWord, heldWord(s), "lane " & to_string(s) & ": data held");
        AffirmIfEqual(GetAlertLogID("OutputHold"), checked_integer(oByteEn), heldBe(s),
                      "lane " & to_string(s) & ": byte count held");
        AffirmIf(GetAlertLogID("OutputHold"), (oLast = '1') = heldLast(s), "lane " & to_string(s) & ": last held");
      end if;
      held(s) := oValid = '1' and iReady = '0';
      if (held(s)) then
        heldWord(s) := oWord;
        heldBe(s)   := checked_integer(oByteEn);
        heldLast(s) := oLast = '1';
        ICover(covHold, s);
      end if;

      if (oValid = '1' and iReady = '1') then
        nb := checked_integer(oByteEn);
        AffirmIf(GetAlertLogID("Protocol"), nb >= 1 and nb <= BYTES_OUT,
                 "lane " & to_string(s) & ": valid beat has legal byte count");
        ICover(covBeat, (s, boolean'pos(nb = BYTES_OUT)));
        for i in 0 to nb - 1 loop
          byte     := oWord(OUT_WIDTH - 1 - i * 8 downto OUT_WIDTH - (i + 1) * 8);
          lastByte := byte;
          outBytes(s) := outBytes(s) + 1;
          Check(SB(SB'low + s), bool2bit(oLast = '1' and i = nb - 1) & byte);
        end loop;
        if (oLast = '1') then
          AffirmIfEqual(req, lastByte, std_logic_vector'(x"D9"),
                        "lane " & to_string(s) & ": oLast on the FF D9 footer, image " & to_string(imgsDone(s)));
          AffirmIf(req, outst(s) > 0, "lane " & to_string(s) & ": oLast only for a closed image");
          outst(s)    := outst(s) - 1;
          imgsDone(s) := imgsDone(s) + 1;
          outBytes(s) := 0;
        end if;
      end if;

      s := (s + 1) mod LANES;
    end procedure step;

    impure function all_idle return boolean is
    begin
      for l in 0 to LANES - 1 loop
        if (wordsLeft(l) > 0 or outst(l) > 0 or imgIdx(l) < imgEnd(l)) then
          return false;
        end if;
      end loop;
      return true;
    end function all_idle;

    procedure do_reset (n : positive) is
    begin
      rst    <= '1';
      iValid <= '0';
      iStart <= '0';
      iEoi   <= '0';
      for i in 1 to n loop
        wait until rising_edge(clk);
      end loop;
      rst <= '0';
      s   := 0;
    end procedure do_reset;

    -- Phase of lane l as seen at its ports (header bytes counted from the
    -- last oLast; a chained image's header queues behind the open one).
    -- With BYTES_OUT >= HDR_LEN the header leaves in one beat, so the header
    -- phase is "iStart seen, header not yet out".
    impure function phase_hit (p : integer) return boolean is
    begin
      for l in 0 to LANES - 1 loop
        case p is
          when RP_HEADER =>
            if (outst(l) = 0 and outBytes(l) > 0 and outBytes(l) < HDR_LEN) then return true; end if;
            if (BYTES_OUT >= HDR_LEN and outst(l) = 0 and outBytes(l) = 0 and imgIdx(l) > imgsDone(l)
                and (startSent(l) or not firstWord(l))) then
              return true;
            end if;
          when RP_DATA   => if (outst(l) = 0 and outBytes(l) >= HDR_LEN and wordsLeft(l) > 0) then return true; end if;
          when RP_EOI    => if (outst(l) > 0) then return true; end if;
          when RP_HOLD   => if (held(l)) then return true; end if;
          when others    => null;
        end case;
      end loop;
      return p = RP_IDLE and all_idle;
    end function phase_hit;

    -- Reset now: aborted images' expected bytes are dropped, every lane
    -- restarts with one fresh image.
    procedure abort_and_reset (p : integer) is
    begin
      do_reset(rv.RandInt(1, 6));
      wait for 1 ns;
      AffirmIf(reqRst, oValid = '0' and oLast = '0', "reset in phase " & to_string(p) & ": stream cleared");
      ICover(covRst, p);
      for l in 0 to LANES - 1 loop
        while not IsEmpty(SB(SB'low + l)) loop
          Pop(SB(SB'low + l), dropped);
        end loop;
        wordsLeft(l) := 0;
        outst(l)     := 0;
        outBytes(l)  := 0;
        held(l)      := false;
        chain(l)     := false;
        notRdy(l)    := 0;
        imgsDone(l)  := imgIdx(l);                                     -- aborted images accounted
        imgEnd(l)    := imgIdx(l) + 1;
      end loop;
    end procedure abort_and_reset;

  begin

    SetAlertLogName("tb_nl_framer_osvvm");
    SetLogEnable(PASSED, FALSE);
    rv.InitSeed(rv'instance_name);
    req    := GetReqID("NL.LaneFrame", 2 * LANES * N_IMAGES);
    reqRst := GetReqID("NL.Reset", 2 * N_RST);

    covRst := NewID("G5.States resetPhase");
    AddBins(covRst, "reset while idle", 3, GenBin(RP_IDLE));
    AddBins(covRst, "reset during header", 3, GenBin(RP_HEADER));
    AddBins(covRst, "reset during payload", 3, GenBin(RP_DATA));
    AddBins(covRst, "reset with EoI pending", 3, GenBin(RP_EOI));
    AddBins(covRst, "reset with output held", 3, GenBin(RP_HOLD));

    covRange := NewID("G2.MinMax field x value");
    SetFieldName(covRange, "field", "value");
    AddCross(covRange, "X = 1", GenBin(0), GenBin(1));
    AddCross(covRange, "X = max", GenBin(0), GenBin(MAX_W));
    AddCross(covRange, "Y = 1", GenBin(1), GenBin(1));
    AddCross(covRange, "Y = max", GenBin(1), GenBin(MAX_H));
    AddCross(covRange, "NEAR = 0", GenBin(2), GenBin(0));
    AddCross(covRange, "NEAR = 255 (MSB set)", GenBin(2), GenBin(255));

    covBeat := NewID("G4.Stream lane x beat");
    SetFieldName(covBeat, "lane", "fullBeat");
    covChain := NewID("G4.Stream lane x chained");
    SetFieldName(covChain, "lane", "chained");
    covHold := NewID("G4.Stream lane backpressure hold");
    SetFieldName(covHold, "lane");
    for l in 0 to LANES - 1 loop
      AddCross(covBeat, "lane=" & to_string(l) & " / partial beat", GenBin(l), GenBin(0));
      AddCross(covBeat, "lane=" & to_string(l) & " / full beat", GenBin(l), GenBin(1));
      AddCross(covChain, "lane=" & to_string(l) & " / serial", GenBin(l), GenBin(0));
      AddCross(covChain, "lane=" & to_string(l) & " / chained (2 EoI queued)", GenBin(l), GenBin(1));
      AddBins(covHold, "lane=" & to_string(l), GenBin(l));
    end loop;

    do_reset(6);

    while not all_idle loop
      step;
      exit when cycles > 1_000_000;
    end loop;
    AffirmIf(GetAlertLogID("Completion"), all_idle, "all lanes framed every image");

    -- Reset rounds: run scored traffic until the target phase, reset, then a
    -- scored recovery image per lane.
    for r in 0 to N_RST - 1 loop
      if (r < 5 * 3) then
        ph := r mod 5;
      else
        ph := rv.RandInt(RP_IDLE, RP_HOLD);
      end if;
      for l in 0 to LANES - 1 loop
        imgEnd(l) := imgIdx(l) + 2;
      end loop;
      hit := false;
      while not hit loop
        step;
        hit := phase_hit(ph) and (ph = RP_IDLE or rv.DistValInt(((1, 1), (0, 3))) = 1);
        exit when cycles > 4_000_000;
      end loop;
      AffirmIf(reqRst, hit, "reset round " & to_string(r) & ": phase " & to_string(ph) & " reached");
      abort_and_reset(ph);
      while not all_idle loop
        step;
        exit when cycles > 4_000_000;
      end loop;
    end loop;

    for l in 0 to LANES - 1 loop
      AffirmIfEqual(GetAlertLogID("Completion"), imgsDone(l), imgIdx(l),
                    "lane " & to_string(l) & " images framed");
      AffirmIf(GetAlertLogID("ScoreboardCompletion"), IsEmpty(SB(SB'low + l)),
               "lane " & to_string(l) & " scoreboard drained");
      AffirmIfEqual(GetAlertLogID("ScoreboardCompletion"), GetErrorCount(SB(SB'low + l)), 0,
                    "lane " & to_string(l) & " scoreboard mismatches");
    end loop;

    WriteBin(covBeat);
    WriteBin(covChain);
    WriteBin(covHold);
    WriteBin(covRst);
    WriteBin(covRange);
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covBeat), "lane x beat coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covChain), "lane x chained coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covHold), "lane hold coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covRst), "reset-phase coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covRange), "header field range coverage closed");

    end_of_test("tb_nl_framer_osvvm");
    wait;

  end process stim;

  watchdog : process is
  begin
    wait for 50 ms;
    Alert(GetAlertLogID("Watchdog"), "tb_nl_framer_osvvm: watchdog timeout", FAILURE);
    std.env.stop;
  end process watchdog;

end architecture sim;
