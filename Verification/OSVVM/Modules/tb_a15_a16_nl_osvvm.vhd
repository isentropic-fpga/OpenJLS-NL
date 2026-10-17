-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- tb_a15_a16_nl_osvvm: a15_a16_nl_encode_run, with the TB owning the per-lane run state
--
-- Slot k belongs to lane k mod 4, as in nl_backend. The TB latches each lane's o*Next on its slot
-- and clears it after the lane's last pixel. Each lane plays its own runs, interleaved slot by slot,
-- and its bits are compared run by run with an A.15 / A.16 reference using that lane's RUNindex, so
-- state leaking between lanes shows up as a wrong bitstream. Lane 0 plays the OpenJLS directed
-- script (climb to 31, boundary at 31, break from 31); the others are random.
--
-- Requirements:
--   T87.A15          A.15 '1' bits
--   T87.A16          terminal: EOL '1', or '0' + J[RUNindex] bits; RI fields
--   NL.RunLaneState  per-lane state ports, idle / reset slots hold state
--
-- Guidelines:
--   G1.Req      T87.A15, T87.A16
--   G1.Golden   A.15 / A.16 C code as a bit-string reference (gen_expected)
--   G1.Nl       NL.RunLaneState
--   G2.Range    A15_A16_nl_encode_run.vhd range comments and asserts
--   G2.MinMax   G4.Run startIdx (RUNindex 0..30), G2.Clamp RUNindex (31); RUNcnt up to 40000
--   G2.Clamp    G2.Clamp RUNindex
--   G2.Checked  checked_* on all outputs
--   G3.Stim     N/A: run events, no signed node
--   G4.Run      G4.Run lane x terminal, lane x immediateBreak, startIdx; RItype is A.17 (nl_loop)
--   G4.Lane     every G4.Run bin crossed with the lane, G4.Stream lane x eoiReset
--   G4.Stream   G4.Stream lane x eoiReset
--   G4.Config   BITNESS 8 / 12 / 16 by the suite generics
--   G4 other    N/A: no Edge / Coding / Branch logic
--   G5          N/A: combinational, the state lives in nl_backend; reset slots hold state
--   G6.Code     fully covered

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use std.textio.all;
  use work.openjls_pkg.all;

library osvvm;
  context osvvm.OsvvmContext;

library tb_support;
  use tb_support.tb_support_pkg.all;

entity tb_a15_a16_nl_osvvm is
  generic (BITNESS : natural range 8 to 16 := 12);
end entity tb_a15_a16_nl_osvvm;

architecture sim of tb_a15_a16_nl_osvvm is

  constant RC_W       : natural := 16;                              -- nl_backend RUN_CNT
  constant CLK_PERIOD : time    := CLK_PERIOD_DEFAULT;
  constant MAXBITS    : natural := 8192;
  constant LANES      : natural := 4;

  constant T_BREAK    : integer := 0;
  constant T_EOL      : integer := 1;

  type t_buf is array (0 to LANES - 1) of string(1 to MAXBITS);
  type t_int is array (0 to LANES - 1) of integer;
  type t_bool is array (0 to LANES - 1) of boolean;
  type t_sl is array (0 to LANES - 1) of std_logic;

  -- Lane 0 directed script: (M, terminal, end of image).
  type t_run is record
    m    : natural;
    term : integer;
    eoi  : std_logic;
  end record t_run;
  type t_script is array (natural range <>) of t_run;
  constant SCRIPT : t_script := (
    (0, T_BREAK, '0'), (1, T_BREAK, '0'), (4, T_BREAK, '0'), (5, T_EOL, '0'),
    (2, T_BREAK, '1'),
    (800, T_EOL, '0'), (800, T_EOL, '0'), (800, T_EOL, '0'),
    (800, T_EOL, '0'), (800, T_EOL, '0'), (800, T_EOL, '0'),
    -- J[24..31] = 8..15: one run past 2^14 reaches 31, the next proves the
    -- index holds at 31 on a boundary, then a break leaves 30.
    (33100, T_EOL, '0'), (40000, T_EOL, '0'), (3, T_BREAK, '1')
  );

  signal clk            : std_logic := '0';
  signal sRst           : std_logic := '0';
  signal sRunIndex      : unsigned(4 downto 0);
  signal sNextBound     : unsigned(RC_W downto 0);
  signal sInRun         : std_logic;
  signal sRunIndexNext  : unsigned(4 downto 0);
  signal sNextBoundNext : unsigned(RC_W downto 0);
  signal sRunCnt        : unsigned(RC_W - 1 downto 0);
  signal sRunHit        : std_logic;
  signal sRunCont       : std_logic;
  signal sModeIsRun     : std_logic;
  signal sIxPx            : unsigned(BITNESS - 1 downto 0);
  signal sRaPx            : unsigned(BITNESS - 1 downto 0);
  signal sRbPx            : unsigned(BITNESS - 1 downto 0);
  signal oRawValid      : std_logic;
  signal oRawSuffixLen  : unsigned(4 downto 0);
  signal oRawSuffixVal  : unsigned(RC_W - 1 downto 0);
  signal oRiValid       : std_logic;
  signal oRiIx          : unsigned(BITNESS - 1 downto 0);
  signal oRiRa          : unsigned(BITNESS - 1 downto 0);
  signal oRiRb          : unsigned(BITNESS - 1 downto 0);
  signal oRiRunIndex    : unsigned(4 downto 0);
  signal oInRunNext     : std_logic;

  -- Append nbits of val (MSB-first) as '0'/'1' chars onto buf.
  procedure app (
    buf   : inout string;
    len   : inout natural;
    val   : natural;
    nbits : natural
  ) is
  begin

    for i in nbits - 1 downto 0 loop

      len := len + 1;
      if ((val / (2 ** i)) mod 2 = 1) then
        buf(len) := '1';
      else
        buf(len) := '0';
      end if;

    end loop;

  end procedure app;

  -- T.87 A.15 + A.16 reference for one run: sequential transcription of the
  -- C code (RUNcnt, RUNindex, J, AppendToBitStream). termIdx is RUNindex at
  -- the terminal (before the A.16 decrement), newIdx the index after the run,
  -- n15 the number of A.15 bits, sat31 an A.15 boundary taken at RUNindex 31.
  procedure gen_expected (
    r0      : natural;
    m       : natural;
    term    : integer;
    buf     : inout string;
    len     : inout natural;
    termIdx : out natural;
    newIdx  : out natural;
    n15     : out natural;
    sat31   : out boolean
  ) is

    variable RUNindex : natural;
    variable RUNcnt   : natural;

  begin

    len      := 0;
    RUNindex := r0;
    RUNcnt   := m;
    sat31    := false;

    -- A.15
    while (RUNcnt >= 2 ** TB_J_TABLE(RUNindex)) loop
      app(buf, len, 1, 1);                                           -- AppendToBitStream(1, 1)
      RUNcnt := RUNcnt - 2 ** TB_J_TABLE(RUNindex);
      if (RUNindex < 31) then
        RUNindex := RUNindex + 1;
      else
        sat31 := true;
      end if;
    end loop;

    n15     := len;
    termIdx := RUNindex;

    -- A.16
    if (term = T_BREAK) then
      app(buf, len, 0, 1);                                           -- AppendToBitStream(0, 1)
      app(buf, len, RUNcnt, TB_J_TABLE(RUNindex));                   -- AppendToBitStream(RUNcnt, J[RUNindex])
      if (RUNindex > 0) then
        RUNindex := RUNindex - 1;
      end if;
    elsif (RUNcnt > 0) then
      app(buf, len, 1, 1);                                           -- AppendToBitStream(1, 1)
    end if;

    newIdx := RUNindex;

  end procedure gen_expected;

begin

  clk <= not clk after CLK_PERIOD / 2;

  dut : entity work.a15_a16_nl_encode_run(behavioral)
    generic map (
      BITNESS       => BITNESS,
      RUN_CNT_WIDTH => RC_W
    )
    port map (
      iRst           => sRst,
      iRunIndex      => sRunIndex,
      iNextBound     => sNextBound,
      iInRun         => sInRun,
      oRunIndexNext  => sRunIndexNext,
      oNextBoundNext => sNextBoundNext,
      iRunCnt        => sRunCnt,
      iRunHit        => sRunHit,
      iRunContinue   => sRunCont,
      iModeIsRun     => sModeIsRun,
      iIx            => sIxPx,
      iRa            => sRaPx,
      iRb            => sRbPx,
      oRawValid      => oRawValid,
      oRawSuffixLen  => oRawSuffixLen,
      oRawSuffixVal  => oRawSuffixVal,
      oRiValid       => oRiValid,
      oRiIx          => oRiIx,
      oRiRa          => oRiRa,
      oRiRb          => oRiRb,
      oRiRunIndex    => oRiRunIndex,
      oInRunNext     => oInRunNext
    );

  stim : process is

    variable rv      : RandomPType;
    variable covTerm : CoverageIDType;
    variable covImm  : CoverageIDType;
    variable covEoi  : CoverageIDType;
    variable covIdx  : CoverageIDType;
    variable req     : AlertLogIDType;
    variable req16   : AlertLogIDType;
    variable reqLane : AlertLogIDType;
    variable covAcc  : CoverageIDType;
    variable expN15  : t_int;
    variable expSat  : t_bool;
    variable n15     : natural;
    variable sat     : boolean;

    -- Lane state owned by the TB (as nl_backend's p_run_state).
    variable stIdx   : t_int;
    variable stBound : t_int;
    variable stIn    : t_sl;
    -- Lane run in progress.
    variable busy    : t_bool := (others => false);
    variable runM    : t_int;
    variable runTerm : t_int;
    variable runEoi  : t_sl;
    variable pos     : t_int;
    variable nMatch  : t_int;
    variable carried : t_int := (others => 0);
    variable expBuf  : t_buf;
    variable expLen  : t_int;
    variable actBuf  : t_buf;
    variable actLen  : t_int;
    variable termIdx : t_int;
    variable newIdx  : t_int;
    variable step    : integer := 0;
    variable runs    : natural := 0;
    variable lane    : natural;
    variable len     : natural;
    variable ti, ni  : natural;
    variable m       : natural;
    variable term    : integer;
    variable last    : boolean;
    variable px      : integer;
    variable msg     : line;

    procedure clear_lane (l : natural) is
    begin
      stIdx(l)   := 0;
      stBound(l) := 1;
      stIn(l)    := '0';
    end procedure clear_lane;

    procedure new_run (l : natural; m0 : natural; t0 : integer; e0 : std_logic) is
    begin
      runM(l)    := m0;
      runTerm(l) := t0;
      runEoi(l)  := e0;
      pos(l)     := 0;
      if (t0 = T_BREAK) then
        nMatch(l) := m0;
      else
        nMatch(l) := m0 - 1;                                        -- EOL: last pixel is the terminal
      end if;
      len := 0;
      gen_expected(carried(l), m0, t0, expBuf(l), len, ti, ni, n15, sat);
      expLen(l)  := len;
      expN15(l)  := n15;
      expSat(l)  := sat;
      termIdx(l) := ti;
      newIdx(l)  := ni;
      actLen(l)  := 0;
      busy(l)    := true;
    end procedure new_run;

    procedure random_run (l : natural) is
    begin
      term := rv.RandInt(0, 1);
      if (term = T_BREAK and rv.DistValInt(((1, 1), (0, 6))) = 1) then
        m := 0;
      elsif (rv.DistValInt(((1, 1), (0, 3))) = 1) then
        m := rv.RandInt(1, 1500);
      else
        m := rv.RandInt(1, 12);
      end if;
      if (term = T_EOL and m = 0) then
        m := 1;
      end if;
      new_run(l, m, term, bool2bit(rv.DistValInt(((1, 1), (0, 9))) = 1));
    end procedure random_run;

  begin

    SetAlertLogName("tb_a15_a16_nl_osvvm");
    SetLogEnable(PASSED, FALSE);
    rv.InitSeed(rv'instance_name);
    req     := GetReqID("T87.A15", 400);
    req16   := GetReqID("T87.A16", 400);
    reqLane := GetReqID("NL.RunLaneState", 400);

    covTerm := NewID("G4.Run lane x terminal");
    SetFieldName(covTerm, "lane", "terminal");
    covImm  := NewID("G4.Run lane x immediateBreak");
    SetFieldName(covImm, "lane", "immediateBreak");
    covEoi  := NewID("G4.Stream lane x eoiReset");
    SetFieldName(covEoi, "lane", "eoiReset");
    for l in 0 to LANES - 1 loop
      AddCross(covTerm, "lane=" & to_string(l) & " / interruption", GenBin(l), GenBin(T_BREAK));
      AddCross(covTerm, "lane=" & to_string(l) & " / end of line", GenBin(l), GenBin(T_EOL));
      AddCross(covImm, "lane=" & to_string(l) & " / nonempty run", GenBin(l), GenBin(0));
      AddCross(covImm, "lane=" & to_string(l) & " / immediate break", GenBin(l), GenBin(1));
      AddCross(covEoi, "lane=" & to_string(l) & " / within image", GenBin(l), GenBin(0));
      AddCross(covEoi, "lane=" & to_string(l) & " / end of image", GenBin(l), GenBin(1));
    end loop;
    covAcc := NewID("G2.Clamp RUNindex");
    SetFieldName(covAcc, "clamp");
    AddBins(covAcc, "A.15 boundary at RUNindex 31 (holds 31)", GenBin(0));
    AddBins(covAcc, "A.16 break at RUNindex 0 (holds 0)", GenBin(1));
    covIdx := NewID("G4.Run startIdx");
    SetFieldName(covIdx, "startIdx");
    AddBins(covIdx, "startIdx0", GenBin(0, 0));
    AddBins(covIdx, "startIdxLo", GenBin(1, 3, 1));
    AddBins(covIdx, "startIdxMid", GenBin(4, 10, 1));
    AddBins(covIdx, "startIdxHi", GenBin(11, 30, 1));
    AddBins(covIdx, "startIdx31", GenBin(31, 31));

    for l in 0 to LANES - 1 loop
      clear_lane(l);
    end loop;
    sRst <= '0';
    wait until rising_edge(clk);

    -- Slot loop: lane = slot mod 4. Lane 0 plays SCRIPT then random; the
    -- others random. A random slot may be idle (not run mode) or a reset.
    loop

      lane := step mod LANES;
      step := step + 1;

      if (not busy(lane)) then
        if (lane = 0 and runs < SCRIPT'length) then
          new_run(0, SCRIPT(runs).m, SCRIPT(runs).term, SCRIPT(runs).eoi);
          runs := runs + 1;
        else
          random_run(lane);
        end if;
      end if;

      -- Lane state on the ports; arbitrary data on idle slots.
      sRunIndex  <= to_unsigned(stIdx(lane), 5);
      sNextBound <= to_unsigned(stBound(lane), RC_W + 1);
      sInRun     <= stIn(lane);
      sRbPx        <= to_unsigned(rv.RandInt(0, 2 ** BITNESS - 1), BITNESS);
      sRaPx        <= to_unsigned(lane + 10, BITNESS);

      if (rv.DistValInt(((1, 1), (0, 15))) = 1) then
        -- Idle slot (regular pixel of this lane): no tokens, state held.
        sModeIsRun <= '0';
        sRunHit    <= bool2bit(rv.RandInt(0, 1) = 1);
        sRunCont   <= bool2bit(rv.RandInt(0, 1) = 1);
        sRunCnt    <= to_unsigned(rv.RandInt(0, 2 ** RC_W - 1), RC_W);
        sRst       <= bool2bit(rv.DistValInt(((1, 1), (0, 3))) = 1);
        wait for 1 ns;
        AffirmIf(reqLane, oRawValid = '0' and oRiValid = '0', "idle slot: no tokens");
        AffirmIf(reqLane, checked_integer(sRunIndexNext) = stIdx(lane) and
                 checked_integer(sNextBoundNext) = stBound(lane) and oInRunNext = stIn(lane),
                 "idle slot: lane state held");
        wait until rising_edge(clk);
        next;
      end if;

      -- Reset slot: run mode but iRst = '1'. Outputs quiet, state held.
      if (rv.DistValInt(((1, 1), (0, 200))) = 1) then
        sModeIsRun <= '1';
        sRunHit    <= '0';
        sRunCont   <= '0';
        sRunCnt    <= to_unsigned(pos(lane), RC_W);
        sRst       <= '1';
        wait for 1 ns;
        AffirmIf(reqLane, oRawValid = '0' and oRiValid = '0', "iRst: no tokens");
        AffirmIf(reqLane, checked_integer(sRunIndexNext) = stIdx(lane) and
                 checked_integer(sNextBoundNext) = stBound(lane),
                 "iRst: lane state held");
        wait until rising_edge(clk);
        next;
      end if;

      sRst       <= '0';
      sModeIsRun <= '1';
      last       := pos(lane) >= nMatch(lane);
      px         := lane + 10;                                       -- Ix = Ra: match
      if (not last) then
        sRunHit  <= '1';
        sRunCont <= '1';
        sRunCnt  <= to_unsigned(pos(lane) + 1, RC_W);
      elsif (runTerm(lane) = T_BREAK) then
        px       := rv.RandInt(0, 2 ** BITNESS - 1);
        sRunHit  <= '0';
        sRunCont <= '0';
        sRunCnt  <= to_unsigned(runM(lane), RC_W);
      else
        sRunHit  <= '1';
        sRunCont <= '0';
        sRunCnt  <= to_unsigned(runM(lane), RC_W);
      end if;
      sIxPx <= to_unsigned(px, BITNESS);
      wait for 1 ns;

      if (oRawValid = '1') then
        len := actLen(lane);
        app(actBuf(lane), len, checked_integer(oRawSuffixVal), checked_integer(oRawSuffixLen));
        actLen(lane) := len;
      end if;

      -- Latch the lane's next state (cleared after its last pixel).
      stIdx(lane)   := checked_integer(sRunIndexNext);
      stBound(lane) := checked_integer(sNextBoundNext);
      stIn(lane)    := oInRunNext;
      pos(lane)     := pos(lane) + 1;

      if (last) then
        write(msg, "lane " & to_string(lane) & " run M=" & to_string(runM(lane)) &
                   " term=" & to_string(runTerm(lane)) & " from RUNindex " & to_string(carried(lane)));
        AffirmIf(req16, oRiValid = bool2bit(runTerm(lane) = T_BREAK), msg.all & ": RI-valid");
        if (runTerm(lane) = T_BREAK) then
          AffirmIfEqual(req16, checked_integer(oRiIx), px, msg.all & ": RI Ix");
          AffirmIfEqual(req16, checked_integer(oRiRa), lane + 10, msg.all & ": RI Ra");
          AffirmIfEqual(req16, checked_integer(oRiRb), to_integer(sRbPx), msg.all & ": RI Rb");
          AffirmIfEqual(req16, checked_integer(oRiRunIndex), termIdx(lane), msg.all & ": RI index before decrement");
        end if;
        -- A.15 bits are the prefix, A.16 the tail.
        n15 := expN15(lane);
        if (actLen(lane) >= n15) then
          AffirmIfEqual(req, actBuf(lane)(1 to n15), expBuf(lane)(1 to n15), msg.all & ": A.15 bits");
        else
          AffirmIfEqual(req, actLen(lane), n15, msg.all & ": A.15 bit-length");
        end if;
        AffirmIfEqual(req16, actLen(lane), expLen(lane), msg.all & ": bit-length");
        if (actLen(lane) = expLen(lane) and expLen(lane) > n15) then
          AffirmIfEqual(req16, actBuf(lane)(n15 + 1 to actLen(lane)), expBuf(lane)(n15 + 1 to expLen(lane)),
                        msg.all & ": A.16 bits");
        end if;
        AffirmIfEqual(reqLane, stIdx(lane), newIdx(lane), msg.all & ": lane RUNindex after run");
        deallocate(msg);

        ICover(covTerm, (lane, runTerm(lane)));
        ICover(covImm, (lane, boolean'pos(runM(lane) = 0)));
        ICover(covEoi, (lane, std_to_int(runEoi(lane))));
        ICover(covIdx, carried(lane));
        if (expSat(lane)) then
          ICover(covAcc, 0);
        end if;
        if (runTerm(lane) = T_BREAK and termIdx(lane) = 0) then
          ICover(covAcc, 1);
        end if;

        if (runEoi(lane) = '1') then
          clear_lane(lane);
          carried(lane) := 0;
        else
          carried(lane) := newIdx(lane);
        end if;
        busy(lane) := false;
      end if;

      wait until rising_edge(clk);

      exit when runs = SCRIPT'length and not busy(0) and IsCovered(covTerm) and IsCovered(covImm) and
                IsCovered(covEoi) and IsCovered(covIdx) and IsCovered(covAcc) and step > 40000;
      exit when step > 4_000_000;

    end loop;

    WriteBin(covTerm);
    WriteBin(covImm);
    WriteBin(covEoi);
    WriteBin(covIdx);
    WriteBin(covAcc);
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covAcc), "RUNindex clamp coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covTerm), "lane x terminal coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covImm), "lane x immediate-break coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covEoi), "lane x EOI-reset coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covIdx), "start-RUNindex coverage closed");

    end_of_test("tb_a15_a16_nl_osvvm");
    wait;

  end process stim;

end architecture sim;
