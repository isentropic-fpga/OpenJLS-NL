-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- byte_stuffer_nl: byte_stuffer shared by 4 lanes, one per clock (lane = iSlot); changed lines marked NL
--
-- Every register is kept 4 times in a ring X_r(0..3) that rotates each clock. Slot 0 is the lane
-- served this cycle: the logic reads X_r(0) and writes X_r(3). Both FIFOs become one FIFO per lane.
--
-- Stage 1 packs bits into fixed FIFO_BITS words, stage 2 is a BRAM FIFO, and stage 3 stuffs up to
-- 4 bytes per cycle by decoding all eight stuffing layouts in parallel.
-- iFlush arrives with the image's last word; oFlushDone pulses with oWordValid on the final beat.

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use work.openjls_pkg.all;
  use work.nl_pkg.all;

library work;
  use work.olo_base_pkg_math.log2ceil;

entity byte_stuffer_nl is
  generic (
    IN_WIDTH            : natural := CO_LIMIT_STD;
    OUT_BYTES_PER_CYCLE : natural := CO_BYTE_STUFFER_OUT_BYTES_PER_CYCLE;
    OUT_WIDTH           : natural := CO_BYTE_STUFFER_OUT_WIDTH;
    BURST_DEPTH         : natural := CO_BYTE_STUFFER_BURST_DEPTH
  );
  port (
    iClk                : in    std_logic;
    iRst                : in    std_logic;
    iSlot               : in    unsigned(1 downto 0);                     -- NL: lane served this cycle
    iStall              : in    std_logic;                                -- No ready output: accepts every beat unless stalled
    iWord               : in    std_logic_vector(IN_WIDTH - 1 downto 0);
    iWordValid          : in    std_logic;
    iWordValidLen       : in    unsigned(log2ceil(IN_WIDTH + 1) - 1 downto 0);
    iFlush              : in    std_logic;
    oWord               : out   std_logic_vector(OUT_WIDTH - 1 downto 0);
    oWordValid          : out   std_logic;
    oValidBytes         : out   unsigned(log2ceil(OUT_BYTES_PER_CYCLE + 1) - 1 downto 0);
    iReady              : in    std_logic;
    oAlmostFull         : out   std_logic_vector(3 downto 0);             -- NL: per lane
    oFlushDone          : out   std_logic
  );
end entity byte_stuffer_nl;

architecture behavioral of byte_stuffer_nl is


  constant FIFO_BYTES             : natural := math_ceil_div(IN_WIDTH, 8);
  constant FIFO_BITS              : natural := FIFO_BYTES * 8;
  constant ACCUM_BITS             : natural := 2 * FIFO_BITS;
  -- Final word's real-bit count; excludes stage 1's byte pad, which is never emitted
  constant LAST_BITS_WIDTH        : natural := log2ceil(FIFO_BITS + 1);

  -- FIFO entry: data & last_flag
  constant LAST_POS               : natural := 0;
  constant DATA_LSB               : natural := 1;
  constant FIFO_WIDTH             : natural := FIFO_BITS + 1;

  -- Max last-flag words in flight in the main FIFO
  constant BYTE_VALID_QUEUE_DEPTH : natural := 3;

  -- Absorbs the words in flight during the ~4-cycle top-level stall latency
  constant STALL_CUSHION_ENTRIES  : natural := 5;
  constant ALM_FULL_LEVEL         : natural := BURST_DEPTH - STALL_CUSHION_ENTRIES;

  -- One FIFO word + 1 byte (deadlock floor), MSB-first
  constant HOLD_BYTES             : natural := FIFO_BYTES + 1;
  constant HOLD_BITS              : natural := HOLD_BYTES * 8;

  -- A stuffed byte starts with '0', so adjacent bytes cannot both stuff: these are
  -- all eight 4-byte layouts. '1' = the lane takes 7 bits.
  type layout_array is array (0 to 7) of std_logic_vector(0 to 3);
  constant STUFF_LAYOUT : layout_array := (
    "0000", "0001", "0010", "0100", "0101", "1000", "1001", "1010"
  );
  type lane_count_array is array (0 to 7, 0 to 3) of natural range 0 to 32;
  constant LANE_START : lane_count_array := (
    (0, 8, 16, 24), (0, 8, 16, 24), (0, 8, 16, 23),
    (0, 8, 15, 23), (0, 8, 15, 23), (0, 7, 15, 23),
    (0, 7, 15, 23), (0, 7, 15, 22)
  );
  constant LANE_END : lane_count_array := (
    (8, 16, 24, 32), (8, 16, 24, 31), (8, 16, 23, 31),
    (8, 15, 23, 31), (8, 15, 23, 30), (7, 15, 23, 31),
    (7, 15, 23, 30), (7, 15, 22, 30)
  );
  type consume_array is array (natural range <>) of natural range 0 to 32;
  constant CONSUME_BITS : consume_array := (0, 7, 8, 15, 16, 22, 23, 24, 30, 31, 32);

  signal sWord                    : std_logic_vector(IN_WIDTH - 1 downto 0);
  signal sWord_r : t_slv_array(0 to 3)(IN_WIDTH - 1 downto 0);  -- NL
  signal sWordValidLen            : unsigned(log2ceil(IN_WIDTH + 1) - 1 downto 0);          -- [0, IN_WIDTH]
  signal sWordValidLen_r : t_unsigned_array(0 to 3)(log2ceil(IN_WIDTH + 1) - 1 downto 0);  -- NL
  signal sWordValid               : std_logic;
  signal sWordValid_r : std_logic_vector(0 to 3);  -- NL
  signal sFlush                   : std_logic;
  signal sFlush_r : std_logic_vector(0 to 3);  -- NL

  signal sAccumBuffer             : std_logic_vector(ACCUM_BITS - 1 downto 0);
  signal sAccumBuffer_r : t_slv_array(0 to 3)(ACCUM_BITS - 1 downto 0);  -- NL
  signal sAccumCountBits          : unsigned(log2ceil(ACCUM_BITS + 1) - 1 downto 0);        -- [0, ACCUM_BITS]
  signal sAccumCountBits_r : t_unsigned_array(0 to 3)(log2ceil(ACCUM_BITS + 1) - 1 downto 0);  -- NL
  signal sAccumCountBitsFlush     : unsigned(log2ceil(ACCUM_BITS + 1) - 1 downto 0);        -- [0, ACCUM_BITS]
  signal sAccumCountBitsFlush_r : t_unsigned_array(0 to 3)(log2ceil(ACCUM_BITS + 1) - 1 downto 0);  -- NL
  signal sFlushValidBits          : unsigned(LAST_BITS_WIDTH - 1 downto 0);                 -- [0, FIFO_BITS]
  signal sFlushValidBits_r : t_unsigned_array(0 to 3)(LAST_BITS_WIDTH - 1 downto 0);  -- NL
  signal sFlushPending            : std_logic;
  signal sFlushPending_r : std_logic_vector(0 to 3);  -- NL

  signal sFifoInData              : std_logic_vector(FIFO_WIDTH - 1 downto 0);
  signal sFifoInData_r : t_slv_array(0 to 3)(FIFO_WIDTH - 1 downto 0);  -- NL
  signal sFifoInValid             : std_logic;
  signal sFifoInValid_r : std_logic_vector(0 to 3);  -- NL
  signal sFifoInReady             : std_logic;
  signal sFifoOutData             : std_logic_vector(FIFO_WIDTH - 1 downto 0);
  signal sFifoOutValid            : std_logic;
  signal sFifoOutReady            : std_logic;
  signal sFifoAlmFull             : std_logic;
  signal sFifoFull                : std_logic;
  -- NL: per-lane FIFO ports
  signal sLaneSel                 : std_logic_vector(3 downto 0);
  signal sFifoOutDataL            : t_slv_array(0 to 3)(FIFO_WIDTH - 1 downto 0);
  signal sFifoInReadyL            : std_logic_vector(3 downto 0);
  signal sFifoOutValidL           : std_logic_vector(3 downto 0);
  signal sFifoFullL               : std_logic_vector(3 downto 0);
  signal sFifoAlmFullL            : std_logic_vector(3 downto 0);
  signal sBvQueueOutDataL         : t_slv_array(0 to 3)(LAST_BITS_WIDTH - 1 downto 0);

  -- FIFO-to-stage-3 skid: breaks the BRAM output route
  signal sSkidWord                : std_logic_vector(FIFO_WIDTH - 1 downto 0);
  signal sSkidWord_r : t_slv_array(0 to 3)(FIFO_WIDTH - 1 downto 0);  -- NL
  signal sSkidData                : std_logic_vector(FIFO_BITS - 1 downto 0);
  signal sSkidValid               : std_logic;
  signal sSkidValid_r : std_logic_vector(0 to 3);  -- NL
  signal sSkidTaken               : std_logic;
  signal sSkidLast                : std_logic;

  -- Final-word real-bit-count queue
  signal sBvQueueInValid          : std_logic;
  signal sBvQueueInValid_r : std_logic_vector(0 to 3);  -- NL
  signal sBvQueueInData           : std_logic_vector(LAST_BITS_WIDTH - 1 downto 0);         -- [0, FIFO_BITS]
  signal sBvQueueInData_r : t_slv_array(0 to 3)(LAST_BITS_WIDTH - 1 downto 0);  -- NL
  signal sBvQueueOutReady         : std_logic;
  signal sBvQueueOutData          : std_logic_vector(LAST_BITS_WIDTH - 1 downto 0);         -- [0, FIFO_BITS]

  signal sStuffBuffer             : std_logic_vector(HOLD_BITS - 1 downto 0);
  signal sStuffBuffer_r : t_slv_array(0 to 3)(HOLD_BITS - 1 downto 0);  -- NL
  signal sStuffBufferBits         : unsigned(log2ceil(HOLD_BITS + 1) - 1 downto 0);         -- [0, HOLD_BITS]
  signal sStuffBufferBits_r : t_unsigned_array(0 to 3)(log2ceil(HOLD_BITS + 1) - 1 downto 0);  -- NL
  signal sStuffBufferLast         : std_logic;
  signal sStuffBufferLast_r : std_logic_vector(0 to 3);  -- NL
  signal sPrevFF                  : std_logic;
  signal sPrevFF_r : std_logic_vector(0 to 3);  -- NL
  signal sOutWordReg              : std_logic_vector(OUT_WIDTH - 1 downto 0);
  signal sOutWordReg_r : t_slv_array(0 to 3)(OUT_WIDTH - 1 downto 0);  -- NL
  signal sOutValidReg             : std_logic;
  signal sOutValidReg_r : std_logic_vector(0 to 3);  -- NL
  signal sOutBytesValidReg        : unsigned(log2ceil(OUT_BYTES_PER_CYCLE + 1) - 1 downto 0); -- [0, OUT_BYTES_PER_CYCLE]
  signal sOutBytesValidReg_r : t_unsigned_array(0 to 3)(log2ceil(OUT_BYTES_PER_CYCLE + 1) - 1 downto 0);  -- NL
  signal sFlushDone               : std_logic;
  signal sFlushDone_r : std_logic_vector(0 to 3);  -- NL

  signal sLastPending             : std_logic;
  signal sLastPending_r : std_logic_vector(0 to 3);  -- NL

begin

  -- NL: lane served this cycle = ring slot 0
  sWord <= sWord_r(0);
  sWordValidLen <= sWordValidLen_r(0);
  sWordValid <= sWordValid_r(0);
  sFlush <= sFlush_r(0);
  sAccumBuffer <= sAccumBuffer_r(0);
  sAccumCountBits <= sAccumCountBits_r(0);
  sAccumCountBitsFlush <= sAccumCountBitsFlush_r(0);
  sFlushValidBits <= sFlushValidBits_r(0);
  sFlushPending <= sFlushPending_r(0);
  sFifoInValid <= sFifoInValid_r(0);
  sFifoInData <= sFifoInData_r(0);
  sBvQueueInValid <= sBvQueueInValid_r(0);
  sBvQueueInData <= sBvQueueInData_r(0);
  sSkidValid <= sSkidValid_r(0);
  sSkidWord <= sSkidWord_r(0);
  sStuffBuffer <= sStuffBuffer_r(0);
  sStuffBufferBits <= sStuffBufferBits_r(0);
  sStuffBufferLast <= sStuffBufferLast_r(0);
  sPrevFF <= sPrevFF_r(0);
  sOutWordReg <= sOutWordReg_r(0);
  sOutValidReg <= sOutValidReg_r(0);
  sOutBytesValidReg <= sOutBytesValidReg_r(0);
  sFlushDone <= sFlushDone_r(0);
  sLastPending <= sLastPending_r(0);

  assert OUT_BYTES_PER_CYCLE = 4
    report "byte_stuffer: OUT_BYTES_PER_CYCLE must be 4 (stuffing arrays are hardcoded to 4 lanes)"
    severity failure;

  assert BURST_DEPTH > STALL_CUSHION_ENTRIES
    report "byte_stuffer: BURST_DEPTH must exceed STALL_CUSHION_ENTRIES"
    severity failure;

  assert not (sFifoInValid = '1' and sFifoInReady = '0')
    report "byte_stuffer: FIFO write dropped - AlmFull cushion undersized vs stall latency"
    severity failure;

  -- Contracts (PSL, simulation only)
  -- psl default clock is rising_edge(iClk);
  -- psl assert always (iRst = '1' -> next (oWordValid = '0' and oFlushDone = '0')) report "byte_stuffer: reset must clear the output beat and oFlushDone";
  -- psl assert never (oFlushDone = '1' and oWordValid = '0') report "byte_stuffer: oFlushDone only fires on a valid output beat (framer iEoi contract)";
  -- psl assert always (oFlushDone = '1' -> next[4] (oFlushDone = '0')) report "byte_stuffer: oFlushDone is a strict 1-cycle pulse"; -- NL: per lane
  -- psl assert always (oWordValid = '1' -> oValidBytes <= OUT_BYTES_PER_CYCLE) report "byte_stuffer: oValidBytes exceeds the per-cycle output cap";

  -- NL: iSlot must count 0, 1, 2, 3 every clock out of reset, else lane state is crossed
  slot_assert_proc : process (iClk) is
    variable vPrevSlot : unsigned(1 downto 0) := (others => '0');
    variable vPrevRun  : boolean := false;
  begin
    if rising_edge(iClk) then
      assert not vPrevRun or iRst = '1' or Is_X(iSlot) or iSlot = vPrevSlot + 1
        report "byte_stuffer_nl: iSlot must advance by one lane per clock"
        severity failure;
      vPrevSlot := iSlot;
      vPrevRun  := iRst = '0';
    end if;
  end process slot_assert_proc;

  oWord       <= sOutWordReg;
  oWordValid  <= sOutValidReg;
  oValidBytes <= sOutBytesValidReg;
  oFlushDone  <= sFlushDone;
  oAlmostFull <= sFifoAlmFullL;  -- NL

  -- Gated by iStall only: bit_packer holds across a stall and the accumulator cannot backpressure

  input_reg_proc : process (iClk) is
  begin

    if rising_edge(iClk) then
      -- NL: rotate lane rings
      sWord_r <= sWord_r(1 to 3) & sWord_r(0);
      sWordValidLen_r <= sWordValidLen_r(1 to 3) & sWordValidLen_r(0);
      sWordValid_r <= sWordValid_r(1 to 3) & sWordValid_r(0);
      sFlush_r <= sFlush_r(1 to 3) & sFlush_r(0);
      if (iRst = '1') then
        sWord_r <= (others => (others => '0'));
        sWordValidLen_r <= (others => (others => '0'));
        sWordValid_r <= (others => '0');
        sFlush_r <= (others => '0');
      elsif (iStall = '0') then
        sWord_r(3) <= iWord;
        sWordValidLen_r(3) <= iWordValidLen;
        sWordValid_r(3) <= iWordValid;
        sFlush_r(3) <= iFlush;
      end if;
    end if;

  end process input_reg_proc;

  stage1_proc : process (iClk) is

    variable vAccumBuffer         : std_logic_vector(ACCUM_BITS - 1 downto 0);
    variable vAccumCountBits      : natural range 0 to ACCUM_BITS;
    variable vAccumCountBitsFlush : natural range 0 to ACCUM_BITS;
    variable vFlushValidBits      : natural range 0 to FIFO_BITS;
    variable vFlushRawBits        : natural range 0 to ACCUM_BITS;
    variable vValidLenInt         : natural;                                                -- [0, IN_WIDTH]
    variable vFlushPending        : std_logic;
    variable vPadBits             : natural;                                                -- [1, FIFO_BITS]
    variable vLastFlag            : std_logic;
    variable vWide                : std_logic_vector(ACCUM_BITS - 1 downto 0);
    variable vMaskTop             : std_logic_vector(ACCUM_BITS - 1 downto 0);
    variable vShifted             : std_logic_vector(ACCUM_BITS - 1 downto 0);
    variable vMask                : std_logic_vector(ACCUM_BITS - 1 downto 0);

  begin

    if rising_edge(iClk) then
      -- NL: rotate lane rings
      sAccumBuffer_r <= sAccumBuffer_r(1 to 3) & sAccumBuffer_r(0);
      sAccumCountBits_r <= sAccumCountBits_r(1 to 3) & sAccumCountBits_r(0);
      sAccumCountBitsFlush_r <= sAccumCountBitsFlush_r(1 to 3) & sAccumCountBitsFlush_r(0);
      sFlushValidBits_r <= sFlushValidBits_r(1 to 3) & sFlushValidBits_r(0);
      sFlushPending_r <= sFlushPending_r(1 to 3) & sFlushPending_r(0);
      sFifoInValid_r <= sFifoInValid_r(1 to 3) & sFifoInValid_r(0);
      sFifoInData_r <= sFifoInData_r(1 to 3) & sFifoInData_r(0);
      sBvQueueInValid_r <= sBvQueueInValid_r(1 to 3) & sBvQueueInValid_r(0);
      sBvQueueInData_r <= sBvQueueInData_r(1 to 3) & sBvQueueInData_r(0);
      if (iRst = '1') then
        sAccumBuffer_r <= (others => (others => '0'));
        sAccumCountBits_r <= (others => (others => '0'));
        sAccumCountBitsFlush_r <= (others => (others => '0'));
        sFlushValidBits_r <= (others => (others => '0'));
        sFlushPending_r <= (others => '0');
        sFifoInValid_r <= (others => '0');
        sFifoInData_r <= (others => (others => '0'));
        sBvQueueInValid_r <= (others => '0');
        sBvQueueInData_r <= (others => (others => '0'));
      else
        vAccumBuffer         := sAccumBuffer;
        vAccumCountBits      := to_integer(sAccumCountBits);
        vAccumCountBitsFlush := to_integer(sAccumCountBitsFlush);
        vFlushValidBits      := to_integer(sFlushValidBits);
        vValidLenInt         := to_integer(sWordValidLen);
        vFlushPending        := sFlushPending;

        sFifoInValid_r(3) <= '0';
        sBvQueueInValid_r(3) <= '0';


        if (sWordValid = '1' and iStall = '0') then
          vWide                                              := (others => '0');
          vWide(ACCUM_BITS - 1 downto ACCUM_BITS - IN_WIDTH) := sWord;
          vMaskTop                                           := (others => '0');

          for i in 0 to IN_WIDTH - 1 loop

            if (i < vValidLenInt) then
              vMaskTop(ACCUM_BITS - 1 - i) := '1';
            end if;

          end loop;

          vShifted        := std_logic_vector(shift_right(unsigned(vWide), vAccumCountBits));
          vMask           := std_logic_vector(shift_right(unsigned(vMaskTop), vAccumCountBits));
          vAccumBuffer    := (vAccumBuffer and not vMask) or (vShifted and vMask);
          vAccumCountBits := vAccumCountBits + vValidLenInt;
        end if;

        -- Flush: pad to a byte, then to a FIFO_BITS multiple so every drain is a fixed shift
        if (sFlush = '1' and iStall = '0') then
          assert vFlushPending = '0'
            report "byte_stuffer: iFlush asserted while a flush is already pending"
            severity failure;

          vFlushRawBits := vAccumCountBits;

          if ((vAccumCountBits mod 8) /= 0) then
            vPadBits := 8 - (vAccumCountBits mod 8);

            for j in 0 to 7 loop

              if (j < vPadBits) then
                vAccumBuffer(ACCUM_BITS - 1 - vAccumCountBits) := '0';
                vAccumCountBits                                := vAccumCountBits + 1;
              end if;

            end loop;

          end if;

          -- Count-only pad: no bits written
          if ((vAccumCountBits mod FIFO_BITS) /= 0) then
            vPadBits        := FIFO_BITS - (vAccumCountBits mod FIFO_BITS);
            vAccumCountBits := vAccumCountBits + vPadBits;
          end if;

          vAccumCountBitsFlush := vAccumCountBits;

          -- Real bits in the last FIFO_BITS slice, in (0, FIFO_BITS]
          vFlushValidBits := vFlushRawBits + FIFO_BITS - vAccumCountBitsFlush;

          vFlushPending := '1';
        end if;

        assert not (sFifoFull = '1' and (vFlushPending = '1' or vAccumCountBits >= FIFO_BITS))
          report "byte_stuffer: FIFO full but accumulator didn't stall"
          severity failure;

        if (sFifoFull = '0') then
          if (vFlushPending = '1') then
            if (vAccumCountBitsFlush = FIFO_BITS) then
              vLastFlag       := '1';
              sBvQueueInValid_r(3) <= '1';
              sBvQueueInData_r(3) <= std_logic_vector(to_unsigned(vFlushValidBits, LAST_BITS_WIDTH));
              vFlushPending   := '0';
            else
              vLastFlag := '0';
            end if;

            sFifoInData_r(3) <= vAccumBuffer(ACCUM_BITS - 1 downto ACCUM_BITS - FIFO_BITS) & vLastFlag;
            sFifoInValid_r(3) <= '1';

            vAccumBuffer         := std_logic_vector(shift_left(unsigned(vAccumBuffer), FIFO_BITS));
            vAccumCountBits      := vAccumCountBits - FIFO_BITS;
            vAccumCountBitsFlush := vAccumCountBitsFlush - FIFO_BITS;
          elsif (vAccumCountBits >= FIFO_BITS) then
            sFifoInData_r(3) <= vAccumBuffer(ACCUM_BITS - 1 downto ACCUM_BITS - FIFO_BITS) & '0';
            sFifoInValid_r(3) <= '1';

            vAccumBuffer    := std_logic_vector(shift_left(unsigned(vAccumBuffer), FIFO_BITS));
            vAccumCountBits := vAccumCountBits - FIFO_BITS;
          end if;
        end if;

        sAccumBuffer_r(3) <= vAccumBuffer;
        sAccumCountBits_r(3) <= to_unsigned(vAccumCountBits, sAccumCountBits'length);
        sAccumCountBitsFlush_r(3) <= to_unsigned(vAccumCountBitsFlush, sAccumCountBitsFlush'length);
        sFlushValidBits_r(3) <= to_unsigned(vFlushValidBits, sFlushValidBits'length);
        sFlushPending_r(3) <= vFlushPending;

        assert vAccumCountBits <= ACCUM_BITS
          report "byte_stuffer: stage 1 accumulator overflow"
          severity failure;
      end if;
    end if;

  end process stage1_proc;

  -- NL: one FIFO pair per lane, touched only on that lane's turn
  gen_lane_fifo : for l in 0 to 3 generate

    sLaneSel(l) <= '1' when iSlot = l else
                   '0';

    fifo_inst : entity work.olo_base_fifo_sync(rtl)
      generic map (
        WIDTH_G        => FIFO_WIDTH,
        DEPTH_G        => BURST_DEPTH,
        ALMFULLON_G    => true,
        ALMFULLLEVEL_G => ALM_FULL_LEVEL,
        RAMSTYLE_G     => "auto",
        RAMBEHAVIOR_G  => "RBW"
      )
      port map (
        Clk            => iClk,
        Rst            => iRst,
        In_Data        => sFifoInData,
        In_Valid       => sFifoInValid and sLaneSel(l),
        In_Ready       => sFifoInReadyL(l),
        Out_Data       => sFifoOutDataL(l),
        Out_Valid      => sFifoOutValidL(l),
        Out_Ready      => sFifoOutReady and sLaneSel(l),
        Full           => sFifoFullL(l),
        AlmFull        => sFifoAlmFullL(l),
        Empty          => open,
        AlmEmpty       => open
      );

    byte_valid_fifo_inst : entity work.olo_base_fifo_sync(rtl)
      generic map (
        WIDTH_G       => LAST_BITS_WIDTH,
        DEPTH_G       => BYTE_VALID_QUEUE_DEPTH,
        RAMSTYLE_G    => "auto",
        RAMBEHAVIOR_G => "RBW"
      )
      port map (
        Clk           => iClk,
        Rst           => iRst,
        In_Data       => sBvQueueInData,
        In_Valid      => sBvQueueInValid and sLaneSel(l),
        In_Ready      => open,
        Out_Data      => sBvQueueOutDataL(l),
        Out_Valid     => open,
        Out_Ready     => sBvQueueOutReady and sLaneSel(l),
        Full          => open,
        Empty         => open
      );

  end generate gen_lane_fifo;

  sFifoInReady     <= sFifoInReadyL(to_integer(iSlot));
  sFifoOutData     <= sFifoOutDataL(to_integer(iSlot));
  sFifoOutValid    <= sFifoOutValidL(to_integer(iSlot));
  sFifoFull        <= sFifoFullL(to_integer(iSlot));
  sFifoAlmFull     <= sFifoAlmFullL(to_integer(iSlot));
  sBvQueueOutData  <= sBvQueueOutDataL(to_integer(iSlot));

  sBvQueueOutReady <= sSkidTaken and sSkidLast;


  sSkidTaken <= '1' when sSkidValid = '1'
                         and sStuffBufferBits <= to_unsigned(HOLD_BITS - FIFO_BITS, sStuffBufferBits'length)
                         and iReady = '1'
                         and sLastPending = '0' else
                '0';
  sFifoOutReady <= '1' when sSkidValid = '0' or sSkidTaken = '1' else
                   '0';

  sSkidData <= sSkidWord(FIFO_WIDTH - 1 downto DATA_LSB);
  sSkidLast <= sSkidWord(LAST_POS);

  skid_proc : process (iClk) is
  begin

    if rising_edge(iClk) then
      -- NL: rotate lane rings
      sSkidValid_r <= sSkidValid_r(1 to 3) & sSkidValid_r(0);
      sSkidWord_r <= sSkidWord_r(1 to 3) & sSkidWord_r(0);
      if (iRst = '1') then
        sSkidValid_r <= (others => '0');
        sSkidWord_r <= (others => (others => '0'));
      else
        if (sSkidTaken = '1') then
          sSkidValid_r(3) <= '0';
        end if;
        if (sFifoOutValid = '1' and sFifoOutReady = '1') then
          sSkidWord_r(3) <= sFifoOutData;
          sSkidValid_r(3) <= '1';
        end if;
      end if;
    end if;

  end process skid_proc;

  stage3_proc : process (iClk) is

    variable vStuffBuffer     : std_logic_vector(HOLD_BITS - 1 downto 0);
    variable vStuffBufferBits : natural range 0 to HOLD_BITS;
    variable vStuffBufferLast : std_logic;
    variable vPrevFF          : std_logic;

    variable vValidBitsInt  : natural range 0 to FIFO_BITS;
    variable vRefillBuffer  : std_logic_vector(HOLD_BITS - 1 downto 0);
    variable vRefillWord    : std_logic_vector(HOLD_BITS - 1 downto 0);
    variable vRefillMask    : std_logic_vector(HOLD_BITS - 1 downto 0);

    variable ff0        : std_logic; -- offset 0
    variable ff1a, ff1b : std_logic; -- offsets 7, 8
    variable ff2a, ff2b : std_logic; -- offsets 15, 16

    variable vPath          : std_logic_vector(0 to 7);
    variable vTakeShift     : std_logic_vector(CONSUME_BITS'range);
    variable vPathMask      : std_logic_vector(OUT_WIDTH - 1 downto 0);
    variable vCandidateWord : std_logic_vector(OUT_WIDTH - 1 downto 0);
    variable vCandidateByte : std_logic_vector(7 downto 0);
    variable vLaneSelected  : std_logic;
    variable vBytesMask     : unsigned(sOutBytesValidReg'range);
    variable vEmitBytesBits : unsigned(sOutBytesValidReg'range);
    variable vBufferMask    : std_logic_vector(HOLD_BITS - 1 downto 0);
    variable vNextBuffer    : std_logic_vector(HOLD_BITS - 1 downto 0);
    variable vCountMask     : unsigned(sStuffBufferBits'range);
    variable vNextCount     : unsigned(sStuffBufferBits'range);

    variable vEmitData   : std_logic_vector(OUT_WIDTH - 1 downto 0);
    variable vEmitBytes  : natural range 0 to OUT_BYTES_PER_CYCLE;
    variable vEmitLastFF : std_logic;
    variable vPadByte    : std_logic_vector(7 downto 0);

  begin

    if rising_edge(iClk) then
      -- NL: rotate lane rings
      sStuffBuffer_r <= sStuffBuffer_r(1 to 3) & sStuffBuffer_r(0);
      sStuffBufferBits_r <= sStuffBufferBits_r(1 to 3) & sStuffBufferBits_r(0);
      sStuffBufferLast_r <= sStuffBufferLast_r(1 to 3) & sStuffBufferLast_r(0);
      sPrevFF_r <= sPrevFF_r(1 to 3) & sPrevFF_r(0);
      sOutWordReg_r <= sOutWordReg_r(1 to 3) & sOutWordReg_r(0);
      sOutValidReg_r <= sOutValidReg_r(1 to 3) & sOutValidReg_r(0);
      sOutBytesValidReg_r <= sOutBytesValidReg_r(1 to 3) & sOutBytesValidReg_r(0);
      sFlushDone_r <= sFlushDone_r(1 to 3) & sFlushDone_r(0);
      sLastPending_r <= sLastPending_r(1 to 3) & sLastPending_r(0);
      if (iRst = '1') then
        sStuffBuffer_r <= (others => (others => '0'));
        sStuffBufferBits_r <= (others => (others => '0'));
        sStuffBufferLast_r <= (others => '0');
        sPrevFF_r <= (others => '0');
        sOutWordReg_r <= (others => (others => '0'));
        sOutValidReg_r <= (others => '0');
        sOutBytesValidReg_r <= (others => (others => '0'));
        sFlushDone_r <= (others => '0');
        sLastPending_r <= (others => '0');
      elsif (sLastPending = '1') then
        -- EOI beat: one padded byte for a residue or dangling 0xFF, else a 0-byte beat

        if (iReady = '1') then
          vStuffBufferBits := to_integer(sStuffBufferBits);
          vPadByte         := (others => '0');

          if (sPrevFF = '1') then
            -- Stuff bit, then up to 7 residue bits
            if (vStuffBufferBits > 0) then
              vPadByte(6 downto 7 - vStuffBufferBits) := sStuffBuffer(HOLD_BITS - 1 downto HOLD_BITS - vStuffBufferBits);
            end if;
          elsif (vStuffBufferBits > 0) then
            vPadByte(7 downto 8 - vStuffBufferBits) := sStuffBuffer(HOLD_BITS - 1 downto HOLD_BITS - vStuffBufferBits);
          end if;

          if (vStuffBufferBits = 0 and sPrevFF = '0') then
            sOutWordReg_r(3) <= (others => '0');
            sOutBytesValidReg_r(3) <= (others => '0');
          else
            sOutWordReg_r(3)(OUT_WIDTH - 1 downto OUT_WIDTH - 8) <= vPadByte;
            sOutWordReg_r(3)(OUT_WIDTH - 9 downto 0) <= (others => '0');
            sOutBytesValidReg_r(3) <= to_unsigned(1, sOutBytesValidReg'length);
          end if;

          sOutValidReg_r(3) <= '1';
          sFlushDone_r(3) <= '1';
          sLastPending_r(3) <= '0';
          sStuffBufferLast_r(3) <= '0';
          sPrevFF_r(3) <= '0';
          sStuffBuffer_r(3) <= (others => '0');
          sStuffBufferBits_r(3) <= (others => '0');
        else
          sOutValidReg_r(3) <= '0';
          sFlushDone_r(3) <= '0';
        end if;
      else
        vStuffBuffer     := sStuffBuffer;
        vStuffBufferBits := to_integer(sStuffBufferBits);
        vStuffBufferLast := sStuffBufferLast;
        vPrevFF          := sPrevFF;
        vEmitBytes       := 0;
        vEmitData        := (others => '0');
        sFlushDone_r(3) <= '0';

        -- (1) Refill from the skid; only the last word may be partial
        if (sSkidTaken = '1') then
          -- At most 8 old bits: the 9 alignments are fixed wiring. Stale trailing bits
          -- are copied but never emitted, and the next refill overwrites them.
          vRefillBuffer := (others => '0');
          for offset in 0 to HOLD_BITS - FIFO_BITS loop

            vRefillWord := (others => '0');
            if (offset > 0) then
              vRefillWord(HOLD_BITS - 1 downto HOLD_BITS - offset) :=
                sStuffBuffer(HOLD_BITS - 1 downto HOLD_BITS - offset);
            end if;
            vRefillWord(HOLD_BITS - 1 - offset downto HOLD_BITS - offset - FIFO_BITS) := sSkidData;
            vRefillMask := (others => bool2bit(vStuffBufferBits = offset));
            vRefillBuffer := vRefillBuffer or (vRefillWord and vRefillMask);

          end loop;
          vStuffBuffer := vRefillBuffer;
          if (sSkidLast = '0') then
            vValidBitsInt := FIFO_BITS;
          else
            vValidBitsInt := to_integer(unsigned(sBvQueueOutData));
            vStuffBufferLast := '1';
          end if;
          vStuffBufferBits := vStuffBufferBits + vValidBitsInt;
        end if;

        -- (2) FF flags at fixed windows
        ff0  := bool2bit(vStuffBuffer(HOLD_BITS - 1 downto HOLD_BITS - 8) = x"FF");
        ff1a := bool2bit(vStuffBuffer(HOLD_BITS - 8 downto HOLD_BITS - 15) = x"FF");
        ff1b := bool2bit(vStuffBuffer(HOLD_BITS - 9 downto HOLD_BITS - 16) = x"FF");
        ff2a := bool2bit(vStuffBuffer(HOLD_BITS - 16 downto HOLD_BITS - 23) = x"FF");
        ff2b := bool2bit(vStuffBuffer(HOLD_BITS - 17 downto HOLD_BITS - 24) = x"FF");

        -- (3) Decode all layouts in parallel from sPrevFF and the fixed windows only
        vPath(0) := not vPrevFF and not ff0 and not ff1b and not ff2b;
        vPath(1) := not vPrevFF and not ff0 and not ff1b and     ff2b;
        vPath(2) := not vPrevFF and not ff0 and     ff1b;
        vPath(3) := not vPrevFF and     ff0 and not ff2a;
        vPath(4) := not vPrevFF and     ff0 and     ff2a;
        vPath(5) :=     vPrevFF and not ff1a and not ff2a;
        vPath(6) :=     vPrevFF and not ff1a and     ff2a;
        vPath(7) :=     vPrevFF and     ff1a;

        vEmitData      := (others => '0');
        vEmitBytesBits := (others => '0');
        vEmitLastFF    := '0';
        vTakeShift     := (others => '0');

        for p in STUFF_LAYOUT'range loop

          vCandidateWord := (others => '0');
          vPathMask      := (others => vPath(p));

          for lane in 0 to OUT_BYTES_PER_CYCLE - 1 loop

            if (STUFF_LAYOUT(p)(lane) = '1') then
              vCandidateByte := '0' & vStuffBuffer(HOLD_BITS - 1 - LANE_START(p, lane)
                                                   downto HOLD_BITS - LANE_END(p, lane));
            else
              vCandidateByte := vStuffBuffer(HOLD_BITS - 1 - LANE_START(p, lane)
                                            downto HOLD_BITS - LANE_END(p, lane));
            end if;
            vCandidateWord(OUT_WIDTH - 1 - lane * 8 downto OUT_WIDTH - (lane + 1) * 8) := vCandidateByte;

            -- Last complete lane; end thresholds rise strictly, so one lane wins
            vLaneSelected := vPath(p) and iReady and
                             bool2bit(vStuffBufferBits >= LANE_END(p, lane));
            if (lane < OUT_BYTES_PER_CYCLE - 1) then
              vLaneSelected := vLaneSelected and
                               bool2bit(vStuffBufferBits < LANE_END(p, lane + 1));
            end if;
            vBytesMask     := (others => vLaneSelected);
            vEmitBytesBits := vEmitBytesBits or (to_unsigned(lane + 1, vEmitBytesBits'length) and vBytesMask);
            vEmitLastFF    := vEmitLastFF or (vLaneSelected and bool2bit(vCandidateByte = x"FF"));

            for k in CONSUME_BITS'range loop

              if (CONSUME_BITS(k) = LANE_END(p, lane)) then
                vTakeShift(k) := vTakeShift(k) or vLaneSelected;
              end if;

            end loop;

          end loop;

          vEmitData := vEmitData or (vCandidateWord and vPathMask);

        end loop;

        -- (4) One-hot fixed shifts and count decrements, no barrel shifter
        vEmitBytes := to_integer(vEmitBytesBits);
        -- No complete first byte (or stalled): keep every bit, independent of the decoder
        vTakeShift(0) := not iReady or bool2bit(vStuffBufferBits < 7) or
                         (not vPrevFF and bool2bit(vStuffBufferBits = 7));
        vNextBuffer := (others => '0');
        vNextCount  := (others => '0');

        for k in CONSUME_BITS'range loop

          vBufferMask := (others => vTakeShift(k));
          vCountMask  := (others => vTakeShift(k));
          vNextBuffer := vNextBuffer or
                         (std_logic_vector(shift_left(unsigned(vStuffBuffer), CONSUME_BITS(k))) and vBufferMask);
          -- Unselected candidates may wrap; their mask is zero
          vNextCount := vNextCount or
                        ((to_unsigned(vStuffBufferBits, vNextCount'length) - CONSUME_BITS(k)) and vCountMask);

        end loop;

        vStuffBuffer     := vNextBuffer;
        vStuffBufferBits := to_integer(vNextCount);
        if (vEmitBytes > 0) then
          vPrevFF := vEmitLastFF;
        end if;

        if (vEmitBytes > 0) then
          sOutWordReg_r(3) <= vEmitData;
          sOutBytesValidReg_r(3) <= to_unsigned(vEmitBytes, sOutBytesValidReg'length);
          sOutValidReg_r(3) <= '1';
        else
          sOutValidReg_r(3) <= '0';
        end if;

        -- Last word in, < 8 bits left: the sLastPending branch emits the final beat
        if (iReady = '1'
            and vStuffBufferLast = '1'
            and vStuffBufferBits < 8) then
          sLastPending_r(3) <= '1';
        end if;

        sStuffBuffer_r(3) <= vStuffBuffer;
        sStuffBufferBits_r(3) <= to_unsigned(vStuffBufferBits, sStuffBufferBits'length);
        sStuffBufferLast_r(3) <= vStuffBufferLast;
        sPrevFF_r(3) <= vPrevFF;
      end if;
    end if;

  end process stage3_proc;

  postponed assert is_x(sWordValidLen) or sWordValidLen <= IN_WIDTH
    report "byte_stuffer: input valid length past IN_WIDTH" severity error;
  postponed assert is_x(sAccumCountBits) or sAccumCountBits <= ACCUM_BITS
    report "byte_stuffer: accumulator count past ACCUM_BITS" severity error;
  postponed assert is_x(sAccumCountBitsFlush) or sAccumCountBitsFlush <= ACCUM_BITS
    report "byte_stuffer: flush count past ACCUM_BITS" severity error;
  postponed assert is_x(sFlushValidBits) or sFlushValidBits <= FIFO_BITS
    report "byte_stuffer: flush valid bits past FIFO_BITS" severity error;
  postponed assert is_x(sBvQueueOutData) or unsigned(sBvQueueOutData) <= FIFO_BITS
    report "byte_stuffer: queued valid bits past FIFO_BITS" severity error;
  postponed assert is_x(sStuffBufferBits) or sStuffBufferBits <= HOLD_BITS
    report "byte_stuffer: stuff buffer count past HOLD_BITS" severity error;
  postponed assert is_x(sOutBytesValidReg) or sOutBytesValidReg <= OUT_BYTES_PER_CYCLE
    report "byte_stuffer: output byte count past OUT_BYTES_PER_CYCLE" severity error;

end architecture behavioral;
