-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- nl_merge: the 4 slice streams into one, in chunks, each after an 8-byte header (D24 B)
--
-- Format: Docs/output_format.md. Per lane two chunk buffers (ping-pong) in one RAM; a chunk closes
-- when header + payload reach CHUNK_BYTES or on the slice's last beat. Closed chunks go out oldest
-- first per lane, lanes in round robin. At 64 bits the header is one beat, so no realign. oLast ends
-- every chunk; oKeep is all ones except on oLast.

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.olo_base_pkg_math.log2ceil;

entity nl_merge is
  generic (
    OUT_WIDTH   : positive range 64 to 64    := 64;   -- wider: see Docs/decisions.md D24
    CHUNK_BYTES : positive range 16 to 65528 := 1024;  -- header included; multiple of 8
    BAND_ID     : natural range 0 to 255    := 0
  );
  port (
    iClk   : in    std_logic;
    iRst   : in    std_logic;
    -- Slice beats of lane iLane: contiguous from the MSB, full except a slice's last
    iLane  : in    unsigned(1 downto 0);
    iWord  : in    std_logic_vector(OUT_WIDTH - 1 downto 0);
    iValid : in    std_logic;
    iBytes : in    unsigned(log2ceil(OUT_WIDTH / 8 + 1) - 1 downto 0);  -- [1, OUT_WIDTH/8]
    iLast  : in    std_logic;
    oReady : out   std_logic_vector(3 downto 0);                       -- per lane
    -- AXI-Stream out
    oData  : out   std_logic_vector(OUT_WIDTH - 1 downto 0);
    oValid : out   std_logic;
    oKeep  : out   std_logic_vector(OUT_WIDTH / 8 - 1 downto 0);
    oLast  : out   std_logic;
    iReady : in    std_logic
  );
end entity nl_merge;

architecture rtl of nl_merge is

  constant VERSION : natural := 1;
  constant NB      : natural := OUT_WIDTH / 8;
  constant PB      : natural := CHUNK_BYTES / NB - 1;                   -- payload beats per chunk
  constant DEPTH   : natural := 8 * PB;                                 -- 4 lanes x 2 chunks
  constant AW      : natural := log2ceil(DEPTH);
  -- Entry: last, keep, data. One read in flight past AlmFull.
  constant QW      : natural := 1 + NB + OUT_WIDTH;
  constant QDEPTH  : natural := 8;

  type t_u16 is array (natural range <>) of unsigned(15 downto 0);
  type t_beat is array (natural range <>) of natural range 0 to PB - 1;
  type t_bytes is array (natural range <>) of unsigned(iBytes'length - 1 downto 0);

  -- Chunk c = 2 * lane + half
  signal sWrHalf   : std_logic_vector(3 downto 0);
  signal sWrBeat   : t_beat(0 to 3);
  signal sImgCount : t_u16(0 to 3);
  signal sFull     : std_logic_vector(0 to 7);
  signal sLen      : t_u16(0 to 7);                                     -- payload bytes
  signal sEnd      : t_beat(0 to 7);                                    -- last beat index
  signal sTail     : t_bytes(0 to 7);                                   -- bytes of the last beat
  signal sSliceEnd : std_logic_vector(0 to 7);
  signal sChImg    : t_u16(0 to 7);
  signal sWe       : std_logic;
  signal sWrAddr   : unsigned(AW - 1 downto 0);

  signal sRdHalf   : std_logic_vector(3 downto 0);
  signal sRdLane   : unsigned(1 downto 0);
  signal sRdBeat   : natural range 0 to PB - 1;
  signal sReading  : std_logic;
  signal sRdAddr   : unsigned(AW - 1 downto 0);
  signal sRam      : std_logic_vector(OUT_WIDTH - 1 downto 0);

  -- Read stage -> queue (aligned with the RAM output)
  signal pValid    : std_logic;
  signal pHdr      : std_logic;
  signal pHdrWord  : std_logic_vector(OUT_WIDTH - 1 downto 0);
  signal pLast     : std_logic;
  signal pKeep     : std_logic_vector(NB - 1 downto 0);
  signal sQIn      : std_logic_vector(QW - 1 downto 0);
  signal sQOut     : std_logic_vector(QW - 1 downto 0);
  signal sQAlm     : std_logic;

  function keep (
    n : unsigned
  ) return std_logic_vector is
    variable v : std_logic_vector(NB - 1 downto 0);
  begin

    for i in 0 to NB - 1 loop
      v(NB - 1 - i) := '1' when i < to_integer(n) else '0';
    end loop;
    return v;

  end function keep;

  function addr (
    lane : unsigned(1 downto 0);
    half : std_logic;
    beat : natural
  ) return unsigned is
  begin

    return to_unsigned((to_integer(lane) * 2 + to_integer(unsigned'('0' & half))) * PB + beat, AW);

  end function addr;

begin

  assert CHUNK_BYTES mod NB = 0
    report "nl_merge: CHUNK_BYTES must be a multiple of 8" severity failure;

  gen_rdy : for l in 0 to 3 generate
    oReady(l) <= not sFull(2 * l + to_integer(unsigned'('0' & sWrHalf(l))));
  end generate gen_rdy;

  sWe     <= iValid and oReady(to_integer(iLane));
  sWrAddr <= addr(iLane, sWrHalf(to_integer(iLane)), sWrBeat(to_integer(iLane)));
  sRdAddr <= addr(sRdLane, sRdHalf(to_integer(sRdLane)), sRdBeat);

  u_ram : entity work.olo_base_ram_sdp
    generic map (
      Depth_g => DEPTH,
      Width_g => OUT_WIDTH
    )
    port map (
      Clk     => iClk,
      Wr_Addr => std_logic_vector(sWrAddr),
      Wr_Ena  => sWe,
      Wr_Data => iWord,
      Rd_Addr => std_logic_vector(sRdAddr),
      Rd_Data => sRam
    );

  -- Writer closes chunks, reader picks and frees them
  p_ctrl : process (iClk) is

    variable vLane : natural range 0 to 3;
    variable vC    : natural range 0 to 7;
    variable vPick : integer range -1 to 3;
    variable vL    : natural range 0 to 3;

  begin

    if rising_edge(iClk) then
      if (iRst = '1') then
        sWrHalf   <= (others => '0');
        sWrBeat   <= (others => 0);
        sImgCount <= (others => (others => '0'));
        sFull     <= (others => '0');
        sRdHalf   <= (others => '0');
        sRdLane   <= (others => '0');
        sRdBeat   <= 0;
        sReading  <= '0';
        pValid    <= '0';
      else
        vLane := to_integer(iLane);
        vC    := 2 * vLane + to_integer(unsigned'('0' & sWrHalf(vLane)));
        if (sWe = '1') then
          if (iLast = '1' or sWrBeat(vLane) = PB - 1) then
            sFull(vC)      <= '1';
            sLen(vC)       <= to_unsigned(sWrBeat(vLane) * NB + to_integer(iBytes), 16);
            sEnd(vC)       <= sWrBeat(vLane);
            sTail(vC)      <= iBytes;
            sSliceEnd(vC)  <= iLast;
            sChImg(vC)     <= sImgCount(vLane);
            sWrHalf(vLane) <= not sWrHalf(vLane);
            sWrBeat(vLane) <= 0;
            if (iLast = '1') then
              sImgCount(vLane) <= sImgCount(vLane) + 1;
            end if;
          else
            sWrBeat(vLane) <= sWrBeat(vLane) + 1;
          end if;
        end if;

        pValid <= '0';
        vC     := 2 * to_integer(sRdLane) + to_integer(unsigned'('0' & sRdHalf(to_integer(sRdLane))));
        if (sQAlm = '0') then
          if (sReading = '1') then
            pValid <= '1';
            pHdr   <= '0';
            if (sRdBeat = sEnd(vC)) then
              pLast                           <= '1';
              pKeep                           <= keep(sTail(vC));
              sFull(vC)                       <= '0';
              sRdHalf(to_integer(sRdLane))    <= not sRdHalf(to_integer(sRdLane));
              sRdBeat                         <= 0;
              sReading                        <= '0';
            else
              pLast   <= '0';
              pKeep   <= (others => '1');
              sRdBeat <= sRdBeat + 1;
            end if;
          else
            -- Round robin from the lane after the last one served
            vPick := -1;
            for i in 1 to 4 loop
              vL := (to_integer(sRdLane) + i) mod 4;
              if (vPick < 0 and sFull(2 * vL + to_integer(unsigned'('0' & sRdHalf(vL)))) = '1') then
                vPick := vL;
              end if;
            end loop;
            if (vPick >= 0) then
              vC       := 2 * vPick + to_integer(unsigned'('0' & sRdHalf(vPick)));
              sRdLane  <= to_unsigned(vPick, 2);
              sReading <= '1';
              pValid   <= '1';
              pHdr     <= '1';
              pLast    <= '0';
              pKeep    <= (others => '1');
              pHdrWord <= x"FFB0" &
                          std_logic_vector(to_unsigned(VERSION, 4)) & std_logic_vector(to_unsigned(vPick, 2)) &
                          sSliceEnd(vC) & '0' &
                          std_logic_vector(to_unsigned(BAND_ID, 8)) & std_logic_vector(sChImg(vC)) &
                          std_logic_vector(sLen(vC));
            end if;
          end if;
        end if;
      end if;
    end if;

  end process p_ctrl;

  sQIn <= pLast & pKeep & pHdrWord when pHdr = '1' else
          pLast & pKeep & sRam;

  -- FWFT queue drives the AXIS output: holds the beat until iReady
  u_queue : entity work.olo_base_fifo_sync
    generic map (
      Width_g        => QW,
      Depth_g        => QDEPTH,
      AlmFullOn_g    => true,
      AlmFullLevel_g => QDEPTH - 2
    )
    port map (
      Clk       => iClk,
      Rst       => iRst,
      In_Data   => sQIn,
      In_Valid  => pValid,
      Out_Data  => sQOut,
      Out_Valid => oValid,
      Out_Ready => iReady,
      AlmFull   => sQAlm
    );

  oLast <= sQOut(QW - 1);
  oKeep <= sQOut(QW - 2 downto OUT_WIDTH);
  oData <= sQOut(OUT_WIDTH - 1 downto 0);

  -- psl default clock is rising_edge(iClk);
  -- psl assert always ((iValid = '1' and iLast = '0') -> iBytes = NB) abort (iRst = '1') report "nl_merge: only a slice's last beat may be partial";
  -- psl assert always (iValid = '1' -> iBytes >= 1) abort (iRst = '1') report "nl_merge: empty beat";

end architecture rtl;
