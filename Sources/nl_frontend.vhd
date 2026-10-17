-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- nl_frontend: raster input to 4 vertical stripes (lanes), everything in front of nl_loop
--
-- Rows go through a double-buffered row RAM and are issued as column groups: column j of lanes
-- 0..3 in 4 consecutive slots, so the lane of a slot is cycle mod 4. Stages, n cycles after issue:
--   I2 Rb, Rc, Rd   I3 D1, D2, Q3 bounds   I4 Q1, Q2   I5 sign merge, context RAM read
--   I8 = L loop input   I15 = E loop exit, Rx and context write back
-- Context RAM: one bank per Q3 candidate at lane*64 + row, ctx = 9*row + bank. The write of the
-- slot that left the loop at L-1 is forwarded here; the loop forwards the newest itself.
-- Ws = W/4 >= 8 so Rx is written back before the next row reads it. The loop has no reset:
-- exits of slots issued before iRst are dropped (oExValid low).

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use work.nl_pkg.all;

library work;
  use work.olo_base_pkg_math.log2ceil;

entity nl_frontend is
  generic (
    BITNESS          : natural range 8 to 16 := 12;
    MAX_IMAGE_WIDTH  : positive              := 4096
  );
  port (
    iClk      : in    std_logic;
    iRst      : in    std_logic;
    -- Per image, stable while iRst = '0'
    iWidth    : in    unsigned(15 downto 0);                              -- multiple of 4
    iHeight   : in    unsigned(15 downto 0);
    iCfgDone  : in    std_logic;
    iNear     : in    unsigned(7 downto 0);
    iT1       : in    unsigned(BITNESS - 1 downto 0);
    iT2       : in    unsigned(BITNESS - 1 downto 0);
    iT3       : in    unsigned(BITNESS - 1 downto 0);
    -- Raster input
    iValid    : in    std_logic;
    iPixel    : in    std_logic_vector(BITNESS - 1 downto 0);
    oReady    : out   std_logic;
    iHold     : in    std_logic;                                          -- do not issue column groups
    -- Clear sweep (A RAMs), 512 cycles per image
    oClr      : out   std_logic;
    oClrAddr  : out   unsigned(8 downto 0);
    -- Loop inputs (L)
    oValid    : out   std_logic;
    oFirstPix : out   std_logic;
    oFirstCol : out   std_logic;
    oEol      : out   std_logic;
    oIx       : out   unsigned(BITNESS - 1 downto 0);
    oRb       : out   unsigned(BITNESS - 1 downto 0);
    oRc       : out   unsigned(BITNESS - 1 downto 0);
    oQ12Zero  : out   std_logic;
    oQ3Bound  : out   t_signed_array(0 to 7)(BITNESS + 1 downto 0);
    oCandCtx  : out   t_unsigned_array(0 to 8)(CO_CTX_WIDTH - 1 downto 0);
    oCandSign : out   std_logic_vector(0 to 8);
    oCandC    : out   t_signed_array(0 to 8)(CO_C_WIDTH - 1 downto 0);
    oCandB    : out   t_signed_array(0 to 8)(6 downto 0);
    oCandN    : out   t_unsigned_array(0 to 8)(6 downto 0);
    -- Loop exit (E)
    iExValid  : in    std_logic;
    iExMode   : in    std_logic_vector(1 downto 0);
    iExRx     : in    unsigned(BITNESS - 1 downto 0);
    iExCtx    : in    unsigned(CO_CTX_WIDTH - 1 downto 0);
    iExCq     : in    signed(CO_C_WIDTH - 1 downto 0);
    iExBq     : in    signed(6 downto 0);
    iExNq     : in    unsigned(6 downto 0);
    -- Tags aligned with the loop exit
    oExValid  : out   std_logic;                                          -- iExValid of a slot issued since reset
    oExLane   : out   unsigned(1 downto 0);
    oExFirstPix : out std_logic;                                          -- first pixel of the lane image
    oExLastPix  : out std_logic;                                          -- last pixel of the lane image
    oExEol    : out   std_logic
  );
end entity nl_frontend;

architecture behavioral of nl_frontend is

  constant AW      : natural := log2ceil(MAX_IMAGE_WIDTH);
  constant SW      : natural := BITNESS + 2;                              -- signed width
  constant WORD_W  : natural := CO_C_WIDTH + 7 + 7;
  constant L       : natural := 8;
  constant E       : natural := L + 7;

  type t_state is (S_CFG, S_DRAIN, S_CLEAR, S_RUN);

  type t_bank_array is array (0 to 8) of unsigned(3 downto 0);

  type t_slot is record
    valid    : std_logic;
    lane     : unsigned(1 downto 0);
    x        : unsigned(AW - 1 downto 0);
    firstPix : std_logic;
    firstCol : std_logic;
    eol      : std_logic;
    lastPix  : std_logic;
    row0     : std_logic;
    -- I3
    ix       : unsigned(BITNESS - 1 downto 0);
    rb       : unsigned(BITNESS - 1 downto 0);
    rc       : unsigned(BITNESS - 1 downto 0);
    rd       : unsigned(BITNESS - 1 downto 0);
    -- I4
    d1       : signed(SW - 1 downto 0);                                 -- [-MAXVAL, MAXVAL]
    d2       : signed(SW - 1 downto 0);                                 -- [-MAXVAL, MAXVAL]
    bound    : t_signed_array(0 to 7)(SW - 1 downto 0);                 -- [1-MAXVAL, 2*MAXVAL]
    -- I5
    q1       : integer range -4 to 4;
    q2       : integer range -4 to 4;
    -- I6
    neg      : std_logic;
    row      : unsigned(5 downto 0);                                    -- [0, 40]
    row9     : unsigned(CO_CTX_WIDTH - 1 downto 0);                     -- [0, 360]
    q12Zero  : std_logic;
    -- I7
    rev      : std_logic_vector(0 to 8);
    bank     : t_bank_array;                                            -- [0, 8]
    ctx      : t_unsigned_array(0 to 8)(CO_CTX_WIDTH - 1 downto 0);     -- [4, 368]
    sign     : std_logic_vector(0 to 8);
    -- I8
    fw       : std_logic_vector(0 to 8);
  end record t_slot;

  type t_slot_array is array (0 to E) of t_slot;

  type t_ring is record
    prevRd : unsigned(BITNESS - 1 downto 0);
    prevRb : unsigned(BITNESS - 1 downto 0);
    col0Rb : unsigned(BITNESS - 1 downto 0);
  end record t_ring;

  type t_ring_array is array (0 to 3) of t_ring;
  type t_base_array is array (0 to 3) of unsigned(AW - 1 downto 0);
  type t_word_array is array (0 to 8) of std_logic_vector(WORD_W - 1 downto 0);

  -- Control
  signal sState    : t_state := S_CFG;
  signal sCnt      : unsigned(8 downto 0);                             -- [0, 511], full type (drain [0, 31])
  signal sCyc      : unsigned(1 downto 0) := (others => '0');          -- lane slot, full type
  signal sWm1      : unsigned(AW - 1 downto 0);                        -- W-1: [31, MAX_IMAGE_WIDTH-1]
  signal sWsM1     : unsigned(15 downto 0);                            -- W/4-1: [7, MAX_IMAGE_WIDTH/4-1]
  signal sHm1      : unsigned(15 downto 0);                            -- H-1: [0, 65534]
  signal sBase     : t_base_array;                                     -- lane*W/4: [0, 3*W/4]
  signal sFull     : std_logic_vector(0 to 1) := "00";
  signal sWrBuf    : std_logic;
  signal sWrCol    : unsigned(AW - 1 downto 0);                        -- [0, W-1]
  signal sRdBuf    : std_logic;
  signal sJ        : unsigned(15 downto 0);                            -- stripe column: [0, W/4-1]
  signal sJp1      : unsigned(15 downto 0);                            -- [1, W/4]
  signal sRow      : unsigned(15 downto 0);                            -- [0, H-1]
  signal sGo       : std_logic;
  signal sReady    : std_logic;
  signal sAccept   : std_logic;

  -- Per image constants
  signal sNegT     : t_signed_array(0 to 2)(SW - 1 downto 0);            -- -T1, -T2, -T3
  signal sPosT     : t_signed_array(0 to 2)(SW - 1 downto 0);            -- T1, T2, T3
  signal sNegN     : signed(SW - 1 downto 0);                          -- [-NMAX, 0]
  signal sPosN     : signed(SW - 1 downto 0);                          -- [0, NMAX]
  signal sBoundK   : t_signed_array(0 to 7)(SW - 1 downto 0);            -- [1-MAXVAL, MAXVAL]

  -- Pipeline
  signal sIss      : t_slot;
  signal p         : t_slot_array;
  signal sRing     : t_ring_array;

  -- RAMs
  signal sRowWrAddr : std_logic_vector(AW downto 0);
  signal sRowRdAddr : std_logic_vector(AW downto 0);
  signal sRowRd     : std_logic_vector(BITNESS - 1 downto 0);
  signal sLbRdAddr  : unsigned(AW - 1 downto 0);
  signal sLbRd      : std_logic_vector(BITNESS - 1 downto 0);
  signal sLbWe      : std_logic;
  signal sLbWrAddr  : unsigned(AW - 1 downto 0);
  signal sLbWrData  : unsigned(BITNESS - 1 downto 0);
  signal sCtxRd     : t_word_array;
  signal sCtxWe     : std_logic_vector(0 to 8);
  signal sCtxWrAddr : unsigned(7 downto 0);                          -- lane & row: row [0, 40]
  signal sCtxWrData : std_logic_vector(WORD_W - 1 downto 0);
  signal sWbValid   : std_logic;
  signal sWbBank    : unsigned(3 downto 0);                             -- [0, 8] when sWbValid
  signal sWbAddr    : unsigned(7 downto 0);                            -- lane & row: row [0, 40]
  signal sWbData    : std_logic_vector(WORD_W - 1 downto 0);

  function bool2sl (
    b : boolean
  ) return std_logic is
  begin

    if (b) then
      return '1';
    else
      return '0';
    end if;

  end function bool2sl;

  function quant (
    d : signed; nT : t_signed_array; pT : t_signed_array; nN, pN : signed
  ) return integer is
  begin

    if (d <= nT(2)) then
      return -4;
    elsif (d <= nT(1)) then
      return -3;
    elsif (d <= nT(0)) then
      return -2;
    elsif (d < nN) then
      return -1;
    elsif (d <= pN) then
      return 0;
    elsif (d < pT(0)) then
      return 1;
    elsif (d < pT(1)) then
      return 2;
    elsif (d < pT(2)) then
      return 3;
    else
      return 4;
    end if;

  end function quant;

begin

  sReady <= not iRst and not sFull(0) when sWrBuf = '0' else
            not iRst and not sFull(1);
  oReady  <= sReady;
  sAccept <= iValid and sReady;

  ctrl_proc : process (iClk) is

    variable vFull : std_logic_vector(0 to 1);
    variable vGo   : std_logic;
    variable v     : t_slot;
    variable vX    : unsigned(AW - 1 downto 0);                        -- [0, W-1]
    variable vN    : signed(SW - 1 downto 0);                          -- [0, NMAX]

  begin

    if rising_edge(iClk) then
      sCyc <= sCyc + 1;
      vFull := sFull;

      -- Per image constants
      vN       := signed(resize(iNear, SW));
      sPosN    <= vN;
      sNegN    <= -vN;
      sPosT(0) <= signed(resize(iT1, SW));
      sPosT(1) <= signed(resize(iT2, SW));
      sPosT(2) <= signed(resize(iT3, SW));
      sNegT(0) <= -signed(resize(iT1, SW));
      sNegT(1) <= -signed(resize(iT2, SW));
      sNegT(2) <= -signed(resize(iT3, SW));
      sBoundK  <= (signed(resize(iT3, SW)), signed(resize(iT2, SW)), signed(resize(iT1, SW)),
                   vN + 1, -vN, 1 - signed(resize(iT1, SW)), 1 - signed(resize(iT2, SW)),
                   1 - signed(resize(iT3, SW)));

      -- Raster write side
      if (sAccept = '1') then
        if (sWrCol = sWm1) then
          sWrCol <= (others => '0');
          if (sWrBuf = '0') then
            vFull(0) := '1';
          else
            vFull(1) := '1';
          end if;
          sWrBuf <= not sWrBuf;
        else
          sWrCol <= sWrCol + 1;
        end if;
      end if;

      -- Issue
      v       := sIss;
      v.valid := '0';
      vGo     := '0';
      case sState is

        when S_CFG =>
          sCnt <= (others => '0');
          if (iCfgDone = '1') then
            sState <= S_CLEAR;
          end if;

        when S_DRAIN =>
          sCnt <= sCnt + 1;
          if (sCnt = 31) then
            sCnt   <= (others => '0');
            sState <= S_CLEAR;
          end if;

        when S_CLEAR =>
          sCnt <= sCnt + 1;
          if (sCnt = 511) then
            sState <= S_RUN;
          end if;

        when S_RUN =>
          if (sCyc = 0) then
            vGo := bool2sl(iHold = '0' and ((sRdBuf = '0' and sFull(0) = '1') or (sRdBuf = '1' and sFull(1) = '1')));
            sGo <= vGo;
          else
            vGo := sGo;
          end if;

      end case;

      if (vGo = '1') then
        vX         := sBase(to_integer(sCyc)) + sJ(AW - 1 downto 0);
        v.valid    := '1';
        v.lane     := sCyc;
        v.x        := vX;
        v.firstCol := bool2sl(sJ = 0);
        v.eol      := bool2sl(sJ = sWsM1);
        v.row0     := bool2sl(sRow = 0);
        v.firstPix := v.firstCol and v.row0;
        v.lastPix  := v.eol and bool2sl(sRow = sHm1);
        sRowRdAddr <= sRdBuf & std_logic_vector(vX);
        if (v.eol = '1') then
          sLbRdAddr <= sBase(to_integer(sCyc));
        else
          sLbRdAddr <= sBase(to_integer(sCyc)) + sJp1(AW - 1 downto 0);
        end if;

        if (sCyc = 3) then
          if (v.eol = '1') then
            sJ   <= (others => '0');
            sJp1 <= to_unsigned(1, 16);
            if (sRdBuf = '0') then
              vFull(0) := '0';
            else
              vFull(1) := '0';
            end if;
            sRdBuf <= not sRdBuf;
            if (v.lastPix = '1') then
              sRow   <= (others => '0');
              sState <= S_DRAIN;
              sCnt   <= (others => '0');
            else
              sRow <= sRow + 1;
            end if;
          else
            sJ   <= sJp1;
            sJp1 <= sJp1 + 1;
          end if;
        end if;
      end if;
      sIss  <= v;
      sFull <= vFull;

      if (iRst = '1') then
        sState   <= S_CFG;
        sFull    <= "00";
        sWrBuf   <= '0';
        sWrCol   <= (others => '0');
        sRdBuf   <= '0';
        sJ       <= (others => '0');
        sJp1     <= to_unsigned(1, 16);
        sRow     <= (others => '0');
        sGo      <= '0';
        sIss.valid <= '0';
        sWm1     <= resize(iWidth - 1, AW);
        sWsM1    <= shift_right(iWidth, 2) - 1;
        sHm1     <= iHeight - 1;
        sBase(0) <= (others => '0');
        sBase(1) <= resize(shift_right(iWidth, 2), AW);
        sBase(2) <= resize(shift_right(iWidth, 1), AW);
        sBase(3) <= resize(iWidth - shift_right(iWidth, 2), AW);
      end if;
    end if;

  end process ctrl_proc;

  oClr     <= '1' when sState = S_CLEAR else '0';
  oClrAddr <= sCnt;

  pipe_proc : process (iClk) is

    variable v    : t_slot;
    variable vR   : t_ring;
    variable vRd  : unsigned(BITNESS - 1 downto 0);
    variable vQ1  : integer range -4 to 4;
    variable vQ2  : integer range -4 to 4;
    variable vRow : integer range 0 to 40;
    variable vB   : integer range 0 to 8;

  begin

    if rising_edge(iClk) then
      p(1) <= sIss;
      p(2) <= p(1);

      -- I2: neighbour ring, one entry per lane, rotating with the slot phase
      v   := p(2);
      vR  := sRing(0);
      vRd := unsigned(sLbRd);
      v.ix := unsigned(sRowRd);
      v.rb := vR.prevRd;
      if (v.firstCol = '1') then
        v.rc := vR.col0Rb;
      else
        v.rc := vR.prevRb;
      end if;
      if (v.eol = '1') then
        v.rd := v.rb;
      else
        v.rd := vRd;
      end if;
      if (v.row0 = '1') then
        v.rb := (others => '0');
        v.rc := (others => '0');
        v.rd := (others => '0');
      end if;
      if (v.valid = '1') then
        vR.prevRd := vRd;
        vR.prevRb := v.rb;
        if (v.firstCol = '1') then
          vR.col0Rb := v.rb;
        end if;
      end if;
      sRing(0 to 2) <= sRing(1 to 3);
      sRing(3)      <= vR;
      p(3)          <= v;

      -- I3: gradients, Q3 bounds
      v    := p(3);
      v.d1 := signed(resize(v.rd, SW)) - signed(resize(v.rb, SW));
      v.d2 := signed(resize(v.rb, SW)) - signed(resize(v.rc, SW));
      for k in 0 to 7 loop
        v.bound(k) := signed(resize(v.rc, SW)) + sBoundK(k);
      end loop;
      p(4) <= v;

      -- I4: quantization
      v    := p(4);
      v.q1 := quant(v.d1, sNegT, sPosT, sNegN, sPosN);
      v.q2 := quant(v.d2, sNegT, sPosT, sNegN, sPosN);
      p(5) <= v;

      -- I5: sign merge, context row
      v     := p(5);
      v.neg := bool2sl(v.q1 < 0 or (v.q1 = 0 and v.q2 < 0));
      if (v.neg = '1') then
        vQ1 := -v.q1;
        vQ2 := -v.q2;
      else
        vQ1 := v.q1;
        vQ2 := v.q2;
      end if;
      vRow      := 9 * vQ1 + vQ2;
      v.row     := to_unsigned(vRow, 6);
      v.row9    := to_unsigned(9 * vRow, CO_CTX_WIDTH);
      v.q12Zero := bool2sl(v.q1 = 0 and v.q2 = 0);
      p(6)      <= v;

      -- I6: candidates (Q3 = t-4)
      v := p(6);
      for t in 0 to 8 loop
        if (v.q12Zero = '1') then
          v.rev(t)  := bool2sl(t < 4);
          v.sign(t) := bool2sl(t < 4);
        else
          v.rev(t)  := v.neg;
          v.sign(t) := v.neg;
        end if;
        if (v.rev(t) = '1') then
          vB := 8 - t;
        else
          vB := t;
        end if;
        v.bank(t) := to_unsigned(vB, 4);
        v.ctx(t)  := v.row9 + vB;
      end loop;
      p(7) <= v;

      -- I7: forward flags against the slot leaving the loop now (same lane, 8 slots earlier)
      v := p(7);
      for t in 0 to 8 loop
        v.fw(t) := iExValid and p(E).valid and bool2sl(iExMode = "00" and iExCtx = v.ctx(t));
      end loop;
      p(8) <= v;

      for k in 9 to E loop
        p(k) <= p(k - 1);
      end loop;

      -- E: write back
      sLbWe     <= iExValid and p(E).valid;
      sLbWrAddr <= p(E).x;
      sLbWrData <= iExRx;
      sWbValid  <= iExValid and p(E).valid and bool2sl(iExMode = "00");
      sWbAddr   <= p(E).lane & p(E).row;
      sWbBank   <= resize(iExCtx - p(E).row9, 4);
      sWbData   <= std_logic_vector(iExCq) & std_logic_vector(iExBq) & std_logic_vector(iExNq);

      if (iRst = '1') then
        for k in 1 to E loop
          p(k).valid <= '0';
        end loop;
        sLbWe    <= '0';
        sWbValid <= '0';
      end if;
    end if;

  end process pipe_proc;

  -- L: loop inputs
  oValid    <= p(L).valid;
  oFirstPix <= p(L).firstPix;
  oFirstCol <= p(L).firstCol;
  oEol      <= p(L).eol;
  oIx       <= p(L).ix;
  oRb       <= p(L).rb;
  oRc       <= p(L).rc;
  oQ12Zero  <= p(L).q12Zero;
  oQ3Bound  <= p(L).bound;
  oCandCtx  <= p(L).ctx;
  oCandSign <= p(L).sign;

  gen_cand : for t in 0 to 8 generate

    signal sWord : std_logic_vector(WORD_W - 1 downto 0);

  begin

    sWord <= sWbData when p(L).fw(t) = '1' else
             sCtxRd(8 - t) when p(L).rev(t) = '1' else
             sCtxRd(t);
    oCandC(t) <= signed(sWord(WORD_W - 1 downto 14));
    oCandB(t) <= signed(sWord(13 downto 7));
    oCandN(t) <= unsigned(sWord(6 downto 0));

  end generate gen_cand;

  oExValid    <= iExValid and p(E).valid;
  oExLane     <= p(E).lane;
  oExFirstPix <= p(E).firstPix;
  oExLastPix  <= p(E).lastPix;
  oExEol      <= p(E).eol;

  -- RAMs
  sRowWrAddr <= sWrBuf & std_logic_vector(sWrCol);

  u_row_ram : entity work.olo_base_ram_sdp
    generic map (
      Depth_g     => 2 ** (AW + 1),
      Width_g     => BITNESS,
      RdLatency_g => 2
    )
    port map (
      Clk     => iClk,
      Wr_Addr => sRowWrAddr,
      Wr_Ena  => sAccept,
      Wr_Data => iPixel,
      Rd_Addr => sRowRdAddr,
      Rd_Data => sRowRd
    );

  u_line_ram : entity work.olo_base_ram_sdp
    generic map (
      Depth_g     => 2 ** AW,
      Width_g     => BITNESS,
      RdLatency_g => 2
    )
    port map (
      Clk     => iClk,
      Wr_Addr => std_logic_vector(sLbWrAddr),
      Wr_Ena  => sLbWe,
      Wr_Data => std_logic_vector(sLbWrData),
      Rd_Addr => std_logic_vector(sLbRdAddr),
      Rd_Data => sLbRd
    );

  sCtxWrAddr <= sCnt(7 downto 0) when sState = S_CLEAR else sWbAddr;
  sCtxWrData <= (WORD_W - 1 downto 1 => '0') & '1' when sState = S_CLEAR else sWbData;

  gen_bank : for b in 0 to 8 generate

    sCtxWe(b) <= '1' when sState = S_CLEAR and sCnt(8) = '0' else
                 sWbValid when sWbBank = b else
                 '0';

    u_ctx_ram : entity work.olo_base_ram_sdp
      generic map (
        Depth_g     => 256,
        Width_g     => WORD_W,
        RdLatency_g => 2
      )
      port map (
        Clk     => iClk,
        Wr_Addr => std_logic_vector(sCtxWrAddr),
        Wr_Ena  => sCtxWe(b),
        Wr_Data => sCtxWrData,
        Rd_Addr => std_logic_vector(p(6).lane) & std_logic_vector(p(6).row),
        Rd_Data => sCtxRd(b)
      );

  end generate gen_bank;

  -- Config at image start, pipeline ranges on valid slots, loop contract at E
  assert_proc : process (iClk) is
    constant MAXVAL : integer := 2 ** BITNESS - 1;
  begin
    if rising_edge(iClk) then
      if (iRst = '0' and sState = S_CFG and iCfgDone = '1') then
        assert sWm1(1 downto 0) = "11" and sWsM1 >= 7 and to_integer(sWm1) < MAX_IMAGE_WIDTH and
               sWsM1 = shift_right(resize(sWm1, 16), 2)
          report "nl_frontend: width not a multiple of 4 in [32, MAX_IMAGE_WIDTH]" severity failure;
        assert sHm1 /= x"FFFF"
          report "nl_frontend: height 0" severity failure;
        assert iNear < iT1 and iT1 <= iT2 and iT2 <= iT3
          report "nl_frontend: thresholds not NEAR < T1 <= T2 <= T3" severity failure;
      end if;
      if (sState = S_DRAIN) then
        assert sCnt <= 31 report "nl_frontend: drain counter overrun" severity failure;
      end if;
      if (sState = S_RUN and sGo = '1') then
        assert sJ <= sWsM1 and sRow <= sHm1
          report "nl_frontend: issue column / row outside the image" severity failure;
      end if;
      if (p(4).valid = '1') then
        assert abs(p(4).d1) <= MAXVAL and abs(p(4).d2) <= MAXVAL
          report "nl_frontend: D1 / D2 outside [-MAXVAL, MAXVAL]" severity failure;
        for k in 0 to 7 loop
          assert p(4).bound(k) >= 1 - MAXVAL and p(4).bound(k) <= 2 * MAXVAL
            report "nl_frontend: Q3 bound outside [1-MAXVAL, 2*MAXVAL]" severity failure;
        end loop;
      end if;
      if (p(7).valid = '1') then
        assert p(7).row <= 40 report "nl_frontend: context row above 40" severity failure;
        for t in 0 to 8 loop
          assert p(7).ctx(t) >= 4 and p(7).ctx(t) <= 368
            report "nl_frontend: candidate ctx outside [4, 368]" severity failure;
        end loop;
      end if;
      assert p(E).valid /= '1' or iExValid = '1'
        report "nl_frontend: loop exit not E - L cycles after the loop input" severity failure;
      if (p(E).valid = '1' and iExMode = "00") then
        assert iExCtx >= p(E).row9 and iExCtx <= p(E).row9 + 8
          report "nl_frontend: regular exit ctx not a candidate of the slot" severity failure;
      end if;
      if (sWbValid = '1') then
        assert sWbBank <= 8 report "nl_frontend: write-back bank above 8" severity failure;
      end if;
    end if;
  end process assert_proc;

  -- A cleared word is N = 1, B = 0
  gen_cand_assert : for t in 0 to 8 generate
    postponed assert oValid /= '1' or is_x(oCandN(t)) or (oCandN(t) >= 1 and oCandN(t) <= 64)
      report "nl_frontend: candidate N outside [1, RESET]" severity error;
    postponed assert oValid /= '1' or is_x(oCandB(t)) or is_x(oCandN(t)) or
                     (oCandB(t) <= 0 and to_integer(oCandB(t)) + to_integer(oCandN(t)) > 0)
      report "nl_frontend: candidate B outside [1-N, 0]" severity error;
  end generate gen_cand_assert;

end architecture behavioral;
