-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- nl_loop: T.87 A.4-A.9, A.12-A.14, A.17-A.19, the Ra -> Rx recurrence
--
-- Pipeline S0..S6 with a 4-cycle loop: Ra and run state S4 -> S1, C S5 -> S2, B and N S6 -> S3.
-- Slot k belongs to lane k mod 4, so one stream needs 3 empty slots between pixels; empty slots
-- pass the lane state through.
-- Everything independent of Ra comes in precomputed: Q3 = -4 + #{k : Ra < iQ3Bound(k)}, and
-- iCand*(t) is the context of Q3 = t - 4 after sign merging. Only the lane's last regular update
-- is forwarded; the context RAM read must already hold older writes.
-- Ranges are for BITNESS 16, NEAR 255, RESET 64; D = 2*NEAR + 1.

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use work.nl_pkg.all;

entity nl_loop is
  generic (
    BITNESS : natural  := 12;
    RESET   : positive := 64
  );
  port (
    iClk      : in    std_logic;
    -- Per image
    iNear     : in    unsigned(7 downto 0);
    iMaxVal   : in    unsigned(BITNESS - 1 downto 0);
    iRecip    : in    unsigned(BITNESS + 10 downto 0);
    iRange    : in    unsigned(BITNESS downto 0);
    iRangeD   : in    unsigned(BITNESS + 1 downto 0);                -- RANGE*(2*NEAR+1)
    -- Per pixel
    iValid    : in    std_logic;
    iFirstPix : in    std_logic;                                      -- first pixel of the stripe image
    iFirstCol : in    std_logic;                                      -- Ra = Rb
    iEol      : in    std_logic;
    iIx       : in    unsigned(BITNESS - 1 downto 0);
    iRb       : in    unsigned(BITNESS - 1 downto 0);
    iRc       : in    unsigned(BITNESS - 1 downto 0);
    iQ12Zero  : in    std_logic;
    iQ3Bound  : in    t_signed_array(0 to 7)(BITNESS + 1 downto 0);
    iCandCtx  : in    t_unsigned_array(0 to 8)(CO_CTX_WIDTH - 1 downto 0);
    iCandSign : in    std_logic_vector(0 to 8);                       -- '1' = SIGN -1
    iCandC    : in    t_signed_array(0 to 8)(CO_C_WIDTH - 1 downto 0);
    iCandB    : in    t_signed_array(0 to 8);
    iCandN    : in    t_unsigned_array(0 to 8);
    -- Loop exit
    oValid    : out   std_logic;
    oMode     : out   std_logic_vector(1 downto 0);                   -- 00 regular, 01 run hit, 10 run interruption
    oRiType   : out   std_logic;
    oRx       : out   unsigned(BITNESS - 1 downto 0);
    oErrval   : out   signed(BITNESS downto 0);                       -- quantized, modulo reduced
    oCtx      : out   unsigned(CO_CTX_WIDTH - 1 downto 0);
    oCq       : out   signed(CO_C_WIDTH - 1 downto 0);
    oBq       : out   signed;
    oNq       : out   unsigned;
    oBpre     : out   signed;                                         -- before the update (A.10, A.11)
    oNpre     : out   unsigned;
    oNRes     : out   std_logic;                                      -- N = RESET before the update
    -- Verification only, open in nl_top
    oPx       : out   unsigned(BITNESS - 1 downto 0)
  );
end entity nl_loop;

architecture behavioral of nl_loop is

  constant W           : natural := BITNESS + 5;                      -- internal signed width
  constant SH          : natural := BITNESS + 10;
  constant CW          : natural := CO_C_WIDTH + 1;                   -- sign-applied C
  constant DW          : natural := CO_D_WIDTH + 1;                   -- signed 2*NEAR+1
  constant NW          : natural := iCandN(0)'length;
  constant BW          : natural := iCandB(0)'length;
  constant STAGES      : natural := 6;

  -- Lane state that must be back in S1 four slots later
  type t_early is record
    ra        : unsigned(BITNESS - 1 downto 0);
    inRun     : std_logic;
    lastValid : std_logic;
    lastCtx   : unsigned(CO_CTX_WIDTH - 1 downto 0);
  end record t_early;

  type t_pipe is record
    -- S0
    firstPix  : std_logic;
    firstCol  : std_logic;
    eol       : std_logic;
    q12Zero   : std_logic;
    ix        : signed(W - 1 downto 0);                       -- [0, MAXVAL]
    rb        : signed(W - 1 downto 0);                       -- [0, MAXVAL]
    rc        : signed(W - 1 downto 0);                       -- [0, MAXVAL]
    rbRc      : signed(W - 1 downto 0);                       -- [-MAXVAL, MAXVAL]
    ixN       : signed(W - 1 downto 0);                       -- [NEAR, MAXVAL+NEAR]
    ixM       : signed(W - 1 downto 0);                       -- [-NEAR, MAXVAL-NEAR]
    rbN       : signed(W - 1 downto 0);                       -- [NEAR, MAXVAL+NEAR]
    rbM       : signed(W - 1 downto 0);                       -- [-NEAR, MAXVAL-NEAR]
    magMax    : signed(W - 1 downto 0);                       -- [NEAR, MAXVAL+NEAR]
    rcGeRb    : std_logic;
    rcLeRb    : std_logic;
    bound     : t_signed_array(0 to 7)(BITNESS + 1 downto 0); -- [1-MAXVAL, 2*MAXVAL]
    cCtx      : t_unsigned_array(0 to 8)(CO_CTX_WIDTH - 1 downto 0);
    cSign     : std_logic_vector(0 to 8);
    cCs       : t_signed_array(0 to 8)(CW - 1 downto 0);      -- [-127, 128]
    cC        : t_signed_array(0 to 8)(CO_C_WIDTH - 1 downto 0); -- [-128, 127]
    cB        : t_signed_array(0 to 8)(BW - 1 downto 0);      -- [1-RESET, 0]
    cN        : t_unsigned_array(0 to 8)(NW - 1 downto 0);    -- [1, RESET]
    -- S1
    st        : t_early;
    ra        : unsigned(BITNESS - 1 downto 0);               -- [0, MAXVAL]
    run       : std_logic;
    hit       : std_logic;
    riType    : std_logic;
    match     : std_logic;
    ctx       : unsigned(CO_CTX_WIDTH - 1 downto 0);
    sign      : std_logic;
    cs        : signed(CW - 1 downto 0);                      -- [-127, 128]
    c         : signed(CO_C_WIDTH - 1 downto 0);              -- [-128, 127]
    b         : signed(BW - 1 downto 0);                      -- [1-N, 0]
    n         : unsigned(NW - 1 downto 0);                    -- [1, RESET]
    pbase     : signed(W - 1 downto 0);                       -- [0, MAXVAL]
    -- S2
    lastC     : signed(CO_C_WIDTH - 1 downto 0);              -- [-128, 127]
    lastCn    : signed(CW - 1 downto 0);                      -- [-127, 128]
    px        : signed(W - 1 downto 0);                       -- [0, MAXVAL]
    m         : unsigned(BITNESS downto 0);                   -- |Errval|+NEAR: [0, MAXVAL+NEAR]
    dirPos    : std_logic;
    ePos      : std_logic;
    -- S3
    q         : unsigned(BITNESS downto 0);                   -- m/D: [0, (MAXVAL+NEAR)/D]
    dS        : signed(DW - 1 downto 0);                      -- +-D
    dE        : signed(DW - 1 downto 0);                      -- +-D
    pxOv      : signed(W - 1 downto 0);                       -- Px-MAXVAL-1: [-MAXVAL-1, -1]
    cP1       : signed(CO_C_WIDTH - 1 downto 0);              -- [-127, 127]
    cM1       : signed(CO_C_WIDTH - 1 downto 0);              -- [-128, 126]
    cNeg      : signed(CW - 1 downto 0);                      -- [-127, 128]
    cP1Neg    : signed(CW - 1 downto 0);                      -- [-127, 127]
    cM1Neg    : signed(CW - 1 downto 0);                      -- [-126, 128]
    lastB     : signed(BW - 1 downto 0);                      -- [1-RESET, 0]
    lastN     : unsigned(NW - 1 downto 0);                    -- [2, RESET]
    nRes      : std_logic;
    n1        : unsigned(NW - 1 downto 0);                    -- [2, RESET]
    -- S4
    rx        : unsigned(BITNESS - 1 downto 0);               -- [0, MAXVAL]
    qdE       : signed(W - 1 downto 0);                       -- +-(MAXVAL+NEAR)
    wrapDn    : std_logic;
    wrapUp    : std_logic;
    x         : signed(W - 1 downto 0);                       -- B+-RANGE*D: [1-RESET-RANGE*D, RANGE*D]
    -- S5
    t         : signed(W - 1 downto 0);                       -- B+Errval*D (halved): [1-RESET-RANGE/2*D, RANGE/2*D]
    dec       : std_logic;
    inc       : std_logic;
    errq      : signed(BITNESS downto 0);                     -- [-floor(RANGE/2), ceil(RANGE/2)-1]
    c1        : signed(CO_C_WIDTH - 1 downto 0);              -- [-128, 127]
    -- S6
    b1        : signed(BW - 1 downto 0);                      -- [1-N, 0]
  end record t_pipe;

  type t_pipe_array is array (0 to STAGES) of t_pipe;

  signal sNear         : signed(W - 1 downto 0);                    -- [0, NMAX]
  signal sMaxVal       : signed(W - 1 downto 0);                    -- MAXVAL
  signal sMaxP1        : signed(W - 1 downto 0);                    -- MAXVAL+1
  signal sD            : unsigned(CO_D_WIDTH - 1 downto 0);         -- 2*NEAR+1: [1, 2*NMAX+1]
  signal sRecip        : unsigned(BITNESS + 10 downto 0);           -- ceil(2^SH/D): [2^SH/(2*NMAX+1), 2^SH]
  signal sRange        : signed(BITNESS downto 0);                  -- RANGE: [2, MAXVAL+1]
  signal sRangeD       : signed(W - 1 downto 0);                    -- RANGE*D: [MAXVAL+1, MAXVAL+4*NEAR+1]
  signal sHalf         : unsigned(BITNESS downto 0);                -- ceil(RANGE/2): [1, (MAXVAL+2)/2]
  signal sRangeMHalf   : unsigned(BITNESS downto 0);                -- floor(RANGE/2): [1, (MAXVAL+1)/2]

  signal sValid        : std_logic_vector(0 to STAGES) := (others => '0');
  signal r             : t_pipe_array;

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

begin

  pipe_proc : process (iClk) is

    variable v      : t_pipe;
    variable vSt    : t_early;
    variable vRa    : signed(W - 1 downto 0);                         -- [0, MAXVAL]
    variable vT     : natural range 0 to 8;
    variable vMatch : std_logic_vector(0 to 8);
    variable vRaLeRb : boolean;
    variable vMed   : signed(W - 1 downto 0);                         -- [0, MAXVAL] (A.5)
    variable vCs    : signed(W - 1 downto 0);                         -- SIGN*C: [-127, 128]
    variable vDiff  : signed(W - 1 downto 0);                         -- Ix-Px unclamped: [-MAXVAL-128, MAXVAL+127]
    variable vMA    : signed(W - 1 downto 0);                         -- [-MAXVAL-128, MAXVAL+NEAR+127]
    variable vMB    : signed(W - 1 downto 0);                         -- [-MAXVAL-128, MAXVAL+NEAR+128]
    variable vM     : signed(W - 1 downto 0);                         -- [0, MAXVAL+NEAR]
    variable vDd    : signed(DW - 1 downto 0);                        -- D: [1, 2*NMAX+1]
    variable vQs    : signed(BITNESS + 1 downto 0);                   -- q: [0, (MAXVAL+NEAR)/D]
    variable vRxRaw : signed(W - 1 downto 0);                         -- Px+-q*D: [-MAXVAL-NEAR, 2*MAXVAL+NEAR]
    variable vOvT   : signed(W - 1 downto 0);                         -- vRxRaw-MAXVAL-1
    variable vN     : unsigned(NW - 1 downto 0);                      -- [1, RESET]
    variable vSum   : signed(W - 1 downto 0);                         -- B+Errval*D: [1-RESET-RANGE/2*D, RANGE/2*D]
    variable vN1    : signed(W - 1 downto 0);                         -- [2, RESET]
    variable vB1    : signed(W - 1 downto 0);                         -- [1-N, 0] after the A.13 clamp

  begin

    if rising_edge(iClk) then
      sNear       <= signed(resize(iNear, W));
      sMaxVal     <= signed(resize(iMaxVal, W));
      sMaxP1      <= signed(resize(iMaxVal, W)) + 1;
      sD          <= resize(iNear & '1', CO_D_WIDTH);
      sRecip      <= iRecip;
      sRange      <= signed(iRange);
      sRangeD     <= signed(resize(iRangeD, W));
      sHalf       <= shift_right(iRange + 1, 1);
      sRangeMHalf <= iRange - shift_right(iRange + 1, 1);

      sValid <= iValid & sValid(0 to STAGES - 1);

      -- S0: input register and precompute
      v           := r(0);
      v.firstPix  := iFirstPix;
      v.firstCol  := iFirstCol;
      v.eol       := iEol;
      v.q12Zero   := iQ12Zero;
      v.ix        := signed(resize(iIx, W));
      v.rb        := signed(resize(iRb, W));
      v.rc        := signed(resize(iRc, W));
      v.rbRc      := v.rb - v.rc;
      v.ixN       := v.ix + sNear;
      v.ixM       := v.ix - sNear;
      v.rbN       := v.rb + sNear;
      v.rbM       := v.rb - sNear;
      v.magMax    := sMaxVal - v.ix + sNear;
      v.rcGeRb    := bool2sl(v.rc >= v.rb);
      v.rcLeRb    := bool2sl(v.rc <= v.rb);
      v.bound     := iQ3Bound;
      v.cCtx      := iCandCtx;
      v.cSign     := iCandSign;
      v.cC        := iCandC;
      v.cB        := iCandB;
      v.cN        := iCandN;
      for k in 0 to 8 loop
        if (iCandSign(k) = '1') then
          v.cCs(k) := -resize(iCandC(k), CW);
        else
          v.cCs(k) := resize(iCandC(k), CW);
        end if;
      end loop;
      r(0) <= v;

      -- S1: Ra, Q3, context select, MED
      v   := r(0);
      vSt := r(4).st;
      if (v.firstPix = '1') then
        vSt.inRun     := '0';
        vSt.lastValid := '0';
      end if;

      if (v.firstCol = '1') then
        vRa := v.rb;
      else
        vRa := signed(resize(vSt.ra, W));
      end if;
      v.ra := unsigned(vRa(BITNESS - 1 downto 0));

      vT := 0;
      for k in 0 to 7 loop
        if (vRa < resize(v.bound(k), W)) then
          vT := vT + 1;
        end if;
      end loop;

      for k in 0 to 8 loop
        vMatch(k) := vSt.lastValid and bool2sl(vSt.lastCtx = v.cCtx(k));
      end loop;

      v.run    := vSt.inRun or bool2sl(v.q12Zero = '1' and vT = 4);
      v.hit    := bool2sl(vRa >= v.ixM and vRa <= v.ixN);
      v.riType := bool2sl(vRa >= v.rbM and vRa <= v.rbN);
      v.ctx    := v.cCtx(vT);
      v.sign   := v.cSign(vT);
      v.cs     := v.cCs(vT);
      v.c      := v.cC(vT);
      v.b      := v.cB(vT);
      v.n      := v.cN(vT);
      v.match  := vMatch(vT) and not v.run;

      vRaLeRb := vRa <= v.rb;
      if (v.rc >= vRa and v.rcGeRb = '1') then
        if (vRaLeRb) then
          vMed := vRa;
        else
          vMed := v.rb;
        end if;
      elsif (v.rc <= vRa and v.rcLeRb = '1') then
        if (vRaLeRb) then
          vMed := v.rb;
        else
          vMed := vRa;
        end if;
      else
        vMed := vRa + v.rbRc;
      end if;

      if (v.run = '1') then
        if (v.riType = '1') then
          v.pbase := vRa;
        else
          v.pbase := v.rb;
        end if;
        v.cs   := (others => '0');
        v.sign := not v.riType and bool2sl(not vRaLeRb);
      else
        v.pbase := vMed;
      end if;

      vSt.inRun := v.run and v.hit and not v.eol;
      if (v.run = '0') then
        vSt.lastValid := '1';
        vSt.lastCtx   := v.ctx;
      end if;
      if (sValid(0) = '1') then
        v.st := vSt;
      else
        v.st := r(4).st;
      end if;
      r(1) <= v;

      -- S2: forward C, Px, error magnitude
      v        := r(1);
      v.lastC  := r(5).lastC;
      v.lastCn := r(5).lastCn;
      if (v.match = '1') then
        v.c := v.lastC;
        if (v.sign = '1') then
          vCs := resize(v.lastCn, W);
        else
          vCs := resize(v.lastC, W);
        end if;
      else
        vCs := resize(v.cs, W);
      end if;

      vDiff := v.ix - v.pbase - vCs;
      vMA   := v.ixN - v.pbase - vCs;
      vMB   := v.pbase + vCs - v.ixM;
      if (v.pbase + vCs < 0) then
        v.px     := (others => '0');
        vM       := v.ixN;
        v.dirPos := '1';
      elsif (v.pbase + vCs - sMaxP1 >= 0) then
        v.px     := sMaxVal;
        vM       := v.magMax;
        v.dirPos := '0';
      else
        v.px := v.pbase + vCs;
        if (vDiff >= 0) then
          vM       := vMA;
          v.dirPos := '1';
        else
          vM       := vMB;
          v.dirPos := '0';
        end if;
      end if;
      assert sValid(1) /= '1' or (vM >= 0 and vM <= sMaxVal + sNear)
        report "nl_loop: |Errval|+NEAR outside [0, MAXVAL+NEAR]" severity failure;
      v.m := unsigned(vM(BITNESS downto 0));
      v.ePos := v.dirPos xor v.sign;
      r(2) <= v;

      -- S3: q = m / (2*NEAR+1), forward B/N, C +-1
      v   := r(2);
      v.q := resize(shift_right(v.m * sRecip, SH), BITNESS + 1);
      vDd := signed(resize(sD, DW));
      if (v.dirPos = '1') then
        v.dS := vDd;
      else
        v.dS := -vDd;
      end if;
      if (v.ePos = '1') then
        v.dE := vDd;
      else
        v.dE := -vDd;
      end if;
      v.pxOv := v.px - sMaxP1;

      v.cP1 := v.c;
      v.cM1 := v.c;
      if (v.c < CO_MAX_C) then
        v.cP1 := v.c + 1;
      end if;
      if (v.c > CO_MIN_C) then
        v.cM1 := v.c - 1;
      end if;
      v.cNeg   := -resize(v.c, CW);
      v.cP1Neg := -resize(v.cP1, CW);
      v.cM1Neg := -resize(v.cM1, CW);

      v.lastB := r(6).lastB;
      v.lastN := r(6).lastN;
      if (v.match = '1') then
        v.b := v.lastB;
        v.n := v.lastN;
      end if;
      v.nRes := bool2sl(v.n = RESET);
      vN     := v.n;
      if (v.nRes = '1') then
        vN := shift_right(vN, 1);
      end if;
      v.n1 := vN + 1;
      r(3) <= v;

      -- S4: Rx, Errval*(2*NEAR+1), modulo flags
      v      := r(3);
      vQs    := signed('0' & v.q);
      vRxRaw := v.px + resize(vQs * v.dS, W);
      vOvT   := v.pxOv + resize(vQs * v.dS, W);
      v.qdE  := resize(vQs * v.dE, W);
      if (vRxRaw < 0) then
        v.rx := (others => '0');
      elsif (vOvT >= 0) then
        v.rx := unsigned(sMaxVal(BITNESS - 1 downto 0));
      else
        v.rx := unsigned(vRxRaw(BITNESS - 1 downto 0));
      end if;
      if (v.run = '1' and v.hit = '1') then
        v.rx := v.ra;
      end if;
      if (sValid(3) = '1') then
        v.st.ra := v.rx;
      end if;

      v.wrapDn := v.ePos and bool2sl(v.q >= sHalf);
      v.wrapUp := not v.ePos and bool2sl(v.q > sRangeMHalf);
      if (v.wrapDn = '1') then
        v.x := resize(v.b, W) - sRangeD;
      elsif (v.wrapUp = '1') then
        v.x := resize(v.b, W) + sRangeD;
      else
        v.x := resize(v.b, W);
      end if;
      r(4) <= v;

      -- S5: B + Errval*(2*NEAR+1), C update, Errval
      v    := r(4);
      vSum := v.x + v.qdE;
      if (v.nRes = '1') then
        v.t := shift_right(vSum, 1);
      else
        v.t := vSum;
      end if;
      vN1   := signed(resize(v.n1, W));
      v.dec := bool2sl(v.t + vN1 <= 0);
      v.inc := bool2sl(v.t > 0);
      if (v.dec = '1') then
        v.c1 := v.cM1;
      elsif (v.inc = '1') then
        v.c1 := v.cP1;
      else
        v.c1 := v.c;
      end if;
      if (sValid(4) = '1' and v.run = '0') then
        v.lastC := v.c1;
        if (v.dec = '1') then
          v.lastCn := v.cM1Neg;
        elsif (v.inc = '1') then
          v.lastCn := v.cP1Neg;
        else
          v.lastCn := v.cNeg;
        end if;
      end if;

      if (v.ePos = '1') then
        if (v.wrapDn = '1') then
          v.errq := resize(signed('0' & v.q), BITNESS + 1) - sRange;
        else
          v.errq := resize(signed('0' & v.q), BITNESS + 1);
        end if;
      else
        if (v.wrapUp = '1') then
          v.errq := sRange - resize(signed('0' & v.q), BITNESS + 1);
        else
          v.errq := -resize(signed('0' & v.q), BITNESS + 1);
        end if;
      end if;
      r(5) <= v;

      -- S6: B update
      v   := r(5);
      vN1 := signed(resize(v.n1, W));
      if (v.dec = '1') then
        vB1 := v.t + vN1;
        if (vB1 + vN1 <= 0) then
          vB1 := 1 - vN1;
        end if;
      elsif (v.inc = '1') then
        vB1 := v.t - vN1;
        if (vB1 > 0) then
          vB1 := (others => '0');
        end if;
      else
        vB1 := v.t;
      end if;
      assert sValid(5) /= '1' or (vB1 <= 0 and vB1 + vN1 > 0)
        report "nl_loop: updated B outside [1-N, 0]" severity failure;
      v.b1 := resize(vB1, BW);
      if (sValid(5) = '1' and v.run = '0') then
        v.lastB := v.b1;
        v.lastN := v.n1;
      end if;
      r(6) <= v;
    end if;

  end process pipe_proc;

  oValid  <= sValid(STAGES);
  oMode   <= "00" when r(STAGES).run = '0' else
             "01" when r(STAGES).hit = '1' else
             "10";
  oRiType <= r(STAGES).riType;
  oRx     <= r(STAGES).rx;
  oErrval <= r(STAGES).errq;
  oCtx    <= r(STAGES).ctx;
  oCq     <= r(STAGES).c1;
  oBq     <= r(STAGES).b1;
  oNq     <= r(STAGES).n1;
  oBpre   <= r(STAGES).b;
  oNpre   <= r(STAGES).n;
  oNRes   <= r(STAGES).nRes;
  oPx     <= unsigned(r(STAGES).px(BITNESS - 1 downto 0));

  -- Valid slots only: idle slots carry junk, U before the first pixel
  range_assert_proc : process (iClk) is
    variable vD   : integer;
    variable vSum : integer;
  begin
    if rising_edge(iClk) then
      vD := to_integer(sD);
      if (sValid(1) = '1') then
        assert r(1).ra <= unsigned(sMaxVal(BITNESS - 1 downto 0))
          report "nl_loop: Ra above MAXVAL" severity failure;
        assert r(1).pbase >= 0 and r(1).pbase <= sMaxVal
          report "nl_loop: MED / run Px outside [0, MAXVAL]" severity failure;
      end if;
      if (sValid(2) = '1') then
        assert r(2).px >= 0 and r(2).px <= sMaxVal
          report "nl_loop: Px outside [0, MAXVAL]" severity failure;
      end if;
      if (sValid(3) = '1') then
        assert to_integer(r(3).q) * vD <= to_integer(r(3).m) and
               to_integer(r(3).m) < (to_integer(r(3).q) + 1) * vD
          report "nl_loop: reciprocal quotient q /= m / (2*NEAR+1)" severity failure;
        assert r(3).n >= 1 and r(3).n <= RESET and r(3).n1 >= 2 and r(3).n1 <= RESET
          report "nl_loop: N outside [1, RESET]" severity failure;
        assert r(3).b <= 0 and to_integer(r(3).b) + to_integer(r(3).n) > 0
          report "nl_loop: B outside [1-N, 0]" severity failure;
      end if;
      if (sValid(4) = '1') then
        assert r(4).rx <= unsigned(sMaxVal(BITNESS - 1 downto 0))
          report "nl_loop: Rx above MAXVAL" severity failure;
      end if;
      if (sValid(5) = '1') then
        assert r(5).errq >= -signed(resize(sRangeMHalf, BITNESS + 1)) and
               r(5).errq < signed(resize(sHalf, BITNESS + 1))
          report "nl_loop: Errval outside the A.9 range" severity failure;
        vSum := to_integer(r(5).b) + to_integer(r(5).errq) * vD;
        if (r(5).nRes = '1') then
          vSum := (vSum - (vSum mod 2)) / 2;
        end if;
        assert to_integer(r(5).t) = vSum
          report "nl_loop: B + Errval*(2*NEAR+1) does not match the modulo-reduced Errval" severity failure;
      end if;
    end if;
  end process range_assert_proc;

  postponed assert oValid /= '1' or is_x(oRx) or oRx <= iMaxVal
    report "nl_loop: oRx above MAXVAL" severity error;
  postponed assert oValid /= '1' or is_x(oErrval) or
                   (oErrval >= -signed(resize(sRangeMHalf, BITNESS + 1)) and oErrval < signed(resize(sHalf, BITNESS + 1)))
    report "nl_loop: oErrval outside the A.9 range" severity error;
  postponed assert oValid /= '1' or oMode /= "00" or is_x(oNq) or (oNq >= 2 and oNq <= RESET)
    report "nl_loop: oNq outside [2, RESET]" severity error;
  postponed assert oValid /= '1' or oMode /= "00" or is_x(oBq) or is_x(oNq) or
                   (oBq <= 0 and to_integer(oBq) + to_integer(oNq) > 0)
    report "nl_loop: oBq outside [1-N, 0]" severity error;

end architecture behavioral;
