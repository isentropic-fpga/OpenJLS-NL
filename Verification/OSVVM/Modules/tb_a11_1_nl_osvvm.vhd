-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- tb_a11_1_nl_osvvm: a11_1_nl_golomb_encoder, LG(k, L) with qbpp as a port
--
-- L = LIMIT (regular, A.5.3) or glimit = LIMIT - J[RUNindex] - 1 (RI, A.22.1). The suffix is not
-- masked to qbpp (the bit packer masks it), so only the low oSuffixLen bits are compared.
-- Every qbpp of a legal NEAR is swept over both modes, every RUNindex, every k, the escape threshold
-- +-1 and the MErrval extremes, then random.
--
-- Guidelines:
--   G2.Clamp    N/A: combinational, no accumulator
--   G3.Stim     G2.MinMax node (MErrval MSB set)
--   G4.Config   G4.Coding qbpp x riMode x escape (NEAR 0 / max)
--   G4 other    N/A: combinational, single datapath
--   G5          N/A: combinational

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use work.openjls_pkg.all;
  use work.olo_base_pkg_math.log2ceil;

library osvvm;
  context osvvm.OsvvmContext;

library tb_support;
  use tb_support.tb_support_pkg.all;

entity tb_a11_1_nl_osvvm is
  generic (BITNESS : natural range 8 to 16 := 12);
end entity tb_a11_1_nl_osvvm;

architecture sim of tb_a11_1_nl_osvvm is

  -- Widths as nl_backend instantiates the encoder.
  constant LIMIT       : natural := 2 * (BITNESS + maximum(8, BITNESS));
  constant AWID        : natural := BITNESS + 7;
  constant K_WIDTH     : natural := log2ceil(AWID + 1);
  constant UNARY_W     : natural := log2ceil(LIMIT);
  constant SUFFIX_W    : natural := AWID;
  constant SUFFIXLEN_W : natural := 5;
  constant MAPPED_W    : natural := BITNESS + 2;

  constant MAXVAL      : natural := 2 ** BITNESS - 1;
  constant NEAR_MAX    : natural := minimum(255, MAXVAL / 2);
  constant K_HI        : integer := minimum(SUFFIX_W, MAPPED_W);
  constant MERR_MAX    : integer := (2 ** MAPPED_W) - 1;

  -- T.87 A.2.1 / C.2.4.1.1.
  function qbpp_of (near : natural) return natural is
    constant RANGE_V : natural := (MAXVAL + 2 * near) / (2 * near + 1) + 1;
  begin
    return log2ceil(RANGE_V);
  end function qbpp_of;

  constant QBPP_LO     : natural := qbpp_of(NEAR_MAX);
  constant QBPP_HI     : natural := qbpp_of(0);

  signal sK         : unsigned(K_WIDTH - 1 downto 0);
  signal sQbpp      : unsigned(4 downto 0);
  signal sMerr      : unsigned(MAPPED_W - 1 downto 0);
  signal sRiMode    : std_logic;
  signal sRunIndex  : unsigned(4 downto 0);
  signal sUnary     : unsigned(UNARY_W - 1 downto 0);
  signal sSuffixLen : unsigned(SUFFIXLEN_W - 1 downto 0);
  signal sSuffixVal : unsigned(SUFFIX_W - 1 downto 0);

  -- L - qbpp - 1 escape threshold for the active mode.
  function threshold_of (
    qbpp   : natural;
    riMode : std_logic;
    runIdx : integer
  ) return integer is
  begin

    if (riMode = '1') then
      -- glimit = LIMIT - J - 1; threshold = glimit - qbpp - 1.
      return LIMIT - TB_J_TABLE(runIdx) - qbpp - 2;
    else
      return LIMIT - qbpp - 1;
    end if;

  end function threshold_of;

begin

  dut : entity work.a11_1_nl_golomb_encoder(behavioral)
    generic map (
      K_WIDTH                => K_WIDTH,
      LIMIT                  => LIMIT,
      UNARY_WIDTH            => UNARY_W,
      SUFFIX_WIDTH           => SUFFIX_W,
      SUFFIXLEN_WIDTH        => SUFFIXLEN_W,
      MAPPED_ERROR_VAL_WIDTH => MAPPED_W
    )
    port map (
      iK              => sK,
      iQbpp           => sQbpp,
      iMappedErrorVal => sMerr,
      iRiMode         => sRiMode,
      iRunIndex       => sRunIndex,
      oUnaryZeros     => sUnary,
      oSuffixLen      => sSuffixLen,
      oSuffixVal      => sSuffixVal
    );

  stim : process is

    variable rv      : RandomPType;
    variable cov     : CoverageIDType;
    variable covK    : CoverageIDType;
    variable covJ    : CoverageIDType;
    variable covRng  : CoverageIDType;
    variable req     : AlertLogIDType;
    variable reqRi   : AlertLogIDType;
    variable reqSel  : AlertLogIDType;
    variable qbpp    : natural;
    variable merr    : integer;
    constant N_RAND  : natural := 20000;

    procedure drive_check (
      qb   : natural;
      kv   : integer;
      mv   : integer;
      ri   : std_logic;
      rix  : integer;
      msg  : string
    ) is

      variable thr     : integer;
      variable high    : integer;
      variable expUn   : integer;
      variable expLen  : integer;
      variable expVal  : integer;
      variable escape  : integer;
      variable sufLen  : integer;

    begin

      sK        <= to_unsigned(kv, K_WIDTH);
      sQbpp     <= to_unsigned(qb, 5);
      sMerr     <= to_unsigned(mv, MAPPED_W);
      sRiMode   <= ri;
      sRunIndex <= to_unsigned(rix, 5);
      wait for 1 ns;

      thr  := threshold_of(qb, ri, rix);
      high := mv / (2 ** kv);

      if (high < thr) then
        expUn  := high;
        expLen := kv;
        expVal := mv mod (2 ** kv);
        escape := 0;
      else
        expUn  := thr;
        expLen := qb;
        expVal := (mv - 1) mod (2 ** qb);
        escape := 1;
      end if;

      if (ri = '1') then
        reqSel := reqRi;
      else
        reqSel := req;
      end if;
      sufLen := checked_integer(sSuffixLen);
      AffirmIfEqual(reqSel, checked_integer(sUnary), expUn, msg & " unary");
      AffirmIfEqual(reqSel, sufLen, expLen, msg & " sufLen");
      if (sufLen = expLen) then
        AffirmIfEqual(reqSel, checked_integer(sSuffixVal mod (2 ** expLen)), expVal, msg & " sufVal");
      end if;

      ICover(cov, (qb, std_to_int(ri), escape));
      ICover(covK, kv);
      if (mv = 0) then
        ICover(covRng, 0);
      end if;
      if (mv = MERR_MAX) then
        ICover(covRng, 1);
      end if;
      if (mv >= 2 ** (MAPPED_W - 1)) then
        ICover(covRng, 2);
      end if;
      if (qb = QBPP_LO) then
        ICover(covRng, 3);
      end if;
      if (qb = QBPP_HI) then
        ICover(covRng, 4);
      end if;
      if (thr = threshold_of(QBPP_HI, '1', 31)) then
        ICover(covRng, 5);
      end if;
      if (ri = '1') then
        ICover(covJ, rix);
      end if;

    end procedure drive_check;

  begin

    SetAlertLogName("tb_a11_1_nl_osvvm");
    SetLogEnable(PASSED, FALSE);
    rv.InitSeed(rv'instance_name);
    -- Guaranteed by the sweep: every reachable qbpp x 32 RUNindex x 2
    -- suffix extremes x 3 fields (RI), and the same per k (regular).
    req   := GetReqID("T87.A11.1", (QBPP_HI - QBPP_LO + 1) * 32 * (K_HI + 1) * 2 * 3);
    reqRi := GetReqID("T87.A22.1", (QBPP_HI - QBPP_LO + 1) * 32 * 2 * 3);

    covK := NewID("G4.Coding k");
    SetFieldName(covK, "k");
    for kval in 0 to K_HI loop
      AddBins(covK, "k=" & to_string(kval), GenBin(kval));
    end loop;
    covJ := NewID("G4.Run RUNindex");
    SetFieldName(covJ, "RUNindex");
    for index in 0 to 31 loop
      AddBins(covJ, "RUNindex=" & to_string(index) & " J=" & to_string(TB_J_TABLE(index)), GenBin(index));
    end loop;
    covRng := NewID("G2.MinMax node");
    SetFieldName(covRng, "node");
    AddBins(covRng, "MErrval=0", GenBin(0));
    AddBins(covRng, "MErrval=max", GenBin(1));
    AddBins(covRng, "MErrval MSB set", GenBin(2));
    AddBins(covRng, "qbpp min (NEAR max)", GenBin(3));
    AddBins(covRng, "qbpp max (lossless)", GenBin(4));
    AddBins(covRng, "threshold min (RI, RUNindex 31, qbpp max)", GenBin(5));
    cov := NewID("G4.Coding qbpp x riMode x escape");
    SetFieldName(cov, "qbpp", "riMode", "escape");
    for qb in QBPP_LO to QBPP_HI loop
      for ri in 0 to 1 loop
        for esc in 0 to 1 loop
          AddCross(cov, "qbpp=" & to_string(qb) & " / riMode=" & to_string(ri) & " / escape=" & to_string(esc),
                   GenBin(qb), GenBin(ri), GenBin(esc));
        end loop;
      end loop;
    end loop;

    -- Every qbpp a legal NEAR can produce (qbpp is monotone in RANGE).
    for qb in QBPP_LO to QBPP_HI loop
      for ri in 0 to 1 loop
        for rix in 0 to 31 loop
          for kv in 0 to K_HI loop
            drive_check(qb, kv, 0, bool2bit(ri = 1), rix, "zero suffix");
            drive_check(qb, kv, MERR_MAX, bool2bit(ri = 1), rix, "maximum suffix");
            for delta in -1 to 1 loop
              merr := threshold_of(qb, bool2bit(ri = 1), rix) * (2 ** kv) + delta;
              if (merr >= 0 and merr <= MERR_MAX) then
                drive_check(qb, kv, merr, bool2bit(ri = 1), rix, "escape threshold");
              end if;
            end loop;
          end loop;
        end loop;
      end loop;
    end loop;

    -- Random: qbpp from a random legal NEAR.
    for i in 1 to N_RAND loop
      qbpp := qbpp_of(rv.RandInt(0, NEAR_MAX));
      drive_check(qbpp, rv.RandInt(0, K_HI), rv.RandInt(1, MERR_MAX), bool2bit(rv.RandInt(0, 1) = 1),
                  rv.RandInt(0, 31), "rand");
    end loop;

    WriteBin(covK);
    WriteBin(covJ);
    WriteBin(cov);
    WriteBin(covRng);
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covRng), "range coverage closed");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covK), "every supported k covered");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(covJ), "every J table entry covered");
    AffirmIf(GetAlertLogID("CoverageClosure"), IsCovered(cov), "qbpp x riMode x escape coverage closed");

    end_of_test("tb_a11_1_nl_osvvm");
    wait;

  end process stim;

end architecture sim;
