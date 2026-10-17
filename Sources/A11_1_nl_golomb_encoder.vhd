-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- a11_1_nl_golomb_encoder: a11_1_golomb_encoder with qbpp as a port (runtime NEAR); changed lines marked NL
--
-- The bit packer masks the suffix to oSuffixLen.

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use work.openjls_pkg.all;

entity a11_1_nl_golomb_encoder is
  generic (
    K_WIDTH                : natural := CO_K_WIDTH_STD;
    LIMIT                  : natural := CO_LIMIT_STD;
    UNARY_WIDTH            : natural := CO_UNARY_WIDTH_STD;
    SUFFIX_WIDTH           : natural := CO_SUFFIX_WIDTH_STD;
    SUFFIXLEN_WIDTH        : natural := CO_SUFFIXLEN_WIDTH_STD;
    MAPPED_ERROR_VAL_WIDTH : natural := CO_MAPPED_ERROR_VAL_WIDTH_STD
  );
  port (
    iValid                 : in    std_logic := '1';                            -- NL: gates the assertions only
    iK                     : in    unsigned (K_WIDTH - 1 downto 0);
    iQbpp                  : in    unsigned (4 downto 0);                       -- NL
    iMappedErrorVal        : in    unsigned (MAPPED_ERROR_VAL_WIDTH - 1 downto 0);
    -- iRunIndex: RUNindex before the A.16 decrement, used only when iRiMode = '1'
    iRiMode                : in    std_logic;
    iRunIndex              : in    unsigned (4 downto 0);
    oUnaryZeros            : out   unsigned (UNARY_WIDTH - 1 downto 0);
    oSuffixLen             : out   unsigned (SUFFIXLEN_WIDTH - 1 downto 0);
    oSuffixVal             : out   unsigned (SUFFIX_WIDTH - 1 downto 0)
  );
end entity a11_1_nl_golomb_encoder;

architecture behavioral of a11_1_nl_golomb_encoder is

begin

  -- LG(k, L) with L = LIMIT (regular) or glimit = LIMIT - J[RUNindex] - 1 (RI, A.22.1)
  p_golomb_encode : process (iValid, iK, iQbpp, iMappedErrorVal, iRiMode, iRunIndex) is

    variable vKInt           : integer;                                         -- [0 .. min(SUFFIX_WIDTH, MAPPED_ERROR_VAL_WIDTH)]
    variable vHighOrder      : unsigned(iMappedErrorVal'range);                 -- [0 .. 2^MAPPED-1] >> k
    variable vLowOrder       : unsigned(iMappedErrorVal'range);                 -- [0 .. 2^k-1]
    variable vMappedErrorDec : unsigned(iMappedErrorVal'range);                 -- [0 .. 2^MAPPED-2], escape implies MErrval >= 1
    variable vUnaryZeros     : unsigned(oUnaryZeros'range);                    -- [0 .. vThreshold] <= LIMIT-2 (QBPP >= 1)
    variable vSuffixLen      : unsigned(oSuffixLen'range);                     -- [0 .. max(k, QBPP)] <= SUFFIX_WIDTH
    variable vIsEscape       : boolean;
    variable vSuffixVal      : unsigned(oSuffixVal'range);                     -- full type
    variable vJ              : natural;                                         -- [0 .. 15] (J table)
    variable vThreshold      : integer;                                         -- [1 .. LIMIT-1]: LIMIT-J-QBPP-2 >= 1 for QBPP <= BITNESS
    variable vQbpp           : natural;                                         -- NL [1 .. 16] (A.2.1, RANGE >= 2)

  begin

    vQbpp := to_integer(iQbpp);                                                   -- NL

    if (iRiMode = '1') then
      vJ         := CO_J_TABLE(to_integer(iRunIndex));
      vThreshold := LIMIT - vJ - vQbpp - 2;                                     -- NL
    else
      vThreshold := LIMIT - vQbpp - 1;                                          -- NL
    end if;

    -- iQbpp is U until nl_config's first image
    assert iValid /= '1' or Is_X(iQbpp) or (vQbpp >= 1 and vQbpp <= SUFFIX_WIDTH and vQbpp <= MAPPED_ERROR_VAL_WIDTH)
      report "a11_1_nl: qbpp out of [1 .. min(SUFFIX_WIDTH, MAPPED_ERROR_VAL_WIDTH)]"
      severity failure;
    assert iValid /= '1' or Is_X(iQbpp) or (vThreshold >= 1 and vThreshold < 2 ** oUnaryZeros'length)
      report "a11_1_nl: escape threshold out of [1 .. 2^UNARY_WIDTH-1]"
      severity failure;
    assert iValid /= '1' or Is_X(iK) or (to_integer(iK) <= SUFFIX_WIDTH and to_integer(iK) <= MAPPED_ERROR_VAL_WIDTH)
      report "a11_1_nl: k out of [0 .. min(SUFFIX_WIDTH, MAPPED_ERROR_VAL_WIDTH)]"
      severity failure;

    vKInt      := to_integer(iK);
    vHighOrder := shift_right(iMappedErrorVal, vKInt);
    vLowOrder  := iMappedErrorVal - shift_left(vHighOrder, vKInt);

    vIsEscape := (vHighOrder >= vThreshold);

    if (not vIsEscape) then
      vUnaryZeros := resize(vHighOrder, vUnaryZeros'length);
      vSuffixLen  := resize(iK, vSuffixLen'length);

      vSuffixVal := resize(vLowOrder, vSuffixVal'length);
    else
      assert (iMappedErrorVal /= 0)
        report "LG(k,LIMIT) escape with MErrval=0 is impossible per T.87/14495-1"
        severity failure;

      vUnaryZeros := to_unsigned(vThreshold, vUnaryZeros'length);
      vSuffixLen  := to_unsigned(vQbpp, vSuffixLen'length);                     -- NL

      vMappedErrorDec := iMappedErrorVal - 1;

      vSuffixVal := resize(vMappedErrorDec, vSuffixVal'length);                  -- NL
    end if;

    oUnaryZeros <= vUnaryZeros;
    oSuffixLen  <= vSuffixLen;
    oSuffixVal  <= vSuffixVal;

  end process p_golomb_encode;

  postponed assert iValid /= '1' or is_x(oUnaryZeros) or oUnaryZeros <= LIMIT - 2
    report "a11_1_nl: unary prefix above LIMIT-2" severity error;
  postponed assert iValid /= '1' or is_x(oSuffixLen) or oSuffixLen <= SUFFIX_WIDTH
    report "a11_1_nl: suffix length above SUFFIX_WIDTH" severity error;

end architecture behavioral;
