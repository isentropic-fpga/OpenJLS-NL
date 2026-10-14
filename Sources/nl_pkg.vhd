-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--

----------------------------------------------------------------------------------
-- Engineer:    Vitor Mendes Camilo
--
-- Module Name: nl_pkg
-- Description: Shared types for the near-lossless core.
--
----------------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

package nl_pkg is

  type t_signed_array is array (natural range <>) of signed;
  type t_unsigned_array is array (natural range <>) of unsigned;
  type t_slv_array is array (natural range <>) of std_logic_vector;

  constant CO_C_WIDTH   : natural := 8;
  constant CO_CTX_WIDTH : natural := 9;   -- 0..366
  constant CO_D_WIDTH   : natural := 9;   -- 2*NEAR+1 <= 511
  constant CO_MIN_C     : integer := -128;
  constant CO_MAX_C     : integer := 127;

end package nl_pkg;
