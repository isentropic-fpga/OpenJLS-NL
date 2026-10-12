-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--

----------------------------------------------------------------------------------
-- Engineer:    Vitor Mendes Camilo
--
-- Module Name: nl_loop_wrap
-- Description: Flat-port wrapper of nl_loop for out-of-context fmax sweeps.
--
----------------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

library work;
  use work.nl_pkg.all;

entity nl_loop_wrap is
  generic (
    BITNESS : natural := 12
  );
  port (
    iClk      : in    std_logic;
    iNear     : in    std_logic_vector(7 downto 0);
    iMaxVal   : in    std_logic_vector(BITNESS - 1 downto 0);
    iRecip    : in    std_logic_vector(BITNESS + 10 downto 0);
    iRange    : in    std_logic_vector(BITNESS downto 0);
    iRangeD   : in    std_logic_vector(BITNESS + 1 downto 0);
    iValid    : in    std_logic;
    iFirstPix : in    std_logic;
    iFirstCol : in    std_logic;
    iEol      : in    std_logic;
    iIx       : in    std_logic_vector(BITNESS - 1 downto 0);
    iRb       : in    std_logic_vector(BITNESS - 1 downto 0);
    iRc       : in    std_logic_vector(BITNESS - 1 downto 0);
    iQ12Zero  : in    std_logic;
    iQ3Bound  : in    std_logic_vector(8 * (BITNESS + 2) - 1 downto 0);
    iCandCtx  : in    std_logic_vector(9 * CO_CTX_WIDTH - 1 downto 0);
    iCandSign : in    std_logic_vector(0 to 8);
    iCandC    : in    std_logic_vector(9 * CO_C_WIDTH - 1 downto 0);
    iCandB    : in    std_logic_vector(9 * 7 - 1 downto 0);
    iCandN    : in    std_logic_vector(9 * 7 - 1 downto 0);
    oValid    : out   std_logic;
    oMode     : out   std_logic_vector(1 downto 0);
    oRiType   : out   std_logic;
    oRx       : out   std_logic_vector(BITNESS - 1 downto 0);
    oErrval   : out   std_logic_vector(BITNESS downto 0);
    oCtx      : out   std_logic_vector(CO_CTX_WIDTH - 1 downto 0);
    oCq       : out   std_logic_vector(CO_C_WIDTH - 1 downto 0);
    oBq       : out   std_logic_vector(6 downto 0);
    oNq       : out   std_logic_vector(6 downto 0)
  );
end entity nl_loop_wrap;

architecture behavioral of nl_loop_wrap is

  constant BOUND_W : natural := BITNESS + 2;

  signal sQ3Bound  : t_signed_array(0 to 7)(BOUND_W - 1 downto 0);
  signal sCandCtx  : t_unsigned_array(0 to 8)(CO_CTX_WIDTH - 1 downto 0);
  signal sCandC    : t_signed_array(0 to 8)(CO_C_WIDTH - 1 downto 0);
  signal sCandB    : t_signed_array(0 to 8)(6 downto 0);
  signal sCandN    : t_unsigned_array(0 to 8)(6 downto 0);
  signal sRx       : unsigned(BITNESS - 1 downto 0);
  signal sErrval   : signed(BITNESS downto 0);
  signal sCtx      : unsigned(CO_CTX_WIDTH - 1 downto 0);
  signal sCq       : signed(CO_C_WIDTH - 1 downto 0);
  signal sBq       : signed(6 downto 0);
  signal sNq       : unsigned(6 downto 0);

begin

  gen_bound : for i in 0 to 7 generate
    sQ3Bound(i) <= signed(iQ3Bound((i + 1) * BOUND_W - 1 downto i * BOUND_W));
  end generate gen_bound;

  gen_cand : for i in 0 to 8 generate
    sCandCtx(i) <= unsigned(iCandCtx((i + 1) * CO_CTX_WIDTH - 1 downto i * CO_CTX_WIDTH));
    sCandC(i)   <= signed(iCandC((i + 1) * CO_C_WIDTH - 1 downto i * CO_C_WIDTH));
    sCandB(i)   <= signed(iCandB((i + 1) * 7 - 1 downto i * 7));
    sCandN(i)   <= unsigned(iCandN((i + 1) * 7 - 1 downto i * 7));
  end generate gen_cand;

  u_loop : entity work.nl_loop
    generic map (
      BITNESS => BITNESS
    )
    port map (
      iClk      => iClk,
      iNear     => unsigned(iNear),
      iMaxVal   => unsigned(iMaxVal),
      iRecip    => unsigned(iRecip),
      iRange    => unsigned(iRange),
      iRangeD   => unsigned(iRangeD),
      iValid    => iValid,
      iFirstPix => iFirstPix,
      iFirstCol => iFirstCol,
      iEol      => iEol,
      iIx       => unsigned(iIx),
      iRb       => unsigned(iRb),
      iRc       => unsigned(iRc),
      iQ12Zero  => iQ12Zero,
      iQ3Bound  => sQ3Bound,
      iCandCtx  => sCandCtx,
      iCandSign => iCandSign,
      iCandC    => sCandC,
      iCandB    => sCandB,
      iCandN    => sCandN,
      oValid    => oValid,
      oMode     => oMode,
      oRiType   => oRiType,
      oRx       => sRx,
      oErrval   => sErrval,
      oCtx      => sCtx,
      oCq       => sCq,
      oBq       => sBq,
      oNq       => sNq
    );

  oRx     <= std_logic_vector(sRx);
  oErrval <= std_logic_vector(sErrval);
  oCtx    <= std_logic_vector(sCtx);
  oCq     <= std_logic_vector(sCq);
  oBq     <= std_logic_vector(sBq);
  oNq     <= std_logic_vector(sNq);

end architecture behavioral;
