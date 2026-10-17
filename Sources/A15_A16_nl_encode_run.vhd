-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- a15_a16_nl_encode_run: a15_a16_encode_run with the run state as ports, so one instance serves all
-- lanes; changed lines marked NL
--
-- sNextBound is the run count of the next A.15 '1' bit: 2^J[RUNindex] at run start, then
-- + 2^J[RUNindex + 1] per hit.

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use work.openjls_pkg.all;

entity a15_a16_nl_encode_run is
  generic (
    BITNESS       : natural := CO_BITNESS_STD;
    RUN_CNT_WIDTH : natural := 16
  );
  port (
    iRst          : in    std_logic;
    -- NL: run state of the current lane
    iRunIndex     : in    unsigned(4 downto 0);                 -- NL
    iNextBound    : in    unsigned(RUN_CNT_WIDTH downto 0);     -- NL
    iInRun        : in    std_logic;                            -- NL
    oRunIndexNext : out   unsigned(4 downto 0);                 -- NL
    oNextBoundNext : out  unsigned(RUN_CNT_WIDTH downto 0);     -- NL

    -- From A14, unregistered
    iRunCnt       : in    unsigned(RUN_CNT_WIDTH - 1 downto 0);
    iRunHit       : in    std_logic;
    iRunContinue  : in    std_logic;

    iModeIsRun    : in    std_logic;

    -- Valid on the break cycle
    iIx           : in    unsigned(BITNESS - 1 downto 0);
    iRa           : in    unsigned(BITNESS - 1 downto 0);
    iRb           : in    unsigned(BITNESS - 1 downto 0);

    -- A.15 '1' bits and the A.16 break prefix; can fire together with oRiValid
    oRawValid     : out   std_logic;
    oRawSuffixLen : out   unsigned(4 downto 0);
    oRawSuffixVal : out   unsigned(RUN_CNT_WIDTH - 1 downto 0);

    -- RI token on break; oRiRunIndex is RUNindex before the A.16 decrement
    oRiValid      : out   std_logic;
    oRiIx         : out   unsigned(BITNESS - 1 downto 0);
    oRiRa         : out   unsigned(BITNESS - 1 downto 0);
    oRiRb         : out   unsigned(BITNESS - 1 downto 0);
    oRiRunIndex   : out   unsigned(4 downto 0);
    oInRunNext    : out   std_logic
  );
end entity a15_a16_nl_encode_run;

architecture behavioral of a15_a16_nl_encode_run is

  constant C_BOUND_WIDTH : natural := RUN_CNT_WIDTH + 1;

  signal sRunIndex       : unsigned(4 downto 0);                       -- NL [0, 31], full type: saturated at 31
  signal sNextBound      : unsigned(C_BOUND_WIDTH - 1 downto 0);       -- NL [1, 2^RUN_CNT_WIDTH + 2^15]
  signal sInRun          : std_logic;                                  -- NL

  signal sRunIndexNext   : unsigned(4 downto 0);                       -- [0, 31], full type
  signal sNextBoundNext  : unsigned(C_BOUND_WIDTH - 1 downto 0);       -- [1, 2^RUN_CNT_WIDTH + 2^15]
  signal sInRunNext      : std_logic;

begin

  -- NL: no clock here; the state owner checks the clear on iEoi

  sRunIndex  <= iRunIndex;                                             -- NL
  sNextBound <= iNextBound;                                            -- NL
  sInRun     <= iInRun;                                                -- NL

  p_comb : process (sRunIndex, sNextBound, sInRun,
                    iRunCnt, iRunHit, iRunContinue, iModeIsRun, iRst,          -- NL: no iEoi
                    iIx, iRa, iRb) is

    variable vJ        : natural;                                      -- [0 .. 15] (J table)
    variable vJNext    : natural;                                      -- [0 .. 15]
    variable vStep     : unsigned(C_BOUND_WIDTH - 1 downto 0);         -- [1 .. 2^15]
    variable vStepNext : unsigned(C_BOUND_WIDTH - 1 downto 0);         -- [1 .. 2^15]
    variable vBoundHit : boolean;
    variable vNewIndex : unsigned(4 downto 0);                         -- [0, 31], full type

  begin

    oRawValid     <= '0';
    oRawSuffixLen <= to_unsigned(1, 5);
    oRawSuffixVal <= to_unsigned(1, RUN_CNT_WIDTH);
    oRiValid      <= '0';
    oRiIx         <= iIx;
    oRiRa         <= iRa;
    oRiRb         <= iRb;
    oRiRunIndex   <= sRunIndex;

    sRunIndexNext  <= sRunIndex;
    sNextBoundNext <= sNextBound;
    sInRunNext     <= sInRun;

    if (iModeIsRun = '1' and iRst = '0') then
      vJ    := CO_J_TABLE(to_integer(sRunIndex));
      vStep := shift_left(to_unsigned(1, C_BOUND_WIDTH), vJ);
      if (sRunIndex < 31) then
        vJNext := CO_J_TABLE(to_integer(sRunIndex) + 1);
      else
        vJNext := CO_J_TABLE(31);
      end if;
      vStepNext := shift_left(to_unsigned(1, C_BOUND_WIDTH), vJNext);

      if (iRunHit = '1') then
        -- A.15: match
        vBoundHit := resize(iRunCnt, C_BOUND_WIDTH) = sNextBound;
        vNewIndex := sRunIndex;

        if (vBoundHit) then
          oRawValid <= '1';
          if (sRunIndex < 31) then
            vNewIndex := sRunIndex + 1;
          end if;
        end if;

        sInRunNext <= '1';

        if (iRunContinue = '0') then
          -- A.16 end of line
          sInRunNext <= '0';
          if (not vBoundHit and iRunCnt > 0) then
            oRawValid <= '1';
          end if;
          sRunIndexNext  <= vNewIndex;
          sNextBoundNext <= shift_left(to_unsigned(1, C_BOUND_WIDTH),
                                       CO_J_TABLE(to_integer(vNewIndex)));
        else
          sRunIndexNext <= vNewIndex;
          if (vBoundHit) then
            sNextBoundNext <= sNextBound + vStepNext;
          end if;
        end if;
      else
        -- A.16: break, including on the first run-mode pixel (RUNcnt = 0 gives a single '0').
        -- Suffix = pixels since the last boundary.
        oRawValid     <= '1';
        oRawSuffixLen <= to_unsigned(vJ + 1, 5);
        oRawSuffixVal <= iRunCnt - resize(sNextBound - vStep, RUN_CNT_WIDTH);
        oRiValid      <= '1';
        oRiIx         <= iIx;
        oRiRa         <= iRa;
        oRiRb         <= iRb;

        if (sRunIndex > 0) then
          vNewIndex := sRunIndex - 1;
        else
          vNewIndex := (others => '0');
        end if;

        sInRunNext     <= '0';
        sRunIndexNext  <= vNewIndex;
        sNextBoundNext <= shift_left(to_unsigned(1, C_BOUND_WIDTH),
                                     CO_J_TABLE(to_integer(vNewIndex)));
      end if;
    end if;

  end process p_comb;

  -- A.16 residual in [0 .. 2^J - 1]
  postponed assert (iModeIsRun /= '1' or iRst /= '0' or iRunHit /= '0' or
                    Is_X(iRunCnt) or Is_X(iNextBound) or Is_X(iRunIndex) or
                    (iNextBound >= shift_left(to_unsigned(1, C_BOUND_WIDTH), CO_J_TABLE(to_integer(iRunIndex))) and
                     resize(iRunCnt, C_BOUND_WIDTH) + shift_left(to_unsigned(1, C_BOUND_WIDTH),
                                                                 CO_J_TABLE(to_integer(iRunIndex))) >= iNextBound and
                     resize(iRunCnt, C_BOUND_WIDTH) < iNextBound))
    report "a15_a16_nl: A.16 residual out of [0 .. 2^J[RUNindex]-1]" severity failure;
  postponed assert is_x(oRawSuffixLen) or (oRawSuffixLen >= 1 and oRawSuffixLen <= 16)
    report "a15_a16_nl: raw suffix length out of [1, 16]" severity error;
  postponed assert is_x(sNextBoundNext) or sNextBoundNext /= 0
    report "a15_a16_nl: next bound wrapped" severity error;

  oInRunNext     <= sInRunNext;
  oRunIndexNext  <= sRunIndexNext;                                     -- NL
  oNextBoundNext <= sNextBoundNext;                                    -- NL

end architecture behavioral;
