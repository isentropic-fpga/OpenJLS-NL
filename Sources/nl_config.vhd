-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--

----------------------------------------------------------------------------------
-- Engineer:    Vitor Mendes Camilo
--
-- Module Name: nl_config - Behavioral
--
-- Notes:
--              Per-image constants derived from NEAR (C.2.4.1.1, A.2.1).
--              iNear is latched while iRst = '1'. After reset a serial divider
--              computes RECIP and RANGE (~2*(BITNESS+11) cycles), then oDone rises.
--              MAXVAL = 2^BITNESS-1, default thresholds (BITNESS >= 8).
--
----------------------------------------------------------------------------------

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;

entity nl_config is
  generic (
    BITNESS : natural range 8 to 16 := 12
  );
  port (
    iClk    : in    std_logic;
    iRst    : in    std_logic;
    iNear   : in    unsigned(7 downto 0);
    oDone   : out   std_logic;
    oNear   : out   unsigned(7 downto 0);
    oRecip  : out   unsigned(BITNESS + 10 downto 0);              -- ceil(2^(BITNESS+10)/(2*NEAR+1))
    oRange  : out   unsigned(BITNESS downto 0);
    oRangeD : out   unsigned(BITNESS + 1 downto 0);               -- RANGE*(2*NEAR+1)
    oQbpp   : out   unsigned(4 downto 0);
    oAinit  : out   unsigned(BITNESS - 1 downto 0);
    oT1     : out   unsigned(BITNESS - 1 downto 0);
    oT2     : out   unsigned(BITNESS - 1 downto 0);
    oT3     : out   unsigned(BITNESS - 1 downto 0)
  );
end entity nl_config;

architecture behavioral of nl_config is

  constant MAXVAL  : natural := 2 ** BITNESS - 1;
  constant NMAX    : natural := minimum(255, MAXVAL / 2);
  constant FACTOR  : natural := (minimum(MAXVAL, 4095) + 128) / 256;
  constant DW      : natural := BITNESS + 11;

  type t_state is (S_DIV_RECIP, S_DIV_RANGE, S_POST, S_DONE);

  signal sState : t_state := S_DIV_RECIP;
  signal sNear  : unsigned(7 downto 0) := (others => '0');
  signal sD     : unsigned(DW - 1 downto 0);                  -- [1 .. 2*NMAX+1]
  signal sNum   : unsigned(DW - 1 downto 0);                  -- [MAXVAL .. 2^(BITNESS+10)+2*NMAX] < 2^DW
  signal sRem   : unsigned(DW - 1 downto 0);                  -- [0 .. sD-1]
  signal sQuo   : unsigned(DW - 1 downto 0);                  -- [0 .. 2^(BITNESS+10)] (RECIP), [0 .. MAXVAL] (RANGE-1)
  signal sCnt   : natural range 0 to DW;
  signal sRange   : unsigned(BITNESS downto 0);               -- [2 .. MAXVAL+1] (A.2.1, NEAR <= MAXVAL/2)
  signal sRangeM1 : unsigned(BITNESS downto 0);               -- [1 .. MAXVAL]

  function clamp_t (
    t, lo : natural
  ) return natural is
  begin

    if (t > MAXVAL or t < lo) then
      return lo;
    end if;
    return t;

  end function clamp_t;

begin

  proc : process (iClk) is

    variable vRem : unsigned(DW - 1 downto 0);                -- [0 .. 2*sD-1] before subtract, [0 .. sD-1] after
    variable vN   : natural;                                  -- [0 .. NMAX]
    variable vT1  : natural;                                  -- [NEAR+1 .. MAXVAL] (C.2.4.1.1 CLAMP)
    variable vT2  : natural;                                  -- [T1 .. MAXVAL]
    variable vT3  : natural;                                  -- [T2 .. MAXVAL]
    variable vQ   : natural;                                  -- [1 .. BITNESS] (qbpp, RANGE >= 2)

  begin

    if rising_edge(iClk) then
      if (iRst = '1') then
        if (iNear > NMAX) then
          sNear <= to_unsigned(NMAX, 8);
        else
          sNear <= iNear;
        end if;
        sState <= S_DIV_RECIP;
        sCnt   <= DW;
        sRem   <= (others => '0');
        oDone  <= '0';
      else
        vN := to_integer(sNear);
        sD <= to_unsigned(2 * vN + 1, DW);

        -- Restoring division sNum / sD, one quotient bit per cycle
        if (sState = S_DIV_RECIP or sState = S_DIV_RANGE) then
          if (sCnt = DW) then
            if (sState = S_DIV_RECIP) then
              sNum <= to_unsigned(2 ** (BITNESS + 10) + 2 * vN, DW);
            else
              sNum <= to_unsigned(MAXVAL + 2 * vN, DW);
            end if;
            sRem <= (others => '0');
            sCnt <= DW - 1;
          else
            vRem := sRem(DW - 2 downto 0) & sNum(sCnt);
            if (vRem >= sD) then
              vRem      := vRem - sD;
              sQuo(sCnt) <= '1';
            else
              sQuo(sCnt) <= '0';
            end if;
            sRem <= vRem;
            if (sCnt = 0) then
              sCnt <= DW;
              if (sState = S_DIV_RECIP) then
                sState <= S_DIV_RANGE;
              else
                sState <= S_POST;
              end if;
            else
              sCnt <= sCnt - 1;
            end if;
          end if;
        end if;

        if (sState = S_DIV_RANGE and sCnt = DW) then
          oRecip <= sQuo(BITNESS + 10 downto 0);
        end if;

        if (sState = S_POST) then
          sRange   <= sQuo(BITNESS downto 0) + 1;
          sRangeM1 <= sQuo(BITNESS downto 0);
          sState  <= S_DONE;
        end if;

        if (sState = S_DONE) then
          oRange  <= sRange;
          oRangeD <= resize(sRange * sD(8 downto 0), BITNESS + 2);
          vQ      := 0;
          for i in 0 to BITNESS loop
            if (sRangeM1(i) = '1') then
              vQ := i + 1;
            end if;
          end loop;
          -- Assertions ----------------------------------------------------------------
          assert (vN <= NMAX and vQ >= 1 and vQ <= BITNESS)
            report "nl_config: NEAR or qbpp out of range" severity failure;
          ------------------------------------------------------------------------------
          oQbpp   <= to_unsigned(vQ, 5);
          oAinit  <= to_unsigned(maximum(2, (to_integer(sRange) + 32) / 64), BITNESS);
          vT1     := clamp_t(FACTOR + 2 + 3 * vN, vN + 1);
          vT2     := clamp_t(4 * FACTOR + 3 + 5 * vN, vT1);
          vT3     := clamp_t(17 * FACTOR + 4 + 7 * vN, vT2);
          assert (vT1 >= vN + 1 and vT1 <= vT2 and vT2 <= vT3 and vT3 <= MAXVAL)
            report "nl_config: thresholds out of NEAR+1 <= T1 <= T2 <= T3 <= MAXVAL" severity failure;
          oT1     <= to_unsigned(vT1, BITNESS);
          oT2     <= to_unsigned(vT2, BITNESS);
          oT3     <= to_unsigned(vT3, BITNESS);
          oNear   <= sNear;
          oDone   <= '1';
        end if;
      end if;
    end if;

  end process proc;

  -- Assertions --------------------------------------------------------------------
  -- psl default clock is rising_edge(iClk);
  -- psl assert always (iRst = '1' -> next (oDone = '0')) report "nl_config: reset must clear oDone";
  -- psl assert never (sState = S_DONE and (sRange < 2 or sRange > MAXVAL + 1)) report "nl_config: RANGE out of [2 .. MAXVAL+1]";
  -- psl assert never (sState = S_DONE and sD > 2 * NMAX + 1) report "nl_config: 2*NEAR+1 out of range";
  -- psl assert never (iRst = '0' and (sState = S_DIV_RECIP or sState = S_DIV_RANGE) and sCnt /= DW and sRem >= sD) report "nl_config: division remainder >= divisor";
  -----------------------------------------------------------------------------------

end architecture behavioral;
