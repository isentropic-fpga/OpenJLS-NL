-- Copyright (C) 2026 Vitor Mendes Camilo
-- SPDX-License-Identifier: GPL-3.0-only
--
-- This file is part of OpenJLS. Available under GPLv3 or a
-- commercial license. See LICENSE and README for details.
--
-- tb_nl_golden: codes one PGM with nl_top at NEAR, the 4 lane streams go to OUT_PREFIX<l>.jls
--
-- build_run.sh crops the width to a multiple of 4 and compares the streams with CharLS per stripe.

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use std.env.all;

entity tb_nl_golden is
  generic (
    PGM_PATH   : string  := "img.pgm";
    OUT_PREFIX : string  := "out";
    BITNESS    : natural := 8;
    NEAR       : natural := 3
  );
end entity tb_nl_golden;

architecture bench of tb_nl_golden is

  constant CLK_PERIOD : time     := 10 ns;
  constant MAX_W      : positive := 65532;
  constant MAX_H      : positive := 65535;
  constant OUT_WIDTH  : natural  := 64;
  constant NB         : natural  := OUT_WIDTH / 8;

  type pixel_array_t is array (natural range <>) of natural;
  type pixel_ptr_t is access pixel_array_t;
  type char_file_t is file of character;

  signal iClk         : std_logic := '0';
  signal iRst         : std_logic;
  signal iValid       : std_logic;
  signal iPixel       : std_logic_vector(BITNESS - 1 downto 0);
  signal oReady       : std_logic;
  signal oData        : std_logic_vector(4 * OUT_WIDTH - 1 downto 0);
  signal oValid       : std_logic_vector(3 downto 0);
  signal oKeep        : std_logic_vector(4 * NB - 1 downto 0);
  signal oLast        : std_logic_vector(3 downto 0);
  signal sImgW        : natural := 32;
  signal sImgH        : natural := 1;
  signal sDone        : std_logic_vector(3 downto 0) := (others => '0');

  procedure read_pgm_int (
    file f : char_file_t;
    v      : out natural
  ) is

    variable c         : character;
    variable acc       : natural := 0;
    variable started   : boolean := false;
    variable inComment : boolean := false;

  begin

    loop

      assert not endfile(f)
        report "PGM: unexpected EOF in header"
        severity failure;
      read(f, c);

      if (inComment) then
        inComment := not (c = LF or c = CR);
      elsif (not started) then
        if (c = '#') then
          inComment := true;
        elsif (c >= '0' and c <= '9') then
          acc     := character'pos(c) - character'pos('0');
          started := true;
        end if;
      elsif (c >= '0' and c <= '9') then
        acc := acc * 10 + character'pos(c) - character'pos('0');
      else
        v := acc;
        return;
      end if;

    end loop;

  end procedure read_pgm_int;

begin

  iClk <= not iClk after CLK_PERIOD / 2;

  dut : entity work.nl_top
    generic map (
      BITNESS          => BITNESS,
      MAX_IMAGE_WIDTH  => MAX_W,
      MAX_IMAGE_HEIGHT => MAX_H,
      OUT_WIDTH        => OUT_WIDTH
    )
    port map (
      iClk         => iClk,
      iRst         => iRst,
      iNear        => std_logic_vector(to_unsigned(NEAR, 8)),
      iImageWidth  => std_logic_vector(to_unsigned(sImgW, 16)),
      iImageHeight => std_logic_vector(to_unsigned(sImgH, 16)),
      iValid       => iValid,
      iPixel       => iPixel,
      oReady       => oReady,
      oData        => oData,
      oValid       => oValid,
      oKeep        => oKeep,
      oLast        => oLast,
      iReady       => (others => '1')
    );

  -- One writer per lane, closes its file on oLast.
  g_cap : for l in 0 to 3 generate

    cap : process is

      file     f : char_file_t;
      variable b : natural;

    begin

      file_open(f, OUT_PREFIX & integer'image(l) & ".jls", write_mode);

      loop

        wait until rising_edge(iClk);

        if (oValid(l) = '1') then

          for k in 0 to NB - 1 loop

            if (oKeep(l * NB + NB - 1 - k) = '1') then
              b := to_integer(unsigned(oData(l * OUT_WIDTH + OUT_WIDTH - 1 - 8 * k downto
                                             l * OUT_WIDTH + OUT_WIDTH - 8 - 8 * k)));
              write(f, character'val(b));
            end if;

          end loop;

          exit when oLast(l) = '1';
        end if;

      end loop;

      file_close(f);
      sDone(l) <= '1';
      wait;

    end process cap;

  end generate g_cap;

  stim : process is

    file     f      : char_file_t;
    variable c      : character;
    variable w      : natural;
    variable h      : natural;
    variable mx     : natural;
    variable pixels : pixel_ptr_t;
    variable cyc    : natural := 0;

  begin

    iRst   <= '1';
    iValid <= '0';
    iPixel <= (others => '0');

    file_open(f, PGM_PATH, read_mode);
    read(f, c);
    assert c = 'P'
      report "PGM: magic mismatch"
      severity failure;
    read(f, c);
    assert c = '5'
      report "PGM: magic mismatch"
      severity failure;
    read_pgm_int(f, w);
    read_pgm_int(f, h);
    read_pgm_int(f, mx);
    assert w mod 4 = 0 and w >= 32 and w <= MAX_W and h <= MAX_H
      report "PGM: unsupported dimensions"
      severity failure;
    assert mx < 2 ** BITNESS and mx >= 2 ** (BITNESS - 1)
      report "PGM maxval does not match BITNESS"
      severity failure;
    pixels := new pixel_array_t(0 to w * h - 1);

    for i in pixels'range loop

      read(f, c);
      pixels(i) := character'pos(c);

      if (mx > 255) then
        read(f, c);
        pixels(i) := pixels(i) * 256 + character'pos(c);
      end if;

    end loop;

    file_close(f);
    sImgW <= w;
    sImgH <= h;

    for i in 0 to 4 loop

      wait until rising_edge(iClk);

    end loop;

    iRst <= '0';

    for i in pixels'range loop

      iPixel <= std_logic_vector(to_unsigned(pixels(i), BITNESS));
      iValid <= '1';
      wait until rising_edge(iClk) and oReady = '1';

    end loop;

    iValid <= '0';

    while sDone /= "1111" loop

      wait until rising_edge(iClk);
      cyc := cyc + 1;
      assert cyc < 100 * w + 100000
        report "timeout waiting for oLast on all lanes"
        severity failure;

    end loop;

    report "tb_nl_golden done: " & integer'image(w) & "x" & integer'image(h) & " NEAR=" & integer'image(NEAR);
    finish;

  end process stim;

end architecture bench;
