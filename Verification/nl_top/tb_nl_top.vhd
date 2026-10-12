-- nl_top stimulus/capture: streams PIX_PATH (one hex pixel per line, raster
-- order) IMAGES times, writes each lane's bytes to OUT_PREFIX<l>.hex.
-- GAPS / BP add random input bubbles / output back-pressure (READY_PCT).

library ieee;
  use ieee.std_logic_1164.all;
  use ieee.numeric_std.all;
  use ieee.math_real.all;
  use std.textio.all;

entity tb_nl_top is
  generic (
    BITNESS    : natural := 8;
    WIDTH      : natural := 64;
    HEIGHT     : natural := 16;
    NEAR       : natural := 3;
    IMAGES     : natural := 1;
    GAPS       : boolean := false;
    BP         : boolean := false;
    READY_PCT  : natural := 60;                                -- iReady duty under BP
    PIX_PATH   : string  := "img.hex";
    OUT_PREFIX : string  := "out"
  );
end entity tb_nl_top;

architecture sim of tb_nl_top is

  constant OUT_WIDTH : natural := 64;
  constant NB        : natural := OUT_WIDTH / 8;

  signal clk    : std_logic := '0';
  signal rst    : std_logic := '1';
  signal valid  : std_logic := '0';
  signal pixel  : std_logic_vector(BITNESS - 1 downto 0) := (others => '0');
  signal ready  : std_logic;
  signal oData  : std_logic_vector(4 * OUT_WIDTH - 1 downto 0);
  signal oValid : std_logic_vector(3 downto 0);
  signal oKeep  : std_logic_vector(4 * NB - 1 downto 0);
  signal oLast  : std_logic_vector(3 downto 0);
  signal iReady : std_logic_vector(3 downto 0) := (others => '1');
  signal done   : boolean := false;

  type t_pix is array (natural range <>) of std_logic_vector(BITNESS - 1 downto 0);

begin

  clk <= not clk after 5 ns when not done;

  dut : entity work.nl_top
    generic map (
      BITNESS          => BITNESS,
      MAX_IMAGE_WIDTH  => 1024,
      MAX_IMAGE_HEIGHT => 1024,
      OUT_WIDTH        => OUT_WIDTH
    )
    port map (
      iClk         => clk,
      iRst         => rst,
      iNear        => std_logic_vector(to_unsigned(NEAR, 8)),
      iImageWidth  => std_logic_vector(to_unsigned(WIDTH, 16)),
      iImageHeight => std_logic_vector(to_unsigned(HEIGHT, 16)),
      iValid       => valid,
      iPixel       => pixel,
      oReady       => ready,
      oData        => oData,
      oValid       => oValid,
      oKeep        => oKeep,
      oLast        => oLast,
      iReady       => iReady
    );

  stim : process is

    file     f    : text;
    variable ln   : line;
    variable v    : std_logic_vector(15 downto 0);
    variable pix  : t_pix(0 to WIDTH * HEIGHT - 1);
    variable s1   : positive := 7;
    variable s2   : positive := 11;
    variable r    : real;
    variable i    : natural;

  begin

    file_open(f, PIX_PATH, read_mode);
    for k in pix'range loop
      readline(f, ln);
      hread(ln, v);
      pix(k) := v(BITNESS - 1 downto 0);
    end loop;
    file_close(f);

    rst <= '1';
    wait for 100 ns;
    wait until rising_edge(clk);
    rst <= '0';

    for img in 1 to IMAGES loop
      i := 0;
      while i < pix'length loop
        valid <= '1';
        if GAPS then
          uniform(s1, s2, r);
          if r < 0.3 then
            valid <= '0';
          end if;
        end if;
        pixel <= pix(i);
        wait until rising_edge(clk);
        if valid = '1' and ready = '1' then
          i := i + 1;
        end if;
      end loop;
    end loop;
    valid <= '0';
    wait;

  end process stim;

  g_bp : if BP generate
    bp_p : process (clk) is
      variable s1 : positive := 3;
      variable s2 : positive := 5;
      variable r  : real;
    begin
      if rising_edge(clk) then
        for l in 0 to 3 loop
          uniform(s1, s2, r);
          iReady(l) <= '1' when r < real(READY_PCT) / 100.0 else '0';
        end loop;
      end if;
    end process bp_p;
  end generate g_bp;

  cap : process is

    type t_files is array (0 to 3) of integer;
    file     f0, f1, f2, f3 : text;
    variable ln    : line;
    variable lasts : t_files := (others => 0);
    variable bytes : t_files := (others => 0);
    variable cyc   : natural := 0;
    variable b     : std_logic_vector(7 downto 0);

  begin

    file_open(f0, OUT_PREFIX & "0.hex", write_mode);
    file_open(f1, OUT_PREFIX & "1.hex", write_mode);
    file_open(f2, OUT_PREFIX & "2.hex", write_mode);
    file_open(f3, OUT_PREFIX & "3.hex", write_mode);
    loop
      wait until rising_edge(clk);
      cyc := cyc + 1;
      for l in 0 to 3 loop
        if oValid(l) = '1' and iReady(l) = '1' then
          for k in 0 to NB - 1 loop
            if oKeep(l * NB + NB - 1 - k) = '1' then
              b := oData(l * OUT_WIDTH + OUT_WIDTH - 1 - 8 * k downto l * OUT_WIDTH + OUT_WIDTH - 8 - 8 * k);
              hwrite(ln, b);
              case l is
                when 0 => writeline(f0, ln);
                when 1 => writeline(f1, ln);
                when 2 => writeline(f2, ln);
                when others => writeline(f3, ln);
              end case;
              bytes(l) := bytes(l) + 1;
            end if;
          end loop;
          if oLast(l) = '1' then
            lasts(l) := lasts(l) + 1;
          end if;
        end if;
      end loop;
      exit when lasts(0) = IMAGES and lasts(1) = IMAGES and lasts(2) = IMAGES and lasts(3) = IMAGES;
      if cyc > 200 * WIDTH * HEIGHT * IMAGES + 10000 then
        report "timeout: lasts " & integer'image(lasts(0)) & integer'image(lasts(1)) &
               integer'image(lasts(2)) & integer'image(lasts(3)) severity error;
        exit;
      end if;
    end loop;
    report "done in " & integer'image(cyc) & " cycles, " & real'image(real(WIDTH * HEIGHT * IMAGES) / real(cyc)) &
           " pix/cycle";
    file_close(f0); file_close(f1); file_close(f2); file_close(f3);
    done <= true;
    wait;

  end process cap;

end architecture sim;
