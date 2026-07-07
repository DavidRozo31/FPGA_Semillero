-- =============================================================
--  cordic_atan2_16.vhd
--  CORDIC modo VECTORING - atan2(y,x) y magnitud en Q2.13 (16 bits)
--
--  A diferencia de cordic_sincos_16 (modo ROTATION, gira un vector
--  un angulo conocido), este modulo gira el vector de entrada
--  (x_in, y_in) HASTA que su componente Y llega a cero. El angulo
--  que acumulo en el camino es atan2(y_in, x_in). La magnitud final
--  del vector (compensada por la ganancia K) es sqrt(x_in^2+y_in^2).
--
--  Correccion de cuadrante: si x_in<0, se rota 180 grados antes de
--  iterar (el algoritmo base solo converge para x>0, rango ±90°).
--
--  Latencia: N_ITER + 1 = 13 ciclos (misma que cordic_sincos_16).
--  Universidad Militar Nueva Granada
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.cordic_pkg.ALL;
use work.ik_pkg.ALL;

entity cordic_atan2_16 is
    Port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        start      : in  std_logic;
        x_in       : in  std_logic_vector(15 downto 0);
        y_in       : in  std_logic_vector(15 downto 0);
        angle_out  : out std_logic_vector(15 downto 0);  -- atan2(y_in,x_in), Q2.13 rad
        mag_out    : out std_logic_vector(15 downto 0);  -- sqrt(x_in^2+y_in^2), Q2.13
        done       : out std_logic
    );
end cordic_atan2_16;

architecture rtl of cordic_atan2_16 is

    type pipe_array is array (0 to N_ITER) of signed(15 downto 0);

    signal x_pipe     : pipe_array := (others => (others => '0'));
    signal y_pipe     : pipe_array := (others => (others => '0'));
    signal z_pipe     : pipe_array := (others => (others => '0'));
    signal valid_pipe : std_logic_vector(N_ITER downto 0) := (others => '0');

    signal x_s, y_s : signed(15 downto 0);

begin

    x_s <= signed(x_in);
    y_s <= signed(y_in);

    process(clk, rst)
        variable x_sh : signed(15 downto 0);
        variable y_sh : signed(15 downto 0);
    begin
        if rst = '1' then
            x_pipe     <= (others => (others => '0'));
            y_pipe     <= (others => (others => '0'));
            z_pipe     <= (others => (others => '0'));
            valid_pipe <= (others => '0');

        elsif rising_edge(clk) then

            valid_pipe(0) <= start;

            if start = '1' then
                -- Correccion de cuadrante: si x<0, rotar 180 grados
                -- (negar x,y) y sembrar z0 con +-PI segun el signo de y.
                if x_s < 0 then
                    x_pipe(0) <= -x_s;
                    y_pipe(0) <= -y_s;
                    if y_s >= 0 then
                        z_pipe(0) <= PI_Q13;
                    else
                        z_pipe(0) <= -PI_Q13;
                    end if;
                else
                    x_pipe(0) <= x_s;
                    y_pipe(0) <= y_s;
                    z_pipe(0) <= (others => '0');
                end if;
            end if;

            for i in 1 to N_ITER loop
                valid_pipe(i) <= valid_pipe(i-1);

                x_sh := shift_right(x_pipe(i-1), i-1);
                y_sh := shift_right(y_pipe(i-1), i-1);

                -- Modo vectoring: el signo de la rotacion lo decide Y
                -- (se busca llevar Y a cero), no Z como en modo rotation.
                if y_pipe(i-1) >= 0 then
                    x_pipe(i) <= x_pipe(i-1) + y_sh;
                    y_pipe(i) <= y_pipe(i-1) - x_sh;
                    z_pipe(i) <= z_pipe(i-1) + ATAN_TABLE(i-1);
                else
                    x_pipe(i) <= x_pipe(i-1) - y_sh;
                    y_pipe(i) <= y_pipe(i-1) + x_sh;
                    z_pipe(i) <= z_pipe(i-1) - ATAN_TABLE(i-1);
                end if;
            end loop;

        end if;
    end process;

    angle_out <= std_logic_vector(z_pipe(N_ITER));
    mag_out   <= std_logic_vector(mul_q13(x_pipe(N_ITER), CORDIC_K));
    done      <= valid_pipe(N_ITER);

end rtl;
