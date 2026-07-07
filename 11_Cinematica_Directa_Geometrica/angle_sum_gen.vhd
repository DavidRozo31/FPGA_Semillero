-- =============================================================
--  angle_sum_gen.vhd
--  Paso 1 — Cinematica Directa por METODO GEOMETRICO
--  Genera los angulos acumulados phi2, phi23, phi234 que necesita
--  el CORDIC secuencial (Paso 2) para armar r y z del brazo.
--
--  phi2   = theta2
--  phi23  = theta2 + theta3            (reducido a [-pi, pi])
--  phi234 = theta2 + theta3 + theta4   (reducido a [-pi, pi])
--
--  Latencia: 1 ciclo. done se mantiene en 1 hasta el proximo start.
--  Universidad Militar Nueva Granada
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.geom_pkg.ALL;

entity angle_sum_gen is
    Port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        start      : in  std_logic;
        theta2_in  : in  std_logic_vector(15 downto 0);
        theta3_in  : in  std_logic_vector(15 downto 0);
        theta4_in  : in  std_logic_vector(15 downto 0);
        phi2_out   : out std_logic_vector(15 downto 0);
        phi23_out  : out std_logic_vector(15 downto 0);
        phi234_out : out std_logic_vector(15 downto 0);
        done       : out std_logic
    );
end angle_sum_gen;

architecture rtl of angle_sum_gen is

    signal phi2_r   : signed(15 downto 0) := (others => '0');
    signal phi23_r  : signed(15 downto 0) := (others => '0');
    signal phi234_r : signed(15 downto 0) := (others => '0');
    signal done_reg : std_logic := '0';

begin

    process(clk, rst)
        variable s23  : signed(17 downto 0);
        variable s234 : signed(17 downto 0);
    begin
        if rst = '1' then
            phi2_r   <= (others => '0');
            phi23_r  <= (others => '0');
            phi234_r <= (others => '0');
            done_reg <= '0';

        elsif rising_edge(clk) then

            -- done baja con nuevo start (se mantiene en 1 el resto del tiempo)
            if start = '1' then
                done_reg <= '0';

                phi2_r <= signed(theta2_in);

                s23  := resize(signed(theta2_in), 18) + resize(signed(theta3_in), 18);
                phi23_r <= wrap_to_pi(s23);

                s234 := s23 + resize(signed(theta4_in), 18);
                phi234_r <= wrap_to_pi(s234);

                done_reg <= '1';
            end if;

        end if;
    end process;

    phi2_out   <= std_logic_vector(phi2_r);
    phi23_out  <= std_logic_vector(phi23_r);
    phi234_out <= std_logic_vector(phi234_r);
    done       <= done_reg;

end rtl;