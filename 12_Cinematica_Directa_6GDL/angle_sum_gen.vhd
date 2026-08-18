-- =============================================================
--  angle_sum_gen.vhd
--  Paso 1 — Cinematica Directa por METODO GEOMETRICO (brazo de 6 GDL)
--  Genera el unico angulo acumulado que hace falta para armar r4,z4
--  (posicion del sistema 4, ver fk_geom_core):
--
--    phi23 = theta2 + theta3   (reducido a [-pi, pi])
--
--  ACTUALIZADO respecto al brazo de 5R+gripper: ya NO hace falta
--  phi234 (theta4 no es coplanar con theta2,theta3 en la tabla DH
--  nueva -- ver leccion 12), y phi2 ya no hace falta calcularlo
--  porque es literalmente theta2 (se cablea theta2_in directo en el
--  BDF donde antes iba phi2_out).
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
        phi23_out  : out std_logic_vector(15 downto 0);
        done       : out std_logic
    );
end angle_sum_gen;

architecture rtl of angle_sum_gen is

    signal phi23_r  : signed(15 downto 0) := (others => '0');
    signal done_reg : std_logic := '0';

begin

    process(clk, rst)
        variable s23 : signed(17 downto 0);
    begin
        if rst = '1' then
            phi23_r  <= (others => '0');
            done_reg <= '0';

        elsif rising_edge(clk) then

            -- done baja con nuevo start (se mantiene en 1 el resto del tiempo)
            if start = '1' then
                done_reg <= '0';

                s23 := resize(signed(theta2_in), 18) + resize(signed(theta3_in), 18);
                phi23_r <= wrap_to_pi(s23);

                done_reg <= '1';
            end if;

        end if;
    end process;

    phi23_out <= std_logic_vector(phi23_r);
    done      <= done_reg;

end rtl;
