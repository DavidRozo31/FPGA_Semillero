-- =============================================================
--  fk_geom_core.vhd
--  Paso 3 — Cinematica Directa por METODO GEOMETRICO
--  Calcula la posicion final (x,y,z) del efector a partir de los
--  senos/cosenos ya calculados por cordic_seq4 y las longitudes
--  de eslabon de geom_pkg. Sin CORDIC, sin matrices — solo
--  multiplicaciones (mul_q13, combinacional) y sumas.
--
--    r = L2*cos2  + L3*cos23  + L45*cos234
--    z = L1 + L2*sin2 + L3*sin23 + L45*sin234
--    x = r*cos1
--    y = r*sin1
--
--  Latencia: 2 ciclos (etapa 1: r,z ; etapa 2: x,y).
--  done_out es un pulso de 1 ciclo (no se mantiene en alto).
--  Universidad Militar Nueva Granada
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.ik_pkg.ALL;
use work.geom_pkg.ALL;

entity fk_geom_core is
    Port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        start      : in  std_logic;

        cos1_in    : in  std_logic_vector(15 downto 0);
        sin1_in    : in  std_logic_vector(15 downto 0);
        cos2_in    : in  std_logic_vector(15 downto 0);
        sin2_in    : in  std_logic_vector(15 downto 0);
        cos23_in   : in  std_logic_vector(15 downto 0);
        sin23_in   : in  std_logic_vector(15 downto 0);
        cos234_in  : in  std_logic_vector(15 downto 0);
        sin234_in  : in  std_logic_vector(15 downto 0);

        x_out      : out std_logic_vector(15 downto 0);
        y_out      : out std_logic_vector(15 downto 0);
        z_out      : out std_logic_vector(15 downto 0);

        done       : out std_logic
    );
end fk_geom_core;

architecture rtl of fk_geom_core is

    type state_t is (S_IDLE, S_STAGE2);
    signal state : state_t := S_IDLE;

    signal cos1_r, sin1_r : signed(15 downto 0) := (others => '0');
    signal r_r, z_r       : signed(15 downto 0) := (others => '0');
    signal x_r, y_r       : signed(15 downto 0) := (others => '0');
    signal done_reg       : std_logic := '0';

begin

    process(clk, rst)
        variable r_v, z_v : signed(15 downto 0);
    begin
        if rst = '1' then
            state    <= S_IDLE;
            done_reg <= '0';
            cos1_r   <= (others => '0');
            sin1_r   <= (others => '0');
            r_r      <= (others => '0');
            z_r      <= (others => '0');
            x_r      <= (others => '0');
            y_r      <= (others => '0');

        elsif rising_edge(clk) then

            case state is

                when S_IDLE =>
                    done_reg <= '0';
                    if start = '1' then
                        -- Etapa 1: r y z (usa cos2/cos23/cos234 tal cual llegan)
                        r_v := mul_q13(L2_Q13, signed(cos2_in))
                             + mul_q13(L3_Q13, signed(cos23_in))
                             + mul_q13(L45_Q13, signed(cos234_in));

                        z_v := L1_Q13
                             + mul_q13(L2_Q13, signed(sin2_in))
                             + mul_q13(L3_Q13, signed(sin23_in))
                             + mul_q13(L45_Q13, signed(sin234_in));

                        r_r <= r_v;
                        z_r <= z_v;

                        -- Se guardan cos1/sin1 para usarlos en la etapa 2
                        cos1_r <= signed(cos1_in);
                        sin1_r <= signed(sin1_in);

                        state <= S_STAGE2;
                    end if;

                when S_STAGE2 =>
                    -- Etapa 2: x = r*cos1 , y = r*sin1
                    x_r      <= mul_q13(r_r, cos1_r);
                    y_r      <= mul_q13(r_r, sin1_r);
                    done_reg <= '1';
                    state    <= S_IDLE;

            end case;
        end if;
    end process;

    x_out <= std_logic_vector(x_r);
    y_out <= std_logic_vector(y_r);
    z_out <= std_logic_vector(z_r);
    done  <= done_reg;

end rtl;