-- =============================================================
--  mat_r05_elems.vhd
--  Paso 4c — Orientacion (Roll/Pitch/Yaw) por METODO GEOMETRICO
--  Calcula los 5 elementos de R0_5 (matriz de rotacion base->efector)
--  que hacen falta para las formulas de tu profesor:
--
--    R11 = -cos1*sin234
--    R21 = -sin1*sin234
--    R31 =  cos234
--    R32 =  sin234*sin5
--    R33 =  sin234*cos5
--
--  Derivado a mano expandiendo R0_4 = R1*R2*R3*R4 (cadena DH) y
--  R0_5 = R0_4*Rx(theta5). Solo productos simples (mul_q13) y un
--  passthrough (R31) — nada de multiplicacion de matriz completa.
--
--  Latencia: 1 ciclo. done_out es un pulso de 1 ciclo.
--  Universidad Militar Nueva Granada
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.ik_pkg.ALL;

entity mat_r05_elems is
    Port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        start      : in  std_logic;

        cos1_in    : in  std_logic_vector(15 downto 0);
        sin1_in    : in  std_logic_vector(15 downto 0);
        cos234_in  : in  std_logic_vector(15 downto 0);
        sin234_in  : in  std_logic_vector(15 downto 0);
        cos5_in    : in  std_logic_vector(15 downto 0);
        sin5_in    : in  std_logic_vector(15 downto 0);

        r11_out    : out std_logic_vector(15 downto 0);
        r21_out    : out std_logic_vector(15 downto 0);
        r31_out    : out std_logic_vector(15 downto 0);
        r32_out    : out std_logic_vector(15 downto 0);
        r33_out    : out std_logic_vector(15 downto 0);

        done       : out std_logic
    );
end mat_r05_elems;

architecture rtl of mat_r05_elems is

    signal r11_r, r21_r, r31_r, r32_r, r33_r : signed(15 downto 0) := (others => '0');
    signal done_reg : std_logic := '0';

begin

    process(clk, rst)
    begin
        if rst = '1' then
            r11_r    <= (others => '0');
            r21_r    <= (others => '0');
            r31_r    <= (others => '0');
            r32_r    <= (others => '0');
            r33_r    <= (others => '0');
            done_reg <= '0';

        elsif rising_edge(clk) then
            done_reg <= '0';
            if start = '1' then
                r11_r <= -mul_q13(signed(cos1_in),   signed(sin234_in));
                r21_r <= -mul_q13(signed(sin1_in),   signed(sin234_in));
                r31_r <=  signed(cos234_in);
                r32_r <=  mul_q13(signed(sin234_in), signed(sin5_in));
                r33_r <=  mul_q13(signed(sin234_in), signed(cos5_in));
                done_reg <= '1';
            end if;
        end if;
    end process;

    r11_out <= std_logic_vector(r11_r);
    r21_out <= std_logic_vector(r21_r);
    r31_out <= std_logic_vector(r31_r);
    r32_out <= std_logic_vector(r32_r);
    r33_out <= std_logic_vector(r33_r);
    done    <= done_reg;

end rtl;
