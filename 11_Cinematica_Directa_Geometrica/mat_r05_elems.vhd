-- =============================================================
--  mat_r05_elems.vhd
--  Paso 4c — Orientacion (Roll/Pitch/Yaw) por METODO GEOMETRICO
--  Calcula los 5 elementos de R0_5 (matriz de rotacion base->efector)
--  que hacen falta para las formulas de tu profesor:
--
--    R11 = -cos1*sin234*cos5 + sin1*sin5
--    R21 = -sin1*sin234*cos5 - cos1*sin5
--    R31 =  cos234*cos5
--    R32 = -cos234*sin5
--    R33 =  sin234
--
--  CORREGIDO (revision con el profesor): theta5 (la muñeca) rota
--  alrededor del eje Z, no de X — R0_5 = R0_4 * Rz(theta5), NO
--  R0_4 * Rx(theta5) como se penso originalmente. Ver la leccion 11
--  del repo del semillero (seccion "Correccion: el eje de la muñeca")
--  para la derivacion completa de por que cambia.
--
--  Derivado a mano expandiendo R0_4 = R1*R2*R3*R4 (cadena DH) y
--  R0_5 = R0_4*Rz(theta5). Solo productos simples (mul_q13) —
--  nada de multiplicacion de matriz completa.
--
--  Latencia: 2 ciclos (etapa 1: productos intermedios cos1*sin234 y
--  sin1*sin234 — etapa 2: combinar con cos5/sin5). done_out es un
--  pulso de 1 ciclo.
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

    type state_t is (S_IDLE, S_STAGE2);
    signal state : state_t := S_IDLE;

    -- etapa 1: productos intermedios y valores capturados
    signal a_r, b_r             : signed(15 downto 0) := (others => '0');  -- cos1*sin234, sin1*sin234
    signal cos1_r, sin1_r       : signed(15 downto 0) := (others => '0');
    signal cos234_r, sin234_r   : signed(15 downto 0) := (others => '0');
    signal cos5_r, sin5_r       : signed(15 downto 0) := (others => '0');

    signal r11_r, r21_r, r31_r, r32_r, r33_r : signed(15 downto 0) := (others => '0');
    signal done_reg : std_logic := '0';

begin

    process(clk, rst)
    begin
        if rst = '1' then
            state    <= S_IDLE;
            a_r      <= (others => '0');
            b_r      <= (others => '0');
            cos1_r   <= (others => '0');
            sin1_r   <= (others => '0');
            cos234_r <= (others => '0');
            sin234_r <= (others => '0');
            cos5_r   <= (others => '0');
            sin5_r   <= (others => '0');
            r11_r    <= (others => '0');
            r21_r    <= (others => '0');
            r31_r    <= (others => '0');
            r32_r    <= (others => '0');
            r33_r    <= (others => '0');
            done_reg <= '0';

        elsif rising_edge(clk) then

            case state is

                when S_IDLE =>
                    done_reg <= '0';
                    if start = '1' then
                        -- Etapa 1: productos que necesitan combinarse con cos5/sin5 despues
                        a_r      <= mul_q13(signed(cos1_in), signed(sin234_in));
                        b_r      <= mul_q13(signed(sin1_in), signed(sin234_in));
                        cos1_r   <= signed(cos1_in);
                        sin1_r   <= signed(sin1_in);
                        cos234_r <= signed(cos234_in);
                        sin234_r <= signed(sin234_in);
                        cos5_r   <= signed(cos5_in);
                        sin5_r   <= signed(sin5_in);
                        state    <= S_STAGE2;
                    end if;

                when S_STAGE2 =>
                    -- Etapa 2: R0_5 = R0_4 * Rz(theta5)
                    r11_r    <= -mul_q13(a_r, cos5_r) + mul_q13(sin1_r, sin5_r);
                    r21_r    <= -mul_q13(b_r, cos5_r) - mul_q13(cos1_r, sin5_r);
                    r31_r    <=  mul_q13(cos234_r, cos5_r);
                    r32_r    <= -mul_q13(cos234_r, sin5_r);
                    r33_r    <=  sin234_r;
                    done_reg <= '1';
                    state    <= S_IDLE;

            end case;
        end if;
    end process;

    r11_out <= std_logic_vector(r11_r);
    r21_out <= std_logic_vector(r21_r);
    r31_out <= std_logic_vector(r31_r);
    r32_out <= std_logic_vector(r32_r);
    r33_out <= std_logic_vector(r33_r);
    done    <= done_reg;

end rtl;
