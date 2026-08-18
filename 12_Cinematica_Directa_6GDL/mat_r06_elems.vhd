-- =============================================================
--  mat_r06_elems.vhd
--  Paso 3 — Orientacion (Roll/Pitch/Yaw) por METODO GEOMETRICO
--  Brazo de 6 GDL real (reemplaza a mat_r05_elems.vhd)
--
--  Calcula los 7 elementos de R0_6 (matriz de rotacion base->efector)
--  que hacen falta: los 5 de siempre para Yaw/Pitch/Roll (R11,R21,
--  R31,R32,R33) MAS R13,R23 -- estos dos ya NO son opcionales: los
--  necesita fk_geom_core para la posicion (ver leccion 12, la muñeca
--  ahora tiene longitud propia y mueve la posicion).
--
--  ================== REVISION 2 (correccion de recursos) ==================
--  La primera version de este archivo calculaba las 31 multiplicaciones
--  que hacen falta EN PARALELO, repartidas en solo 3 ciclos -- eso
--  disparo la cuenta de multiplicadores dedicados que pide Quartus
--  (9-21 al mismo tiempo) muy por encima de los 30 "elementos de 9
--  bits" que trae el EP4CE6 completo. El sobrante se armo con
--  puertas logicas (carisimo), y el diseño completo paso de 1,754 LEs
--  (brazo de 5R) a 9,118 LEs (145% del chip, Fitter Failed). Ver
--  leccion 12 del repo para el analisis completo con los numeros.
--
--  CORREGIDO aqui aplicando la MISMA idea que ya usa cordic_seq6 (un
--  solo CORDIC reutilizado en 6 pasadas en vez de 6 en paralelo): un
--  SOLO multiplicador (mul_q13, una unica linea "mul_r := mul_q13(...)"
--  en todo el archivo) reutilizado en 31 pasos secuenciales, uno por
--  ciclo, seleccionando operandos con un case sobre un contador "step".
--  Se verifico primero en Python que esta secuencia da EXACTO lo mismo
--  que la version paralela (bit a bit, sobre los mismos dos casos que
--  ya se probaron en ModelSim) antes de traducir a VHDL.
--  ==========================================================================
--
--    R11 = c6*(c1*(c23*s5) + c5*t5)  + s6*t6b
--    R21 = c6*(-c5*t6 + s1*(c23*s5)) + s6*t5b
--    R31 = -c6*t9 - s6*(c23*c4)
--    R32 = -c6*(c23*c4) + s6*t9
--    R33 = (c5*s23) + s4*(c23*s5)
--    R13 = c1*(c23*c5) - s5*t5
--    R23 = s1*(c23*c5) + s5*t6
--
--  con t5,t5b,t6,t6b,t9 = mismos terminos intermedios de siempre
--  (ver comentario historico mas abajo en el codigo, pasos 9-14).
--
--  Latencia: 31 ciclos de multiplicacion + 1 de latch = ~32 ciclos
--  (mucho mas que la version paralela de 3 ciclos, a proposito --
--  se sacrifica velocidad por area, prioridad de todo este proyecto
--  desde el metodo geometrico). done_out es un pulso de 1 ciclo.
--  Universidad Militar Nueva Granada
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.ik_pkg.ALL;

entity mat_r06_elems is
    Port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        start      : in  std_logic;

        cos1_in    : in  std_logic_vector(15 downto 0);
        sin1_in    : in  std_logic_vector(15 downto 0);
        cos23_in   : in  std_logic_vector(15 downto 0);
        sin23_in   : in  std_logic_vector(15 downto 0);
        cos4_in    : in  std_logic_vector(15 downto 0);
        sin4_in    : in  std_logic_vector(15 downto 0);
        cos5_in    : in  std_logic_vector(15 downto 0);
        sin5_in    : in  std_logic_vector(15 downto 0);
        cos6_in    : in  std_logic_vector(15 downto 0);
        sin6_in    : in  std_logic_vector(15 downto 0);

        r11_out    : out std_logic_vector(15 downto 0);
        r21_out    : out std_logic_vector(15 downto 0);
        r31_out    : out std_logic_vector(15 downto 0);
        r32_out    : out std_logic_vector(15 downto 0);
        r33_out    : out std_logic_vector(15 downto 0);
        r13_out    : out std_logic_vector(15 downto 0);
        r23_out    : out std_logic_vector(15 downto 0);

        done       : out std_logic
    );
end mat_r06_elems;

architecture rtl of mat_r06_elems is

    type state_t is (S_IDLE, S_COMPUTE);
    signal state : state_t := S_IDLE;
    signal step  : integer range 0 to 30 := 0;

    -- entradas latcheadas (se usan a lo largo de los 31 pasos)
    signal c1_r, s1_r, c23_r, s23_r : signed(15 downto 0) := (others => '0');
    signal c4_r, s4_r, c5_r, s5_r   : signed(15 downto 0) := (others => '0');
    signal c6_r, s6_r               : signed(15 downto 0) := (others => '0');

    -- pasos 0-8: productos directos de las entradas
    signal p_s1s4, p_c1c4, p_c23s5, p_c4s1   : signed(15 downto 0) := (others => '0');
    signal p_c1s4, p_c23c4, p_c23c5, p_s23s5 : signed(15 downto 0) := (others => '0');
    signal p_c5s23                           : signed(15 downto 0) := (others => '0');

    -- pasos 9-14: terminos intermedios (igual formula que la version paralela)
    signal t5, t5b, t6, t6b, t9 : signed(15 downto 0) := (others => '0');

    -- pasos 15-30: acumuladores parciales de R11,R21 (necesitan 2 productos cada uno)
    signal g_r11a, g_r11sum, g_r11h1 : signed(15 downto 0) := (others => '0');
    signal g_r21a, g_r21sum, g_r21h1 : signed(15 downto 0) := (others => '0');
    signal g_r31a, g_r32a            : signed(15 downto 0) := (others => '0');
    signal g_r13a, g_r23a            : signed(15 downto 0) := (others => '0');

    signal r11_r, r21_r, r31_r, r32_r, r33_r : signed(15 downto 0) := (others => '0');
    signal r13_r, r23_r                      : signed(15 downto 0) := (others => '0');
    signal done_reg : std_logic := '0';

begin

    process(clk, rst)
        variable mul_a, mul_b, mul_r : signed(15 downto 0);
    begin
        if rst = '1' then
            state    <= S_IDLE;
            step     <= 0;
            done_reg <= '0';
            c1_r <= (others => '0'); s1_r <= (others => '0');
            c23_r <= (others => '0'); s23_r <= (others => '0');
            c4_r <= (others => '0'); s4_r <= (others => '0');
            c5_r <= (others => '0'); s5_r <= (others => '0');
            c6_r <= (others => '0'); s6_r <= (others => '0');
            p_s1s4 <= (others => '0'); p_c1c4 <= (others => '0');
            p_c23s5 <= (others => '0'); p_c4s1 <= (others => '0');
            p_c1s4 <= (others => '0'); p_c23c4 <= (others => '0');
            p_c23c5 <= (others => '0'); p_s23s5 <= (others => '0');
            p_c5s23 <= (others => '0');
            t5 <= (others => '0'); t5b <= (others => '0');
            t6 <= (others => '0'); t6b <= (others => '0'); t9 <= (others => '0');
            g_r11a <= (others => '0'); g_r11sum <= (others => '0'); g_r11h1 <= (others => '0');
            g_r21a <= (others => '0'); g_r21sum <= (others => '0'); g_r21h1 <= (others => '0');
            g_r31a <= (others => '0'); g_r32a <= (others => '0');
            g_r13a <= (others => '0'); g_r23a <= (others => '0');
            r11_r <= (others => '0'); r21_r <= (others => '0');
            r31_r <= (others => '0'); r32_r <= (others => '0'); r33_r <= (others => '0');
            r13_r <= (others => '0'); r23_r <= (others => '0');

        elsif rising_edge(clk) then

            case state is

                when S_IDLE =>
                    done_reg <= '0';
                    if start = '1' then
                        c1_r  <= signed(cos1_in);  s1_r  <= signed(sin1_in);
                        c23_r <= signed(cos23_in); s23_r <= signed(sin23_in);
                        c4_r  <= signed(cos4_in);  s4_r  <= signed(sin4_in);
                        c5_r  <= signed(cos5_in);  s5_r  <= signed(sin5_in);
                        c6_r  <= signed(cos6_in);  s6_r  <= signed(sin6_in);
                        step  <= 0;
                        state <= S_COMPUTE;
                    end if;

                when S_COMPUTE =>
                    -- ---- seleccion de operandos (unico multiplicador reutilizado) ----
                    case step is
                        when 0 => mul_a := s1_r; mul_b := s4_r;
                        when 1 => mul_a := c1_r; mul_b := c4_r;
                        when 2 => mul_a := c23_r; mul_b := s5_r;
                        when 3 => mul_a := c4_r; mul_b := s1_r;
                        when 4 => mul_a := c1_r; mul_b := s4_r;
                        when 5 => mul_a := c23_r; mul_b := c4_r;
                        when 6 => mul_a := c23_r; mul_b := c5_r;
                        when 7 => mul_a := s23_r; mul_b := s5_r;
                        when 8 => mul_a := c5_r; mul_b := s23_r;
                        when 9 => mul_a := s23_r; mul_b := p_c1s4;
                        when 10 => mul_a := s23_r; mul_b := p_c4s1;
                        when 11 => mul_a := s23_r; mul_b := p_s1s4;
                        when 12 => mul_a := s23_r; mul_b := p_c1c4;
                        when 13 => mul_a := s4_r; mul_b := p_c23c5;
                        when 14 => mul_a := s4_r; mul_b := p_c23s5;
                        when 15 => mul_a := c1_r; mul_b := p_c23s5;
                        when 16 => mul_a := c5_r; mul_b := t5;
                        when 17 => mul_a := c6_r; mul_b := g_r11sum;
                        when 18 => mul_a := s6_r; mul_b := t6b;
                        when 19 => mul_a := s1_r; mul_b := p_c23s5;
                        when 20 => mul_a := c5_r; mul_b := t6;
                        when 21 => mul_a := c6_r; mul_b := g_r21sum;
                        when 22 => mul_a := s6_r; mul_b := t5b;
                        when 23 => mul_a := c6_r; mul_b := t9;
                        when 24 => mul_a := s6_r; mul_b := p_c23c4;
                        when 25 => mul_a := c6_r; mul_b := p_c23c4;
                        when 26 => mul_a := s6_r; mul_b := t9;
                        when 27 => mul_a := c1_r; mul_b := p_c23c5;
                        when 28 => mul_a := s5_r; mul_b := t5;
                        when 29 => mul_a := s1_r; mul_b := p_c23c5;
                        when others => mul_a := s5_r; mul_b := t6;  -- step 30
                    end case;

                    mul_r := mul_q13(mul_a, mul_b);  -- UNICA llamada a mul_q13 de todo el archivo

                    -- ---- ruteo del resultado al destino de este paso ----
                    case step is
                        when 0 => p_s1s4  <= mul_r;
                        when 1 => p_c1c4  <= mul_r;
                        when 2 => p_c23s5 <= mul_r;
                        when 3 => p_c4s1  <= mul_r;
                        when 4 => p_c1s4  <= mul_r;
                        when 5 => p_c23c4 <= mul_r;
                        when 6 => p_c23c5 <= mul_r;
                        when 7 => p_s23s5 <= mul_r;
                        when 8 => p_c5s23 <= mul_r;
                        when 9 => t5  <= mul_r + p_c4s1;
                        when 10 => t5b <= mul_r + p_c1s4;
                        when 11 => t6  <= p_c1c4 - mul_r;
                        when 12 => t6b <= mul_r - p_s1s4;
                        when 13 => t9  <= mul_r - p_s23s5;
                        when 14 => r33_r <= p_c5s23 + mul_r;
                        when 15 => g_r11a <= mul_r;
                        when 16 => g_r11sum <= g_r11a + mul_r;
                        when 17 => g_r11h1 <= mul_r;
                        when 18 => r11_r <= g_r11h1 + mul_r;
                        when 19 => g_r21a <= mul_r;
                        when 20 => g_r21sum <= g_r21a - mul_r;
                        when 21 => g_r21h1 <= mul_r;
                        when 22 => r21_r <= g_r21h1 + mul_r;
                        when 23 => g_r31a <= mul_r;
                        when 24 => r31_r <= -g_r31a - mul_r;
                        when 25 => g_r32a <= mul_r;
                        when 26 => r32_r <= -g_r32a + mul_r;
                        when 27 => g_r13a <= mul_r;
                        when 28 => r13_r <= g_r13a - mul_r;
                        when 29 => g_r23a <= mul_r;
                        when others =>  -- step 30, ultimo paso
                            r23_r    <= g_r23a + mul_r;
                            done_reg <= '1';
                            state    <= S_IDLE;
                    end case;

                    if step < 30 then
                        step <= step + 1;
                    end if;

            end case;
        end if;
    end process;

    r11_out <= std_logic_vector(r11_r);
    r21_out <= std_logic_vector(r21_r);
    r31_out <= std_logic_vector(r31_r);
    r32_out <= std_logic_vector(r32_r);
    r33_out <= std_logic_vector(r33_r);
    r13_out <= std_logic_vector(r13_r);
    r23_out <= std_logic_vector(r23_r);
    done    <= done_reg;

end rtl;
