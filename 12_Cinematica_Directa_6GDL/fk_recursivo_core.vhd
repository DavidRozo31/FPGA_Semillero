-- =============================================================
--  fk_recursivo_core.vhd
--  Cinematica Directa 6 GDL — METODO GEOMETRICO RECURSIVO (correccion del
--  profesor, Inversa2R.pdf). Reemplaza a mat_r06_elems.vhd + fk_geom_core.vhd
--  de la revision anterior.
--
--  ================== POR QUE ESTE MODULO REEMPLAZA A LOS DOS ANTERIORES ====
--  El metodo anterior (formula cerrada, reducida con sympy/CSE) daba los
--  mismos numeros pero ni el profesor ni el estudiante pudieron seguir la
--  derivacion -- no es un metodo reconocible. El profesor pidio el metodo
--  ESTANDAR: acumular POSICION y ORIENTACION eslabon por eslabon,
--
--    O_0 = [0,0,0] ; R_0^0 = I
--    para i = 1..6:
--        p_i = [a_i*cos(theta_i'), a_i*sin(theta_i'), d_i]   (offset local
--              del eslabon i, de su propia fila en la tabla DH)
--        O_i = O_(i-1) + R_(i-1)^0 * p_i
--        R_i^0 = R_(i-1)^0 * R_i        (R_i = dh_rot(theta_i', alpha_i))
--
--  Como la posicion AHORA necesita las mismas matrices de rotacion
--  acumuladas que la orientacion (R_(i-1)^0 en cada paso), ya no tiene
--  sentido tener dos modulos separados -- se fusionan en uno solo.
--
--  ================== SIMPLIFICACIONES QUE MANTIENEN EL AREA BAJA ===========
--  1) Todos los alpha_i de esta tabla son multiplos de 90 grados -- cada
--     R_i se arma SIN multiplicar, solo acomodando +-cos/+-sin/0 en la
--     matriz (ver derivacion completa en la leccion 12/13 del repo).
--  2) Las filas 3 y 4 tienen theta_i'=theta_i+90 grados -- en vez de correr
--     el CORDIC sobre ese angulo desplazado, se usa la identidad
--     cos(x+90)=-sin(x), sin(x+90)=cos(x) (ver cE3,cE4 abajo).
--  3) Un SOLO multiplicador reutilizado en serie (mismo criterio que
--     cordic_seq6 y que la revision anterior de mat_r06_elems/fk_geom_core)
--     -- 74 pasos, verificados en Python (con la misma aritmetica entera
--     truncada de mul_q13, simulando registros reales, no solo evaluacion
--     inmediata) antes de escribir este archivo, para no arriesgar un error
--     de transcripcion en un modulo tan grande.
--  ============================================================================
--
--  Latencia: 74 pasos de multiplicacion + 1 de latch = ~75 ciclos.
--  Universidad Militar Nueva Granada
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.ik_pkg.ALL;
use work.geom_pkg.ALL;

entity fk_recursivo_core is
    Port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        start      : in  std_logic;

        cos1_in    : in  std_logic_vector(15 downto 0);
        sin1_in    : in  std_logic_vector(15 downto 0);
        cos2_in    : in  std_logic_vector(15 downto 0);
        sin2_in    : in  std_logic_vector(15 downto 0);
        cos3_in    : in  std_logic_vector(15 downto 0);
        sin3_in    : in  std_logic_vector(15 downto 0);
        cos4_in    : in  std_logic_vector(15 downto 0);
        sin4_in    : in  std_logic_vector(15 downto 0);
        cos5_in    : in  std_logic_vector(15 downto 0);
        sin5_in    : in  std_logic_vector(15 downto 0);
        cos6_in    : in  std_logic_vector(15 downto 0);
        sin6_in    : in  std_logic_vector(15 downto 0);

        -- orientacion (R0_6), hacia atan2_seq3 (sin cambios en ese modulo)
        r11_out    : out std_logic_vector(15 downto 0);
        r21_out    : out std_logic_vector(15 downto 0);
        r31_out    : out std_logic_vector(15 downto 0);
        r32_out    : out std_logic_vector(15 downto 0);
        r33_out    : out std_logic_vector(15 downto 0);

        -- posicion final
        x_out      : out std_logic_vector(15 downto 0);
        y_out      : out std_logic_vector(15 downto 0);
        z_out      : out std_logic_vector(15 downto 0);

        done       : out std_logic
    );
end fk_recursivo_core;

architecture rtl of fk_recursivo_core is

    type state_t is (S_IDLE, S_COMPUTE);
    signal state : state_t := S_IDLE;
    signal step  : integer range 0 to 73 := 0;

    -- entradas latcheadas (se usan a lo largo de los 74 pasos)
    signal c1_r, s1_r, c2_r, s2_r, c3_r, s3_r : signed(15 downto 0) := (others => '0');
    signal c4_r, s4_r, c5_r, s5_r, c6_r, s6_r : signed(15 downto 0) := (others => '0');

    signal done_reg : std_logic := '0';

    -- === BLOQUE GENERADO (74 pasos, 131 registros internos) ===
    -- --- declaracion de senales internas ---
    signal Lc2 : signed(15 downto 0) := (others => '0');
    signal Ls2 : signed(15 downto 0) := (others => '0');
    signal O2_0 : signed(15 downto 0) := (others => '0');
    signal O2_1 : signed(15 downto 0) := (others => '0');
    signal O2_2 : signed(15 downto 0) := (others => '0');
    signal O4_0 : signed(15 downto 0) := (others => '0');
    signal O4_1 : signed(15 downto 0) := (others => '0');
    signal O4_2 : signed(15 downto 0) := (others => '0');
    signal O6_0 : signed(15 downto 0) := (others => '0');
    signal O6_1 : signed(15 downto 0) := (others => '0');
    signal O6_2 : signed(15 downto 0) := (others => '0');
    signal R2_00 : signed(15 downto 0) := (others => '0');
    signal R2_01 : signed(15 downto 0) := (others => '0');
    signal R2_02 : signed(15 downto 0) := (others => '0');
    signal R2_10 : signed(15 downto 0) := (others => '0');
    signal R2_11 : signed(15 downto 0) := (others => '0');
    signal R2_12 : signed(15 downto 0) := (others => '0');
    signal R2_20 : signed(15 downto 0) := (others => '0');
    signal R2_21 : signed(15 downto 0) := (others => '0');
    signal R2_22 : signed(15 downto 0) := (others => '0');
    signal R2_t0_0 : signed(15 downto 0) := (others => '0');
    signal R2_t0_1 : signed(15 downto 0) := (others => '0');
    signal R2_t0_2 : signed(15 downto 0) := (others => '0');
    signal R2_t1_0 : signed(15 downto 0) := (others => '0');
    signal R2_t1_1 : signed(15 downto 0) := (others => '0');
    signal R2_t1_2 : signed(15 downto 0) := (others => '0');
    signal R2_t2_0 : signed(15 downto 0) := (others => '0');
    signal R2_t2_1 : signed(15 downto 0) := (others => '0');
    signal R2_t2_2 : signed(15 downto 0) := (others => '0');
    signal R2_t3_0 : signed(15 downto 0) := (others => '0');
    signal R2_t3_1 : signed(15 downto 0) := (others => '0');
    signal R2_t3_2 : signed(15 downto 0) := (others => '0');
    signal R3_00 : signed(15 downto 0) := (others => '0');
    signal R3_01 : signed(15 downto 0) := (others => '0');
    signal R3_02 : signed(15 downto 0) := (others => '0');
    signal R3_10 : signed(15 downto 0) := (others => '0');
    signal R3_11 : signed(15 downto 0) := (others => '0');
    signal R3_12 : signed(15 downto 0) := (others => '0');
    signal R3_20 : signed(15 downto 0) := (others => '0');
    signal R3_21 : signed(15 downto 0) := (others => '0');
    signal R3_22 : signed(15 downto 0) := (others => '0');
    signal R3_t0_0 : signed(15 downto 0) := (others => '0');
    signal R3_t0_1 : signed(15 downto 0) := (others => '0');
    signal R3_t0_2 : signed(15 downto 0) := (others => '0');
    signal R3_t1_0 : signed(15 downto 0) := (others => '0');
    signal R3_t1_1 : signed(15 downto 0) := (others => '0');
    signal R3_t1_2 : signed(15 downto 0) := (others => '0');
    signal R3_t2_0 : signed(15 downto 0) := (others => '0');
    signal R3_t2_1 : signed(15 downto 0) := (others => '0');
    signal R3_t2_2 : signed(15 downto 0) := (others => '0');
    signal R3_t3_0 : signed(15 downto 0) := (others => '0');
    signal R3_t3_1 : signed(15 downto 0) := (others => '0');
    signal R3_t3_2 : signed(15 downto 0) := (others => '0');
    signal R4_00 : signed(15 downto 0) := (others => '0');
    signal R4_01 : signed(15 downto 0) := (others => '0');
    signal R4_02 : signed(15 downto 0) := (others => '0');
    signal R4_10 : signed(15 downto 0) := (others => '0');
    signal R4_11 : signed(15 downto 0) := (others => '0');
    signal R4_12 : signed(15 downto 0) := (others => '0');
    signal R4_20 : signed(15 downto 0) := (others => '0');
    signal R4_21 : signed(15 downto 0) := (others => '0');
    signal R4_22 : signed(15 downto 0) := (others => '0');
    signal R4_t0_0 : signed(15 downto 0) := (others => '0');
    signal R4_t0_1 : signed(15 downto 0) := (others => '0');
    signal R4_t0_2 : signed(15 downto 0) := (others => '0');
    signal R4_t1_0 : signed(15 downto 0) := (others => '0');
    signal R4_t1_1 : signed(15 downto 0) := (others => '0');
    signal R4_t1_2 : signed(15 downto 0) := (others => '0');
    signal R4_t2_0 : signed(15 downto 0) := (others => '0');
    signal R4_t2_1 : signed(15 downto 0) := (others => '0');
    signal R4_t2_2 : signed(15 downto 0) := (others => '0');
    signal R4_t3_0 : signed(15 downto 0) := (others => '0');
    signal R4_t3_1 : signed(15 downto 0) := (others => '0');
    signal R4_t3_2 : signed(15 downto 0) := (others => '0');
    signal R5_00 : signed(15 downto 0) := (others => '0');
    signal R5_01 : signed(15 downto 0) := (others => '0');
    signal R5_02 : signed(15 downto 0) := (others => '0');
    signal R5_10 : signed(15 downto 0) := (others => '0');
    signal R5_11 : signed(15 downto 0) := (others => '0');
    signal R5_12 : signed(15 downto 0) := (others => '0');
    signal R5_20 : signed(15 downto 0) := (others => '0');
    signal R5_21 : signed(15 downto 0) := (others => '0');
    signal R5_22 : signed(15 downto 0) := (others => '0');
    signal R5_t0_0 : signed(15 downto 0) := (others => '0');
    signal R5_t0_1 : signed(15 downto 0) := (others => '0');
    signal R5_t0_2 : signed(15 downto 0) := (others => '0');
    signal R5_t1_0 : signed(15 downto 0) := (others => '0');
    signal R5_t1_1 : signed(15 downto 0) := (others => '0');
    signal R5_t1_2 : signed(15 downto 0) := (others => '0');
    signal R5_t2_0 : signed(15 downto 0) := (others => '0');
    signal R5_t2_1 : signed(15 downto 0) := (others => '0');
    signal R5_t2_2 : signed(15 downto 0) := (others => '0');
    signal R5_t3_0 : signed(15 downto 0) := (others => '0');
    signal R5_t3_1 : signed(15 downto 0) := (others => '0');
    signal R5_t3_2 : signed(15 downto 0) := (others => '0');
    signal R6_00 : signed(15 downto 0) := (others => '0');
    signal R6_01 : signed(15 downto 0) := (others => '0');
    signal R6_02 : signed(15 downto 0) := (others => '0');
    signal R6_10 : signed(15 downto 0) := (others => '0');
    signal R6_11 : signed(15 downto 0) := (others => '0');
    signal R6_12 : signed(15 downto 0) := (others => '0');
    signal R6_20 : signed(15 downto 0) := (others => '0');
    signal R6_21 : signed(15 downto 0) := (others => '0');
    signal R6_22 : signed(15 downto 0) := (others => '0');
    signal R6_t0_0 : signed(15 downto 0) := (others => '0');
    signal R6_t0_1 : signed(15 downto 0) := (others => '0');
    signal R6_t0_2 : signed(15 downto 0) := (others => '0');
    signal R6_t1_0 : signed(15 downto 0) := (others => '0');
    signal R6_t1_1 : signed(15 downto 0) := (others => '0');
    signal R6_t1_2 : signed(15 downto 0) := (others => '0');
    signal R6_t2_0 : signed(15 downto 0) := (others => '0');
    signal R6_t2_1 : signed(15 downto 0) := (others => '0');
    signal R6_t2_2 : signed(15 downto 0) := (others => '0');
    signal R6_t3_0 : signed(15 downto 0) := (others => '0');
    signal R6_t3_1 : signed(15 downto 0) := (others => '0');
    signal R6_t3_2 : signed(15 downto 0) := (others => '0');
    signal cE3 : signed(15 downto 0) := (others => '0');
    signal cE4 : signed(15 downto 0) := (others => '0');
    signal negc1 : signed(15 downto 0) := (others => '0');
    signal o2t_0a : signed(15 downto 0) := (others => '0');
    signal o2t_0b : signed(15 downto 0) := (others => '0');
    signal o2t_1a : signed(15 downto 0) := (others => '0');
    signal o2t_1b : signed(15 downto 0) := (others => '0');
    signal o2t_2a : signed(15 downto 0) := (others => '0');
    signal o2t_2b : signed(15 downto 0) := (others => '0');
    signal o4t_0 : signed(15 downto 0) := (others => '0');
    signal o4t_1 : signed(15 downto 0) := (others => '0');
    signal o4t_2 : signed(15 downto 0) := (others => '0');
    signal o6t_0 : signed(15 downto 0) := (others => '0');
    signal o6t_1 : signed(15 downto 0) := (others => '0');
    signal o6t_2 : signed(15 downto 0) := (others => '0');

begin

    process(clk, rst)
        variable mul_a, mul_b, mul_r : signed(15 downto 0);
    begin
        if rst = '1' then
            state    <= S_IDLE;
            step     <= 0;
            done_reg <= '0';
            c1_r <= (others => '0'); s1_r <= (others => '0');
            c2_r <= (others => '0'); s2_r <= (others => '0');
            c3_r <= (others => '0'); s3_r <= (others => '0');
            c4_r <= (others => '0'); s4_r <= (others => '0');
            c5_r <= (others => '0'); s5_r <= (others => '0');
            c6_r <= (others => '0'); s6_r <= (others => '0');

            -- --- reset (bloque generado) ---
            Lc2 <= (others => '0');
            Ls2 <= (others => '0');
            O2_0 <= (others => '0');
            O2_1 <= (others => '0');
            O2_2 <= (others => '0');
            O4_0 <= (others => '0');
            O4_1 <= (others => '0');
            O4_2 <= (others => '0');
            O6_0 <= (others => '0');
            O6_1 <= (others => '0');
            O6_2 <= (others => '0');
            R2_00 <= (others => '0');
            R2_01 <= (others => '0');
            R2_02 <= (others => '0');
            R2_10 <= (others => '0');
            R2_11 <= (others => '0');
            R2_12 <= (others => '0');
            R2_20 <= (others => '0');
            R2_21 <= (others => '0');
            R2_22 <= (others => '0');
            R2_t0_0 <= (others => '0');
            R2_t0_1 <= (others => '0');
            R2_t0_2 <= (others => '0');
            R2_t1_0 <= (others => '0');
            R2_t1_1 <= (others => '0');
            R2_t1_2 <= (others => '0');
            R2_t2_0 <= (others => '0');
            R2_t2_1 <= (others => '0');
            R2_t2_2 <= (others => '0');
            R2_t3_0 <= (others => '0');
            R2_t3_1 <= (others => '0');
            R2_t3_2 <= (others => '0');
            R3_00 <= (others => '0');
            R3_01 <= (others => '0');
            R3_02 <= (others => '0');
            R3_10 <= (others => '0');
            R3_11 <= (others => '0');
            R3_12 <= (others => '0');
            R3_20 <= (others => '0');
            R3_21 <= (others => '0');
            R3_22 <= (others => '0');
            R3_t0_0 <= (others => '0');
            R3_t0_1 <= (others => '0');
            R3_t0_2 <= (others => '0');
            R3_t1_0 <= (others => '0');
            R3_t1_1 <= (others => '0');
            R3_t1_2 <= (others => '0');
            R3_t2_0 <= (others => '0');
            R3_t2_1 <= (others => '0');
            R3_t2_2 <= (others => '0');
            R3_t3_0 <= (others => '0');
            R3_t3_1 <= (others => '0');
            R3_t3_2 <= (others => '0');
            R4_00 <= (others => '0');
            R4_01 <= (others => '0');
            R4_02 <= (others => '0');
            R4_10 <= (others => '0');
            R4_11 <= (others => '0');
            R4_12 <= (others => '0');
            R4_20 <= (others => '0');
            R4_21 <= (others => '0');
            R4_22 <= (others => '0');
            R4_t0_0 <= (others => '0');
            R4_t0_1 <= (others => '0');
            R4_t0_2 <= (others => '0');
            R4_t1_0 <= (others => '0');
            R4_t1_1 <= (others => '0');
            R4_t1_2 <= (others => '0');
            R4_t2_0 <= (others => '0');
            R4_t2_1 <= (others => '0');
            R4_t2_2 <= (others => '0');
            R4_t3_0 <= (others => '0');
            R4_t3_1 <= (others => '0');
            R4_t3_2 <= (others => '0');
            R5_00 <= (others => '0');
            R5_01 <= (others => '0');
            R5_02 <= (others => '0');
            R5_10 <= (others => '0');
            R5_11 <= (others => '0');
            R5_12 <= (others => '0');
            R5_20 <= (others => '0');
            R5_21 <= (others => '0');
            R5_22 <= (others => '0');
            R5_t0_0 <= (others => '0');
            R5_t0_1 <= (others => '0');
            R5_t0_2 <= (others => '0');
            R5_t1_0 <= (others => '0');
            R5_t1_1 <= (others => '0');
            R5_t1_2 <= (others => '0');
            R5_t2_0 <= (others => '0');
            R5_t2_1 <= (others => '0');
            R5_t2_2 <= (others => '0');
            R5_t3_0 <= (others => '0');
            R5_t3_1 <= (others => '0');
            R5_t3_2 <= (others => '0');
            R6_00 <= (others => '0');
            R6_01 <= (others => '0');
            R6_02 <= (others => '0');
            R6_10 <= (others => '0');
            R6_11 <= (others => '0');
            R6_12 <= (others => '0');
            R6_20 <= (others => '0');
            R6_21 <= (others => '0');
            R6_22 <= (others => '0');
            R6_t0_0 <= (others => '0');
            R6_t0_1 <= (others => '0');
            R6_t0_2 <= (others => '0');
            R6_t1_0 <= (others => '0');
            R6_t1_1 <= (others => '0');
            R6_t1_2 <= (others => '0');
            R6_t2_0 <= (others => '0');
            R6_t2_1 <= (others => '0');
            R6_t2_2 <= (others => '0');
            R6_t3_0 <= (others => '0');
            R6_t3_1 <= (others => '0');
            R6_t3_2 <= (others => '0');
            cE3 <= (others => '0');
            cE4 <= (others => '0');
            negc1 <= (others => '0');
            o2t_0a <= (others => '0');
            o2t_0b <= (others => '0');
            o2t_1a <= (others => '0');
            o2t_1b <= (others => '0');
            o2t_2a <= (others => '0');
            o2t_2b <= (others => '0');
            o4t_0 <= (others => '0');
            o4t_1 <= (others => '0');
            o4t_2 <= (others => '0');
            o6t_0 <= (others => '0');
            o6t_1 <= (others => '0');
            o6t_2 <= (others => '0');

        elsif rising_edge(clk) then

            case state is

                when S_IDLE =>
                    done_reg <= '0';
                    if start = '1' then
                        c1_r <= signed(cos1_in); s1_r <= signed(sin1_in);
                        c2_r <= signed(cos2_in); s2_r <= signed(sin2_in);
                        c3_r <= signed(cos3_in); s3_r <= signed(sin3_in);
                        c4_r <= signed(cos4_in); s4_r <= signed(sin4_in);
                        c5_r <= signed(cos5_in); s5_r <= signed(sin5_in);
                        c6_r <= signed(cos6_in); s6_r <= signed(sin6_in);
                        step  <= 0;
                        state <= S_COMPUTE;
                    end if;

                when S_COMPUTE =>
                    -- --- CASE 1: operandos (paso 0..73) ---
                    case step is
                        when 0 => mul_a := c1_r; mul_b := c2_r;
                        when 1 => mul_a := to_signed(0,16); mul_b := s2_r;
                        when 2 => mul_a := c1_r; mul_b := s2_r;
                        when 3 => mul_a := to_signed(0,16); mul_b := c2_r;
                        when 4 => mul_a := s1_r; mul_b := c2_r;
                        when 5 => mul_a := to_signed(0,16); mul_b := s2_r;
                        when 6 => mul_a := s1_r; mul_b := s2_r;
                        when 7 => mul_a := to_signed(0,16); mul_b := c2_r;
                        when 8 => mul_a := to_signed(0,16); mul_b := c2_r;
                        when 9 => mul_a := to_signed(8192,16); mul_b := s2_r;
                        when 10 => mul_a := to_signed(0,16); mul_b := s2_r;
                        when 11 => mul_a := to_signed(8192,16); mul_b := c2_r;
                        when 12 => mul_a := L2_Q13; mul_b := c2_r;
                        when 13 => mul_a := L2_Q13; mul_b := s2_r;
                        when 14 => mul_a := Lc2; mul_b := c1_r;
                        when 15 => mul_a := Ls2; mul_b := to_signed(0,16);
                        when 16 => mul_a := Lc2; mul_b := s1_r;
                        when 17 => mul_a := Ls2; mul_b := to_signed(0,16);
                        when 18 => mul_a := Lc2; mul_b := to_signed(0,16);
                        when 19 => mul_a := Ls2; mul_b := to_signed(8192,16);
                        when 20 => mul_a := R2_00; mul_b := cE3;
                        when 21 => mul_a := R2_01; mul_b := c3_r;
                        when 22 => mul_a := R2_00; mul_b := c3_r;
                        when 23 => mul_a := R2_01; mul_b := cE3;
                        when 24 => mul_a := R2_10; mul_b := cE3;
                        when 25 => mul_a := R2_11; mul_b := c3_r;
                        when 26 => mul_a := R2_10; mul_b := c3_r;
                        when 27 => mul_a := R2_11; mul_b := cE3;
                        when 28 => mul_a := R2_20; mul_b := cE3;
                        when 29 => mul_a := R2_21; mul_b := c3_r;
                        when 30 => mul_a := R2_20; mul_b := c3_r;
                        when 31 => mul_a := R2_21; mul_b := cE3;
                        when 32 => mul_a := R3_00; mul_b := cE4;
                        when 33 => mul_a := R3_01; mul_b := c4_r;
                        when 34 => mul_a := R3_00; mul_b := c4_r;
                        when 35 => mul_a := R3_01; mul_b := cE4;
                        when 36 => mul_a := R3_10; mul_b := cE4;
                        when 37 => mul_a := R3_11; mul_b := c4_r;
                        when 38 => mul_a := R3_10; mul_b := c4_r;
                        when 39 => mul_a := R3_11; mul_b := cE4;
                        when 40 => mul_a := R3_20; mul_b := cE4;
                        when 41 => mul_a := R3_21; mul_b := c4_r;
                        when 42 => mul_a := R3_20; mul_b := c4_r;
                        when 43 => mul_a := R3_21; mul_b := cE4;
                        when 44 => mul_a := LD4_Q13; mul_b := R3_02;
                        when 45 => mul_a := LD4_Q13; mul_b := R3_12;
                        when 46 => mul_a := LD4_Q13; mul_b := R3_22;
                        when 47 => mul_a := R4_00; mul_b := c5_r;
                        when 48 => mul_a := R4_01; mul_b := s5_r;
                        when 49 => mul_a := R4_00; mul_b := s5_r;
                        when 50 => mul_a := R4_01; mul_b := c5_r;
                        when 51 => mul_a := R4_10; mul_b := c5_r;
                        when 52 => mul_a := R4_11; mul_b := s5_r;
                        when 53 => mul_a := R4_10; mul_b := s5_r;
                        when 54 => mul_a := R4_11; mul_b := c5_r;
                        when 55 => mul_a := R4_20; mul_b := c5_r;
                        when 56 => mul_a := R4_21; mul_b := s5_r;
                        when 57 => mul_a := R4_20; mul_b := s5_r;
                        when 58 => mul_a := R4_21; mul_b := c5_r;
                        when 59 => mul_a := R5_00; mul_b := c6_r;
                        when 60 => mul_a := R5_01; mul_b := s6_r;
                        when 61 => mul_a := R5_00; mul_b := s6_r;
                        when 62 => mul_a := R5_01; mul_b := c6_r;
                        when 63 => mul_a := R5_10; mul_b := c6_r;
                        when 64 => mul_a := R5_11; mul_b := s6_r;
                        when 65 => mul_a := R5_10; mul_b := s6_r;
                        when 66 => mul_a := R5_11; mul_b := c6_r;
                        when 67 => mul_a := R5_20; mul_b := c6_r;
                        when 68 => mul_a := R5_21; mul_b := s6_r;
                        when 69 => mul_a := R5_20; mul_b := s6_r;
                        when 70 => mul_a := R5_21; mul_b := c6_r;
                        when 71 => mul_a := LD6_Q13; mul_b := R5_02;
                        when 72 => mul_a := LD6_Q13; mul_b := R5_12;
                        when others => mul_a := LD6_Q13; mul_b := R5_22; -- step 73
                    end case;

                    mul_r := mul_q13(mul_a, mul_b);  -- UNICA llamada a mul_q13 de todo el archivo

                    -- --- CASE 2: destino ---
                    case step is
                        when 0 => R2_t0_0 <= mul_r; negc1 <= -c1_r; cE3 <= -s3_r; cE4 <= -s4_r;
                        when 1 => R2_00 <= mul_r + R2_t0_0;
                        when 2 => R2_t2_0 <= mul_r;
                        when 3 => R2_01 <= mul_r - R2_t2_0; R2_02 <= s1_r;
                        when 4 => R2_t0_1 <= mul_r;
                        when 5 => R2_10 <= mul_r + R2_t0_1;
                        when 6 => R2_t2_1 <= mul_r;
                        when 7 => R2_11 <= mul_r - R2_t2_1; R2_12 <= negc1;
                        when 8 => R2_t0_2 <= mul_r;
                        when 9 => R2_20 <= mul_r + R2_t0_2;
                        when 10 => R2_t2_2 <= mul_r;
                        when 11 => R2_21 <= mul_r - R2_t2_2; R2_22 <= to_signed(0,16);
                        when 12 => Lc2 <= mul_r;
                        when 13 => Ls2 <= mul_r;
                        when 14 => o2t_0a <= mul_r;
                        when 15 => O2_0 <= mul_r + o2t_0a + to_signed(0,16);
                        when 16 => o2t_1a <= mul_r;
                        when 17 => O2_1 <= mul_r + o2t_1a + to_signed(0,16);
                        when 18 => o2t_2a <= mul_r;
                        when 19 => O2_2 <= mul_r + o2t_2a + L1_Q13;
                        when 20 => R3_t0_0 <= mul_r;
                        when 21 => R3_00 <= mul_r + R3_t0_0; R3_01 <= R2_02;
                        when 22 => R3_t2_0 <= mul_r;
                        when 23 => R3_02 <= R3_t2_0 - (mul_r);
                        when 24 => R3_t0_1 <= mul_r;
                        when 25 => R3_10 <= mul_r + R3_t0_1; R3_11 <= R2_12;
                        when 26 => R3_t2_1 <= mul_r;
                        when 27 => R3_12 <= R3_t2_1 - (mul_r);
                        when 28 => R3_t0_2 <= mul_r;
                        when 29 => R3_20 <= mul_r + R3_t0_2; R3_21 <= R2_22;
                        when 30 => R3_t2_2 <= mul_r;
                        when 31 => R3_22 <= R3_t2_2 - (mul_r);
                        when 32 => R4_t0_0 <= mul_r;
                        when 33 => R4_00 <= mul_r + R4_t0_0; R4_01 <= R3_02;
                        when 34 => R4_t2_0 <= mul_r;
                        when 35 => R4_02 <= R4_t2_0 - (mul_r);
                        when 36 => R4_t0_1 <= mul_r;
                        when 37 => R4_10 <= mul_r + R4_t0_1; R4_11 <= R3_12;
                        when 38 => R4_t2_1 <= mul_r;
                        when 39 => R4_12 <= R4_t2_1 - (mul_r);
                        when 40 => R4_t0_2 <= mul_r;
                        when 41 => R4_20 <= mul_r + R4_t0_2; R4_21 <= R3_22;
                        when 42 => R4_t2_2 <= mul_r;
                        when 43 => R4_22 <= R4_t2_2 - (mul_r);
                        when 44 => O4_0 <= mul_r + O2_0;
                        when 45 => O4_1 <= mul_r + O2_1;
                        when 46 => O4_2 <= mul_r + O2_2;
                        when 47 => R5_t0_0 <= mul_r;
                        when 48 => R5_00 <= mul_r + R5_t0_0; R5_01 <= -R4_02;
                        when 49 => R5_t2_0 <= mul_r;
                        when 50 => R5_02 <= mul_r - R5_t2_0;
                        when 51 => R5_t0_1 <= mul_r;
                        when 52 => R5_10 <= mul_r + R5_t0_1; R5_11 <= -R4_12;
                        when 53 => R5_t2_1 <= mul_r;
                        when 54 => R5_12 <= mul_r - R5_t2_1;
                        when 55 => R5_t0_2 <= mul_r;
                        when 56 => R5_20 <= mul_r + R5_t0_2; R5_21 <= -R4_22;
                        when 57 => R5_t2_2 <= mul_r;
                        when 58 => R5_22 <= mul_r - R5_t2_2;
                        when 59 => R6_t0_0 <= mul_r;
                        when 60 => R6_00 <= mul_r + R6_t0_0;
                        when 61 => R6_t2_0 <= mul_r;
                        when 62 => R6_01 <= mul_r - R6_t2_0; R6_02 <= R5_02;
                        when 63 => R6_t0_1 <= mul_r;
                        when 64 => R6_10 <= mul_r + R6_t0_1;
                        when 65 => R6_t2_1 <= mul_r;
                        when 66 => R6_11 <= mul_r - R6_t2_1; R6_12 <= R5_12;
                        when 67 => R6_t0_2 <= mul_r;
                        when 68 => R6_20 <= mul_r + R6_t0_2;
                        when 69 => R6_t2_2 <= mul_r;
                        when 70 => R6_21 <= mul_r - R6_t2_2; R6_22 <= R5_22;
                        when 71 => O6_0 <= mul_r + O4_0;
                        when 72 => O6_1 <= mul_r + O4_1;
                        when others =>  -- step 73, ultimo paso
                            O6_2     <= mul_r + O4_2;
                            done_reg <= '1';
                            state    <= S_IDLE;
                    end case;

                    if step < 73 then
                        step <= step + 1;
                    end if;

            end case;
        end if;
    end process;

    -- salidas: R11=R6_00 R21=R6_10 R31=R6_20 R32=R6_21 R33=R6_22 ; x,y,z=O6
    r11_out <= std_logic_vector(R6_00);
    r21_out <= std_logic_vector(R6_10);
    r31_out <= std_logic_vector(R6_20);
    r32_out <= std_logic_vector(R6_21);
    r33_out <= std_logic_vector(R6_22);

    x_out <= std_logic_vector(O6_0);
    y_out <= std_logic_vector(O6_1);
    z_out <= std_logic_vector(O6_2);

    done <= done_reg;

end rtl;
