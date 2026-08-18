-- =============================================================
--  fk_geom_core.vhd
--  Posicion final (x,y,z) del efector — METODO GEOMETRICO
--  Brazo de 6 GDL real (reemplaza al fk_geom_core.vhd del 5R+gripper)
--
--  ================== REVISION 2 (correccion de recursos) ==================
--  Igual que mat_r06_elems.vhd (ver ese archivo para el analisis
--  completo): la primera version disparaba hasta 7 multiplicaciones
--  en paralelo en un mismo ciclo. Aqui el numero es menor (9 en
--  total) pero se corrige con el mismo criterio, un solo
--  multiplicador reutilizado en 9 pasos secuenciales, para no seguir
--  sumando presion sobre los 30 "elementos de 9 bits" del EP4CE6 que
--  ya se agotan con mat_r06_elems solo.
--  ==========================================================================
--
--    Parte A (posicion del sistema 4, igual espiritu que el brazo de
--    5R pero con Ld4 en vez de L45, sin termino phi234):
--      r4 = L2*cos2 + Ld4*cos23
--      z4 = L1 + L2*sin2 + Ld4*sin23
--      x4 = r4*cos1 ; y4 = r4*sin1
--
--    Parte B (aporte de la muñeca, Ld6 en la direccion de Z6 =
--    columna 3 de R0_6, que llega ya calculada como r13,r23,r33
--    desde mat_r06_elems):
--      x = x4 + Ld6*r13
--      y = y4 + Ld6*r23
--      z = z4 + Ld6*r33
--
--  Latencia: 9 pasos de multiplicacion + 1 de combinacion final +
--  1 de latch = ~11 ciclos (antes eran 3 con multiplicadores en
--  paralelo).
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

        r13_in     : in  std_logic_vector(15 downto 0);  -- columna 3 de R0_6 (de mat_r06_elems)
        r23_in     : in  std_logic_vector(15 downto 0);
        r33_in     : in  std_logic_vector(15 downto 0);

        x_out      : out std_logic_vector(15 downto 0);
        y_out      : out std_logic_vector(15 downto 0);
        z_out      : out std_logic_vector(15 downto 0);

        done       : out std_logic
    );
end fk_geom_core;

architecture rtl of fk_geom_core is

    type state_t is (S_IDLE, S_COMPUTE, S_COMBINE);
    signal state : state_t := S_IDLE;
    signal step  : integer range 0 to 8 := 0;

    signal c1_r, s1_r, c2_r, s2_r, c23_r, s23_r : signed(15 downto 0) := (others => '0');
    signal r13_r, r23_r, r33_r                  : signed(15 downto 0) := (others => '0');

    signal pr1, pz1              : signed(15 downto 0) := (others => '0');
    signal r4_r, z4_r            : signed(15 downto 0) := (others => '0');
    signal x4_r, y4_r            : signed(15 downto 0) := (others => '0');
    signal wrist_x_r, wrist_y_r  : signed(15 downto 0) := (others => '0');
    signal wrist_z_r             : signed(15 downto 0) := (others => '0');

    signal x_r, y_r, z_r : signed(15 downto 0) := (others => '0');
    signal done_reg      : std_logic := '0';

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
            c23_r <= (others => '0'); s23_r <= (others => '0');
            r13_r <= (others => '0'); r23_r <= (others => '0'); r33_r <= (others => '0');
            pr1 <= (others => '0'); pz1 <= (others => '0');
            r4_r <= (others => '0'); z4_r <= (others => '0');
            x4_r <= (others => '0'); y4_r <= (others => '0');
            wrist_x_r <= (others => '0'); wrist_y_r <= (others => '0'); wrist_z_r <= (others => '0');
            x_r <= (others => '0'); y_r <= (others => '0'); z_r <= (others => '0');

        elsif rising_edge(clk) then

            case state is

                when S_IDLE =>
                    done_reg <= '0';
                    if start = '1' then
                        c1_r  <= signed(cos1_in);  s1_r  <= signed(sin1_in);
                        c2_r  <= signed(cos2_in);  s2_r  <= signed(sin2_in);
                        c23_r <= signed(cos23_in); s23_r <= signed(sin23_in);
                        r13_r <= signed(r13_in); r23_r <= signed(r23_in); r33_r <= signed(r33_in);
                        step  <= 0;
                        state <= S_COMPUTE;
                    end if;

                when S_COMPUTE =>
                    -- ---- seleccion de operandos (unico multiplicador reutilizado) ----
                    case step is
                        when 0 => mul_a := L2_Q13; mul_b := c2_r;
                        when 1 => mul_a := LD4_Q13; mul_b := c23_r;
                        when 2 => mul_a := L2_Q13; mul_b := s2_r;
                        when 3 => mul_a := LD4_Q13; mul_b := s23_r;
                        when 4 => mul_a := r4_r; mul_b := c1_r;
                        when 5 => mul_a := r4_r; mul_b := s1_r;
                        when 6 => mul_a := LD6_Q13; mul_b := r13_r;
                        when 7 => mul_a := LD6_Q13; mul_b := r23_r;
                        when others => mul_a := LD6_Q13; mul_b := r33_r;  -- step 8
                    end case;

                    mul_r := mul_q13(mul_a, mul_b);  -- UNICA llamada a mul_q13 de todo el archivo

                    -- ---- ruteo del resultado al destino de este paso ----
                    case step is
                        when 0 => pr1  <= mul_r;
                        when 1 => r4_r <= pr1 + mul_r;
                        when 2 => pz1  <= mul_r;
                        when 3 => z4_r <= L1_Q13 + pz1 + mul_r;
                        when 4 => x4_r <= mul_r;
                        when 5 => y4_r <= mul_r;
                        when 6 => wrist_x_r <= mul_r;
                        when 7 => wrist_y_r <= mul_r;
                        when others => wrist_z_r <= mul_r;  -- step 8
                    end case;

                    if step < 8 then
                        step <= step + 1;
                    else
                        state <= S_COMBINE;
                    end if;

                when S_COMBINE =>
                    -- combinacion final: solo sumas, sin multiplicar
                    x_r      <= x4_r + wrist_x_r;
                    y_r      <= y4_r + wrist_y_r;
                    z_r      <= z4_r + wrist_z_r;
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
