-- =============================================================
--  cordic_seq6.vhd
--  Paso 2 — Cinematica Directa por METODO GEOMETRICO (brazo de 6 GDL)
--
--  ================== REVISION 2 (correccion del profesor) ==================
--  El metodo geometrico recursivo (Inversa2R.pdf) usa el theta_i PROPIO de
--  cada fila de la tabla DH, no angulos acumulados (phi2, phi23) -- por eso
--  esta version ya NO necesita angle_sum_gen.vhd: corre CORDIC sobre
--  theta1..theta6 CRUDOS, una pasada por articulacion, sin sumar nada.
--
--  El desfase +pi/2 de las filas 3 y 4 (theta3+pi/2, theta4+pi/2) tampoco
--  hace falta pasarlo por el CORDIC: cos(x+pi/2)=-sin(x) y sin(x+pi/2)=cos(x)
--  son identidades trigonometricas exactas -- el modulo siguiente
--  (fk_recursivo_core) arma esas dos con un simple cambio de signo de
--  cos3/sin3 y cos4/sin4, sin gastar CORDIC ni multiplicador extra en eso.
--  ==========================================================================
--
--  Pasadas: theta1, theta2, theta3, theta4, theta5, theta6 (crudos).
--
--  No modifica cordic_sincos_16.vhd (se instancia tal cual).
--  Latencia total ~ 6*(N_ITER+1) + overhead FSM = ~78-84 ciclos.
--  Universidad Militar Nueva Granada
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity cordic_seq6 is
    Port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        start      : in  std_logic;

        theta1_in  : in  std_logic_vector(15 downto 0);
        theta2_in  : in  std_logic_vector(15 downto 0);
        theta3_in  : in  std_logic_vector(15 downto 0);
        theta4_in  : in  std_logic_vector(15 downto 0);
        theta5_in  : in  std_logic_vector(15 downto 0);
        theta6_in  : in  std_logic_vector(15 downto 0);

        cos1_out   : out std_logic_vector(15 downto 0);
        sin1_out   : out std_logic_vector(15 downto 0);
        cos2_out   : out std_logic_vector(15 downto 0);
        sin2_out   : out std_logic_vector(15 downto 0);
        cos3_out   : out std_logic_vector(15 downto 0);
        sin3_out   : out std_logic_vector(15 downto 0);
        cos4_out   : out std_logic_vector(15 downto 0);
        sin4_out   : out std_logic_vector(15 downto 0);
        cos5_out   : out std_logic_vector(15 downto 0);
        sin5_out   : out std_logic_vector(15 downto 0);
        cos6_out   : out std_logic_vector(15 downto 0);
        sin6_out   : out std_logic_vector(15 downto 0);

        done       : out std_logic
    );
end cordic_seq6;

architecture rtl of cordic_seq6 is

    -- CORDIC unico, reutilizado (no se modifica, se instancia tal cual)
    component cordic_sincos_16 is
        Port (
            clk      : in  std_logic;
            rst      : in  std_logic;
            start    : in  std_logic;
            angle_in : in  std_logic_vector(15 downto 0);
            sin_out  : out std_logic_vector(15 downto 0);
            cos_out  : out std_logic_vector(15 downto 0);
            done     : out std_logic
        );
    end component;

    type state_t is (S_IDLE, S_W1, S_W2, S_W3, S_W4, S_W5, S_W6);
    signal state : state_t := S_IDLE;

    signal cordic_start : std_logic := '0';
    signal cordic_done  : std_logic;
    signal angle_mux    : std_logic_vector(15 downto 0) := (others => '0');
    signal cordic_cos   : std_logic_vector(15 downto 0);
    signal cordic_sin   : std_logic_vector(15 downto 0);

    signal cos1_r, sin1_r : std_logic_vector(15 downto 0) := (others => '0');
    signal cos2_r, sin2_r : std_logic_vector(15 downto 0) := (others => '0');
    signal cos3_r, sin3_r : std_logic_vector(15 downto 0) := (others => '0');
    signal cos4_r, sin4_r : std_logic_vector(15 downto 0) := (others => '0');
    signal cos5_r, sin5_r : std_logic_vector(15 downto 0) := (others => '0');
    signal cos6_r, sin6_r : std_logic_vector(15 downto 0) := (others => '0');
    signal done_reg       : std_logic := '0';

    -- Deteccion de flanco de subida en start (mismo truco de siempre)
    signal start_prev : std_logic := '0';
    signal start_edge : std_logic;

begin

    start_edge <= start and not start_prev;

    U_CORDIC : cordic_sincos_16
        port map(
            clk      => clk,
            rst      => rst,
            start    => cordic_start,
            angle_in => angle_mux,
            sin_out  => cordic_sin,
            cos_out  => cordic_cos,
            done     => cordic_done
        );

    process(clk, rst)
    begin
        if rst = '1' then
            state        <= S_IDLE;
            cordic_start <= '0';
            done_reg     <= '0';
            start_prev   <= '0';
            cos1_r <= (others => '0'); sin1_r <= (others => '0');
            cos2_r <= (others => '0'); sin2_r <= (others => '0');
            cos3_r <= (others => '0'); sin3_r <= (others => '0');
            cos4_r <= (others => '0'); sin4_r <= (others => '0');
            cos5_r <= (others => '0'); sin5_r <= (others => '0');
            cos6_r <= (others => '0'); sin6_r <= (others => '0');

        elsif rising_edge(clk) then

            cordic_start <= '0';  -- pulso de 1 ciclo por defecto
            start_prev   <= start;

            case state is

                when S_IDLE =>
                    done_reg <= '0';
                    if start_edge = '1' then
                        angle_mux    <= theta1_in;
                        cordic_start <= '1';
                        state        <= S_W1;
                    end if;

                when S_W1 =>
                    if cordic_done = '1' then
                        cos1_r       <= cordic_cos;
                        sin1_r       <= cordic_sin;
                        angle_mux    <= theta2_in;
                        cordic_start <= '1';
                        state        <= S_W2;
                    end if;

                when S_W2 =>
                    if cordic_done = '1' then
                        cos2_r       <= cordic_cos;
                        sin2_r       <= cordic_sin;
                        angle_mux    <= theta3_in;
                        cordic_start <= '1';
                        state        <= S_W3;
                    end if;

                when S_W3 =>
                    if cordic_done = '1' then
                        cos3_r       <= cordic_cos;
                        sin3_r       <= cordic_sin;
                        angle_mux    <= theta4_in;
                        cordic_start <= '1';
                        state        <= S_W4;
                    end if;

                when S_W4 =>
                    if cordic_done = '1' then
                        cos4_r       <= cordic_cos;
                        sin4_r       <= cordic_sin;
                        angle_mux    <= theta5_in;
                        cordic_start <= '1';
                        state        <= S_W5;
                    end if;

                when S_W5 =>
                    if cordic_done = '1' then
                        cos5_r       <= cordic_cos;
                        sin5_r       <= cordic_sin;
                        angle_mux    <= theta6_in;
                        cordic_start <= '1';
                        state        <= S_W6;
                    end if;

                when S_W6 =>
                    if cordic_done = '1' then
                        cos6_r   <= cordic_cos;
                        sin6_r   <= cordic_sin;
                        done_reg <= '1';
                        state    <= S_IDLE;
                    end if;

            end case;
        end if;
    end process;

    cos1_out <= cos1_r; sin1_out <= sin1_r;
    cos2_out <= cos2_r; sin2_out <= sin2_r;
    cos3_out <= cos3_r; sin3_out <= sin3_r;
    cos4_out <= cos4_r; sin4_out <= sin4_r;
    cos5_out <= cos5_r; sin5_out <= sin5_r;
    cos6_out <= cos6_r; sin6_out <= sin6_r;
    done     <= done_reg;

end rtl;
