-- =============================================================
--  atan2_seq3.vhd
--  Paso 4d — Orientacion (Roll/Pitch/Yaw) por METODO GEOMETRICO
--  Reutiliza UN SOLO cordic_atan2_16 en 3 pasadas secuenciales:
--
--    Pasada 1: x=R11, y=R21        -> angle=Yaw   , mag=rho
--    Pasada 2: x=rho, y=-R31       -> angle=Pitch          (usa rho de la pasada 1)
--    Pasada 3: x=R33, y=R32        -> angle=Roll
--
--  La pasada 2 depende del resultado (mag/rho) de la pasada 1 —
--  por eso no se puede paralelizar como cordic_seq5, tiene que
--  ser estrictamente secuencial.
--
--  No modifica cordic_atan2_16.vhd (se instancia tal cual).
--  Latencia total ~ 3*(N_ITER+1) + overhead FSM = ~40-42 ciclos.
--  Universidad Militar Nueva Granada
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity atan2_seq3 is
    Port (
        clk        : in  std_logic;
        rst        : in  std_logic;
        start      : in  std_logic;

        r11_in     : in  std_logic_vector(15 downto 0);
        r21_in     : in  std_logic_vector(15 downto 0);
        r31_in     : in  std_logic_vector(15 downto 0);
        r32_in     : in  std_logic_vector(15 downto 0);
        r33_in     : in  std_logic_vector(15 downto 0);

        yaw_out    : out std_logic_vector(15 downto 0);
        pitch_out  : out std_logic_vector(15 downto 0);
        roll_out   : out std_logic_vector(15 downto 0);

        done       : out std_logic
    );
end atan2_seq3;

architecture rtl of atan2_seq3 is

    -- CORDIC vectoring unico, reutilizado (no se modifica, se instancia tal cual)
    component cordic_atan2_16 is
        Port (
            clk       : in  std_logic;
            rst       : in  std_logic;
            start     : in  std_logic;
            x_in      : in  std_logic_vector(15 downto 0);
            y_in      : in  std_logic_vector(15 downto 0);
            angle_out : out std_logic_vector(15 downto 0);
            mag_out   : out std_logic_vector(15 downto 0);
            done      : out std_logic
        );
    end component;

    type state_t is (S_IDLE, S_W1, S_W2, S_W3);
    signal state : state_t := S_IDLE;

    signal cordic_start : std_logic := '0';
    signal cordic_done  : std_logic;
    signal x_mux, y_mux : std_logic_vector(15 downto 0) := (others => '0');
    signal c_angle      : std_logic_vector(15 downto 0);
    signal c_mag        : std_logic_vector(15 downto 0);

    signal yaw_r, pitch_r, roll_r : std_logic_vector(15 downto 0) := (others => '0');
    signal done_reg                : std_logic := '0';

    -- Deteccion de flanco de subida en start (mismo truco que cordic_seq4/5)
    signal start_prev : std_logic := '0';
    signal start_edge : std_logic;

begin

    start_edge <= start and not start_prev;

    U_ATAN2 : cordic_atan2_16
        port map(
            clk       => clk,
            rst       => rst,
            start     => cordic_start,
            x_in      => x_mux,
            y_in      => y_mux,
            angle_out => c_angle,
            mag_out   => c_mag,
            done      => cordic_done
        );

    process(clk, rst)
    begin
        if rst = '1' then
            state        <= S_IDLE;
            cordic_start <= '0';
            done_reg     <= '0';
            start_prev   <= '0';
            x_mux        <= (others => '0');
            y_mux        <= (others => '0');
            yaw_r        <= (others => '0');
            pitch_r      <= (others => '0');
            roll_r       <= (others => '0');

        elsif rising_edge(clk) then

            cordic_start <= '0';  -- pulso de 1 ciclo por defecto
            start_prev   <= start;

            case state is

                when S_IDLE =>
                    done_reg <= '0';
                    if start_edge = '1' then
                        x_mux        <= r11_in;
                        y_mux        <= r21_in;
                        cordic_start <= '1';
                        state        <= S_W1;
                    end if;

                when S_W1 =>
                    if cordic_done = '1' then
                        yaw_r        <= c_angle;
                        x_mux        <= c_mag;                              -- rho de la pasada 1
                        y_mux        <= std_logic_vector(-signed(r31_in));  -- -R31
                        cordic_start <= '1';
                        state        <= S_W2;
                    end if;

                when S_W2 =>
                    if cordic_done = '1' then
                        pitch_r      <= c_angle;
                        x_mux        <= r33_in;
                        y_mux        <= r32_in;
                        cordic_start <= '1';
                        state        <= S_W3;
                    end if;

                when S_W3 =>
                    if cordic_done = '1' then
                        roll_r   <= c_angle;
                        done_reg <= '1';
                        state    <= S_IDLE;
                    end if;

            end case;
        end if;
    end process;

    yaw_out   <= yaw_r;
    pitch_out <= pitch_r;
    roll_out  <= roll_r;
    done      <= done_reg;

end rtl;
