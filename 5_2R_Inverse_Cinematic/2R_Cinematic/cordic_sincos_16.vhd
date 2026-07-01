-- =============================================================
--  cordic_sincos_16.vhd
--  CORDIC modo rotacion - Sin y Cos en Q2.13 (16 bits)
--
--  CORRECCION: reescrito con un solo proceso secuencial en
--  lugar de generate, para garantizar propagacion correcta
--  de valid_pipe en ModelSim/Quartus 18.1
--
--  Entradas:
--    angle_in : angulo en Q2.13 (radianes x 8192), rango +/-pi
--  Salidas:
--    sin_out  : sin(angle_in) en Q2.13
--    cos_out  : cos(angle_in) en Q2.13
--    done     : pulso 1 ciclo cuando resultado valido
--
--  Latencia: N_ITER + 1 = 13 ciclos
--
--  Universidad Militar Nueva Granada
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.cordic_pkg.ALL;

entity cordic_sincos_16 is
    Port (
        clk      : in  std_logic;
        rst      : in  std_logic;
        start    : in  std_logic;
        angle_in : in  signed(15 downto 0);
        sin_out  : out signed(15 downto 0);
        cos_out  : out signed(15 downto 0);
        done     : out std_logic
    );
end cordic_sincos_16;

architecture rtl of cordic_sincos_16 is

    -- Arrays de pipeline: indice 0 = entrada, N_ITER = salida
    type pipe_array is array (0 to N_ITER) of signed(15 downto 0);

    signal x_pipe     : pipe_array := (others => (others => '0'));
    signal y_pipe     : pipe_array := (others => (others => '0'));
    signal z_pipe     : pipe_array := (others => (others => '0'));
    signal valid_pipe : std_logic_vector(N_ITER downto 0) := (others => '0');
    signal fcos_pipe  : std_logic_vector(N_ITER downto 0) := (others => '0');
    signal fsin_pipe  : std_logic_vector(N_ITER downto 0) := (others => '0');

    -- PI/2 en Q2.13
    constant SC_PI2_POS : signed(15 downto 0) := to_signed( 12868, 16);
    constant SC_PI2_NEG : signed(15 downto 0) := to_signed(-12868, 16);
    -- PI en Q2.13
    constant SC_PI_Q13  : signed(15 downto 0) := to_signed( 25736, 16);

begin

    -- ----------------------------------------------------------
    --  Proceso unico: etapa 0 (carga) en ciclo 0,
    --  etapas 1..N_ITER (rotaciones) en ciclos siguientes.
    --  Se usan variables locales para calcular los shifts.
    -- ----------------------------------------------------------
    process(clk, rst)
        variable x_sh : signed(15 downto 0);
        variable y_sh : signed(15 downto 0);
        variable ang  : signed(15 downto 0);
    begin
        if rst = '1' then
            x_pipe     <= (others => (others => '0'));
            y_pipe     <= (others => (others => '0'));
            z_pipe     <= (others => (others => '0'));
            valid_pipe <= (others => '0');
            fcos_pipe  <= (others => '0');
            fsin_pipe  <= (others => '0');

        elsif rising_edge(clk) then

            -- --------------------------------------------------
            --  Etapa 0: carga y reduccion de cuadrante
            -- --------------------------------------------------
            valid_pipe(0) <= start;
            fcos_pipe(0)  <= '0';
            fsin_pipe(0)  <= '0';

            if start = '1' then
                -- x inicial = K/2 (se compensa con shift_left al final)
                x_pipe(0) <= shift_right(CORDIC_K, 1);
                y_pipe(0) <= (others => '0');

                -- Reduccion de cuadrante
                if angle_in > SC_PI2_POS then
                    -- Cuadrante 2: usar pi - theta, negar cos
                    ang          := SC_PI_Q13 - angle_in;
                    fcos_pipe(0) <= '1';
                    fsin_pipe(0) <= '0';
                elsif angle_in < SC_PI2_NEG then
                    -- Cuadrante 3: usar -pi - theta, negar ambos
                    ang          := -SC_PI_Q13 - angle_in;
                    fcos_pipe(0) <= '1';
                    fsin_pipe(0) <= '1';
                else
                    -- Cuadrante 1 y 4: sin cambio
                    ang          := angle_in;
                    fcos_pipe(0) <= '0';
                    fsin_pipe(0) <= '0';
                end if;
                z_pipe(0) <= ang;
            end if;

            -- --------------------------------------------------
            --  Etapas 1..N_ITER: rotaciones CORDIC
            --  x(i) = x(i-1) - d * y(i-1) * 2^-(i-1)
            --  y(i) = y(i-1) + d * x(i-1) * 2^-(i-1)
            --  z(i) = z(i-1) - d * arctan(2^-(i-1))
            --  donde d = +1 si z(i-1) >= 0, -1 si no
            -- --------------------------------------------------
            for i in 1 to N_ITER loop
                valid_pipe(i) <= valid_pipe(i-1);
                fcos_pipe(i)  <= fcos_pipe(i-1);
                fsin_pipe(i)  <= fsin_pipe(i-1);

                x_sh := shift_right(x_pipe(i-1), i-1);
                y_sh := shift_right(y_pipe(i-1), i-1);

                if z_pipe(i-1) >= 0 then
                    x_pipe(i) <= x_pipe(i-1) - y_sh;
                    y_pipe(i) <= y_pipe(i-1) + x_sh;
                    z_pipe(i) <= z_pipe(i-1) - ATAN_TABLE(i-1);
                else
                    x_pipe(i) <= x_pipe(i-1) + y_sh;
                    y_pipe(i) <= y_pipe(i-1) - x_sh;
                    z_pipe(i) <= z_pipe(i-1) + ATAN_TABLE(i-1);
                end if;
            end loop;

        end if;
    end process;

    -- ----------------------------------------------------------
    --  Salidas: aplicar correcciones de cuadrante y factor x2
    -- ----------------------------------------------------------
    cos_out <= -shift_left(x_pipe(N_ITER), 1)
               when fcos_pipe(N_ITER) = '1'
               else shift_left(x_pipe(N_ITER), 1);

    sin_out <= -shift_left(y_pipe(N_ITER), 1)
               when fsin_pipe(N_ITER) = '1'
               else shift_left(y_pipe(N_ITER), 1);

    done <= valid_pipe(N_ITER);

end rtl;