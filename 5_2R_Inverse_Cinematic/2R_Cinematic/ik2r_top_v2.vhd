-- =============================================================
--  ik2r_top_v2.vhd
--  Top-Level Cinemática Inversa Robot 2R
--  Interfaz de 16 bits directa — L1, L2 son puertos dinámicos
--
--  Mejoras respecto a ik2r_top.vhd original:
--    - L1, L2 son puertos directos de 16 bits (no byte-a-byte)
--      → el usuario los puede cambiar en tiempo real
--    - Px, Py también como puertos de 16 bits + strobe
--    - Usa ik2r_pipeline_core (pipeline completo, ~65 ciclos)
--    - Salida directa de 16 bits para theta1, theta2
--    - También mantiene mux de salida byte-a-byte para
--      compatibilidad con sistemas de 8 bits
--
--  Protocolo:
--    1. Conectar L1, L2 continuamente (se actualizan en cada start)
--    2. Escribir px_in, py_in
--    3. Pulsar start='1' durante 1 ciclo
--    4. Esperar done='1' (~66 ciclos después)
--    5. Leer theta1_out, theta2_out directamente
--       O leer data_out con out_sel
--
--  Formato Q2.13: real = raw / 8192.0
--    Ejemplo: θ = 1.5708 rad → raw = 12868
--    Ejemplo: L1 = 0.5 m    → raw = 4096
--
--  Universidad Militar Nueva Granada
-- =============================================================
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.ik_pkg.ALL;

entity ik2r_top_v2 is
    Port (
        clk        : in  std_logic;
        rst        : in  std_logic;

        -- -------------------------------------------------------
        --  Entradas — todas en Q2.13 (signed 16 bits)
        -- -------------------------------------------------------
        px_in      : in  signed(15 downto 0);   -- coord X efector
        py_in      : in  signed(15 downto 0);   -- coord Y efector
        l1_in      : in  signed(15 downto 0);   -- longitud eslabón 1 (dinámica)
        l2_in      : in  signed(15 downto 0);   -- longitud eslabón 2 (dinámica)
        start      : in  std_logic;             -- pulso inicio

        -- -------------------------------------------------------
        --  Salidas directas Q2.13
        -- -------------------------------------------------------
        theta1_out : out signed(15 downto 0);
        theta2_out : out signed(15 downto 0);
        done       : out std_logic;
        error_out  : out std_logic;

        -- -------------------------------------------------------
        --  Salida byte a byte (compatible con sistemas de 8 bits)
        --  out_sel: "00"=θ1 LSB, "01"=θ1 MSB
        --           "10"=θ2 LSB, "11"=θ2 MSB
        -- -------------------------------------------------------
        out_sel    : in  std_logic_vector(1 downto 0);
        data_out   : out std_logic_vector(7 downto 0)
    );
end ik2r_top_v2;

architecture rtl of ik2r_top_v2 is

    component ik2r_pipeline_core is
        Port (
            clk        : in  std_logic;
            rst        : in  std_logic;
            start      : in  std_logic;
            px_in      : in  signed(15 downto 0);
            py_in      : in  signed(15 downto 0);
            l1_in      : in  signed(15 downto 0);
            l2_in      : in  signed(15 downto 0);
            theta1_out : out signed(15 downto 0);
            theta2_out : out signed(15 downto 0);
            done       : out std_logic;
            error_out  : out std_logic
        );
    end component;

    signal t1_s     : signed(15 downto 0);
    signal t2_s     : signed(15 downto 0);
    signal done_s   : std_logic;
    signal error_s  : std_logic;

    -- Latches para salida estable
    signal t1_latch : signed(15 downto 0) := (others => '0');
    signal t2_latch : signed(15 downto 0) := (others => '0');
    signal done_lat : std_logic := '0';
    signal err_lat  : std_logic := '0';

begin

    U_CORE : ik2r_pipeline_core port map (
        clk        => clk,
        rst        => rst,
        start      => start,
        px_in      => px_in,
        py_in      => py_in,
        l1_in      => l1_in,
        l2_in      => l2_in,
        theta1_out => t1_s,
        theta2_out => t2_s,
        done       => done_s,
        error_out  => error_s
    );

    -- Latch de resultados al pulso done
    process(clk, rst)
    begin
        if rst = '1' then
            t1_latch <= (others => '0');
            t2_latch <= (others => '0');
            done_lat <= '0';
            err_lat  <= '0';
        elsif rising_edge(clk) then
            done_lat <= '0';
            if done_s = '1' then
                t1_latch <= t1_s;
                t2_latch <= t2_s;
                err_lat  <= error_s;
                done_lat <= '1';
            end if;
        end if;
    end process;

    -- Mux de salida byte-a-byte
    process(out_sel, t1_latch, t2_latch)
    begin
        case out_sel is
            when "00"   => data_out <= std_logic_vector(t1_latch(7  downto 0));
            when "01"   => data_out <= std_logic_vector(t1_latch(15 downto 8));
            when "10"   => data_out <= std_logic_vector(t2_latch(7  downto 0));
            when "11"   => data_out <= std_logic_vector(t2_latch(15 downto 8));
            when others => data_out <= (others => '0');
        end case;
    end process;

    theta1_out <= t1_latch;
    theta2_out <= t2_latch;
    done       <= done_lat;
    error_out  <= err_lat;

end rtl;
