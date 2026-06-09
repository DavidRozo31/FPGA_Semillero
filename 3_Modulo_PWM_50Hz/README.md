# Módulo PWM_50Hz

En esta carpeta se documentará el diseño e implementación del módulo PWM a 50Hz para FPGA.


```vhdl
LIBRARY IEEE;
USE IEEE.STD_LOGIC_1164.ALL;
USE IEEE.NUMERIC_STD.ALL;

ENTITY PWM_50Hz IS
    Generic(
        N : integer := 20
    );
    Port (
        clk       : in  std_logic;
        start     : in  std_logic;
        dutycycle : in  std_logic_vector(N-1 downto 0);  -- cambiado de unsigned a slv
        pwm_out   : out std_logic
    );
END PWM_50Hz;

ARCHITECTURE PWM_50Hz_arch OF PWM_50Hz IS
    constant PERIOD : unsigned(N-1 downto 0) := to_unsigned(1000000, N);
    signal counter  : unsigned(N-1 downto 0) := (others => '0');

BEGIN
    -- Contador principal
    process(clk)
    begin
        if rising_edge(clk) then
            if start = '0' then
                counter <= (others => '0');
            elsif counter = PERIOD - 1 then
                counter <= (others => '0');
            else
                counter <= counter + 1;
            end if;
        end if;
    end process;

    -- Comparador PWM: conversión slv → unsigned al comparar
    pwm_out <= '1' when (counter < unsigned(dutycycle) and start = '1') else '0';

END PWM_50Hz_arch;
```


