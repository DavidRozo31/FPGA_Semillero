LIBRARY ieee;
USE ieee.std_logic_1164.all;

ENTITY done_latch_reg IS
    PORT (
        clk        : IN  STD_LOGIC;
        rst        : IN  STD_LOGIC;
        start      : IN  STD_LOGIC;
        done_ik    : IN  STD_LOGIC;
        done_latch : OUT STD_LOGIC
    );
END done_latch_reg;

ARCHITECTURE rtl OF done_latch_reg IS
    SIGNAL done_prev : STD_LOGIC := '0';
    SIGNAL latch_reg : STD_LOGIC := '0';
BEGIN

    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                latch_reg <= '0';
                done_prev <= '0';
            else
                done_prev <= done_ik;
                if start = '1' then
                    latch_reg <= '0';
                elsif done_ik = '1' and done_prev = '0' then
                    latch_reg <= '1';
                end if;
            end if;
        end if;
    end process;

    done_latch <= latch_reg;

END rtl;