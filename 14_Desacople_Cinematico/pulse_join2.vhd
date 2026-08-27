library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity pulse_join2 is
    Port (
        clk     : in  std_logic;
        rst     : in  std_logic;
        pulse_a : in  std_logic;
        pulse_b : in  std_logic;
        joined  : out std_logic
    );
end pulse_join2;

architecture rtl of pulse_join2 is
    type state_t is (S_WAIT_BOTH, S_GOT_A, S_GOT_B);
    signal state   : state_t := S_WAIT_BOTH;
    signal out_reg : std_logic := '0';
begin
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                state   <= S_WAIT_BOTH;
                out_reg <= '0';
            else
                out_reg <= '0';

                case state is
                    when S_WAIT_BOTH =>
                        if pulse_a = '1' and pulse_b = '1' then
                            out_reg <= '1';
                        elsif pulse_a = '1' then
                            state <= S_GOT_A;
                        elsif pulse_b = '1' then
                            state <= S_GOT_B;
                        end if;

                    when S_GOT_A =>
                        if pulse_b = '1' then
                            out_reg <= '1';
                            state   <= S_WAIT_BOTH;
                        end if;

                    when S_GOT_B =>
                        if pulse_a = '1' then
                            out_reg <= '1';
                            state   <= S_WAIT_BOTH;
                        end if;
                end case;
            end if;
        end if;
    end process;

    joined <= out_reg;
end rtl;