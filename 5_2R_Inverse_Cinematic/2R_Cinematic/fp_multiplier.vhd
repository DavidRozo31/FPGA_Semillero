library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity fp_multiplier is
    Port (
        clk     : in  std_logic;
        rst     : in  std_logic;
        valid_i : in  std_logic;
        a_in    : in  std_logic_vector(15 downto 0);
        b_in    : in  std_logic_vector(15 downto 0);
        p_out   : out std_logic_vector(15 downto 0);
        valid_o : out std_logic
    );
end fp_multiplier;

architecture rtl of fp_multiplier is
    signal flag      : std_logic := '0';
    signal prod_reg  : signed(31 downto 0) := (others => '0');
    signal valid_reg : std_logic := '0';
begin
    process(clk)
    begin
        if rising_edge(clk) then
            if rst = '1' then
                flag      <= '0';
                prod_reg  <= (others => '0');
                valid_reg <= '0';
            elsif valid_i = '1' or flag = '1' then
                flag      <= '1';
                prod_reg  <= signed(a_in) * signed(b_in);
                valid_reg <= '1';
            end if;
        end if;
    end process;

    p_out   <= std_logic_vector(prod_reg(28 downto 13));
    valid_o <= valid_reg;
end rtl;