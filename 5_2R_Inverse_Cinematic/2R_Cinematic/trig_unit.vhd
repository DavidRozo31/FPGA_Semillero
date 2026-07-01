library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use work.cordic_pkg.ALL;

entity trig_unit is
    Port (
        clk       : in  std_logic;
        rst       : in  std_logic;
        data_in   : in  std_logic_vector(7 downto 0);
        byte_sel  : in  std_logic_vector(1 downto 0);  -- CAMBIO: 3→2 bits (solo 2 bytes por operando)
        load      : in  std_logic;
        start     : in  std_logic;
        data_out  : out std_logic_vector(7 downto 0);
        out_sel   : in  std_logic_vector(1 downto 0);  -- CAMBIO: 3→2 bits (solo 2 bytes resultado)
        done      : out std_logic
    );
end trig_unit;

architecture rtl of trig_unit is
    signal operand_a_reg  : signed(15 downto 0) := (others => '0');  -- x_in
    signal operand_b_reg  : signed(15 downto 0) := (others => '0');  -- y_in
    signal start_delayed  : std_logic := '0';
    signal operand_a_del  : signed(15 downto 0) := (others => '0');
    signal operand_b_del  : signed(15 downto 0) := (others => '0');
    signal atan_result    : signed(15 downto 0);

    component cordic_atan2 is
        Port (
            clk   : in  std_logic;
            rst   : in  std_logic;
            start : in  std_logic;
            x_in  : in  signed(15 downto 0);
            y_in  : in  signed(15 downto 0);
            angle : out signed(15 downto 0);
            done  : out std_logic
        );
    end component;

begin
    U_ATAN2: cordic_atan2 port map (
        clk   => clk,
        rst   => rst,
        start => start_delayed,
        x_in  => operand_a_del,
        y_in  => operand_b_del,
        angle => atan_result,
        done  => done
    );

    -- Cargar bytes: 00/01 = operand_a (x), 10/11 = operand_b (y)
    process(clk, rst)
    begin
        if rst = '1' then
            operand_a_reg <= (others => '0');
            operand_b_reg <= (others => '0');
        elsif rising_edge(clk) then
            if load = '1' then
                case byte_sel is
                    when "00" => operand_a_reg(7  downto 0)  <= signed(data_in);
                    when "01" => operand_a_reg(15 downto 8)  <= signed(data_in);
                    when "10" => operand_b_reg(7  downto 0)  <= signed(data_in);
                    when "11" => operand_b_reg(15 downto 8)  <= signed(data_in);
                    when others => null;
                end case;
            end if;
        end if;
    end process;

    -- Retrasar start y operandos un ciclo
    process(clk, rst)
    begin
        if rst = '1' then
            start_delayed <= '0';
            operand_a_del <= (others => '0');
            operand_b_del <= (others => '0');
        elsif rising_edge(clk) then
            start_delayed <= start;
            operand_a_del <= operand_a_reg;
            operand_b_del <= operand_b_reg;
        end if;
    end process;

    -- Leer bytes del resultado
    process(out_sel, atan_result)
    begin
        case out_sel is
            when "00"   => data_out <= std_logic_vector(atan_result(7  downto 0));
            when "01"   => data_out <= std_logic_vector(atan_result(15 downto 8));
            when others => data_out <= (others => '0');
        end case;
    end process;

end rtl;