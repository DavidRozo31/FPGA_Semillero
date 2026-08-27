library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity Mat3_Transpose is
    Port (
        clk, rst, start : in  STD_LOGIC;
        a11,a12,a13,a21,a22,a23,a31,a32,a33 : in  STD_LOGIC_VECTOR(15 downto 0);
        done : out STD_LOGIC;
        t11,t12,t13,t21,t22,t23,t31,t32,t33 : out STD_LOGIC_VECTOR(15 downto 0)
    );
end Mat3_Transpose;

architecture rtl of Mat3_Transpose is
begin
    process(clk, rst)
    begin
        if rst = '1' then
            t11<=(others=>'0'); t12<=(others=>'0'); t13<=(others=>'0');
            t21<=(others=>'0'); t22<=(others=>'0'); t23<=(others=>'0');
            t31<=(others=>'0'); t32<=(others=>'0'); t33<=(others=>'0');
            done <= '0';
        elsif rising_edge(clk) then
            if start = '1' then
                t11<=a11; t12<=a21; t13<=a31;
                t21<=a12; t22<=a22; t23<=a32;
                t31<=a13; t32<=a23; t33<=a33;
                done <= '1';
            else
                done <= '0';
            end if;
        end if;
    end process;
end rtl;