library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

package cordic_pkg is
    constant N_BITS : integer := 16;
    constant N_ITER : integer := 12;

    type atan_table_t is array (0 to N_ITER-1) of signed(N_BITS-1 downto 0);

    -- arctan(2^-i) * 2^13  — verificado con: round(atan(2^-i) * 8192)
    -- i=0:  atan(1)      = 0.7854 rad → 0.7854*8192 =  6434
    -- i=1:  atan(0.5)    = 0.4636 rad → 0.4636*8192 =  3799
    -- i=2:  atan(0.25)   = 0.2450 rad → 0.2450*8192 =  2008
    -- i=3:  atan(0.125)  = 0.1244 rad → 0.1244*8192 =  1019
    -- i=4:  atan(0.0625) = 0.0624 rad → 0.0624*8192 =   511
    constant ATAN_TABLE : atan_table_t := (
        to_signed( 6434, 16),  -- arctan(2^0)
        to_signed( 3799, 16),  -- arctan(2^-1)
        to_signed( 2008, 16),  -- arctan(2^-2)
        to_signed( 1019, 16),  -- arctan(2^-3)
        to_signed(  511, 16),  -- arctan(2^-4)
        to_signed(  256, 16),  -- arctan(2^-5)
        to_signed(  128, 16),  -- arctan(2^-6)
        to_signed(   64, 16),  -- arctan(2^-7)
        to_signed(   32, 16),  -- arctan(2^-8)
        to_signed(   16, 16),  -- arctan(2^-9)
        to_signed(    8, 16),  -- arctan(2^-10)
        to_signed(    4, 16)   -- arctan(2^-11)
    );

    -- K = 0.6073 en Q2.13 = round(0.6073 * 8192) = 4978
    constant CORDIC_K : signed(15 downto 0) := to_signed(4978, 16);

end package cordic_pkg;