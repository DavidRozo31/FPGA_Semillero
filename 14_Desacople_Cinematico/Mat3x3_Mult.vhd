library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity Mat3x3_Mult is
    Port (
        clk, rst, start : in  STD_LOGIC;
        a11,a12,a13,a21,a22,a23,a31,a32,a33 : in  STD_LOGIC_VECTOR(15 downto 0);
        b11,b12,b13,b21,b22,b23,b31,b32,b33 : in  STD_LOGIC_VECTOR(15 downto 0);
        done : out STD_LOGIC;
        c11,c12,c13,c21,c22,c23,c31,c32,c33 : out STD_LOGIC_VECTOR(15 downto 0)
    );
end Mat3x3_Mult;

architecture struct of Mat3x3_Mult is

    component fp_multiplier
        Port(clk,rst,valid_i: in STD_LOGIC; a_in,b_in: in STD_LOGIC_VECTOR(15 downto 0);
             valid_o: out STD_LOGIC; p_out: out STD_LOGIC_VECTOR(15 downto 0));
    end component;
    component fp_adder
        Port(clk,rst,op,valid_i: in STD_LOGIC; a_in,b_in: in STD_LOGIC_VECTOR(15 downto 0);
             valid_o: out STD_LOGIC; p_out: out STD_LOGIC_VECTOR(15 downto 0));
    end component;
    component pulse_join2
        Port(clk,rst,pulse_a,pulse_b: in STD_LOGIC; joined: out STD_LOGIC);
    end component;

    type mat9 is array (0 to 2, 0 to 2) of STD_LOGIC_VECTOR(15 downto 0);
    type mat9_sl is array (0 to 2, 0 to 2) of STD_LOGIC;

    signal A, B, C : mat9;
    signal elem_done : mat9_sl;
    signal ZERO_OP : STD_LOGIC := '0'; -- suma

begin
    -- Empaquetar los puertos planos en matrices internas
    A(0,0)<=a11; A(0,1)<=a12; A(0,2)<=a13;
    A(1,0)<=a21; A(1,1)<=a22; A(1,2)<=a23;
    A(2,0)<=a31; A(2,1)<=a32; A(2,2)<=a33;

    B(0,0)<=b11; B(0,1)<=b12; B(0,2)<=b13;
    B(1,0)<=b21; B(1,1)<=b22; B(1,2)<=b23;
    B(2,0)<=b31; B(2,1)<=b32; B(2,2)<=b33;

    c11<=C(0,0); c12<=C(0,1); c13<=C(0,2);
    c21<=C(1,0); c22<=C(1,1); c23<=C(1,2);
    c31<=C(2,0); c32<=C(2,1); c33<=C(2,2);

    ROWS: for I in 0 to 2 generate
        COLS: for J in 0 to 2 generate
            signal p1, p2, p3, s1 : STD_LOGIC_VECTOR(15 downto 0);
            signal d_p1, d_p2, d_p3, d_s1, d_add2 : STD_LOGIC;
        begin
            -- Cij = Ai0*B0j + Ai1*B1j + Ai2*B2j
            MULT1: fp_multiplier port map(clk,rst,start,A(I,0),B(0,J),d_p1,p1);
            MULT2: fp_multiplier port map(clk,rst,start,A(I,1),B(1,J),d_p2,p2);
            MULT3: fp_multiplier port map(clk,rst,start,A(I,2),B(2,J),d_p3,p3);

            ADD1: fp_adder port map(clk,rst,'0',d_p1,p1,p2,d_s1,s1);

            JOIN_S1P3: pulse_join2 port map(clk,rst,d_s1,d_p3,d_add2);
            ADD2: fp_adder port map(clk,rst,'0',d_add2,s1,p3,elem_done(I,J),C(I,J));
        end generate COLS;
    end generate ROWS;

    done <= elem_done(0,0) and elem_done(0,1) and elem_done(0,2) and
            elem_done(1,0) and elem_done(1,1) and elem_done(1,2) and
            elem_done(2,0) and elem_done(2,1) and elem_done(2,2);

end struct;