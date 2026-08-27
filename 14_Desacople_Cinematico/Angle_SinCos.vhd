library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity Angle_SinCos is
    Port (
        clk, rst, start : in  STD_LOGIC;
        theta : in  STD_LOGIC_VECTOR(15 downto 0);
        done  : out STD_LOGIC;
        cos_v, sin_v, neg_sin_v, neg_cos_v : out STD_LOGIC_VECTOR(15 downto 0)
    );
end Angle_SinCos;

architecture struct of Angle_SinCos is
    component cordic_sincos_16
        Port(clk,rst,start: in STD_LOGIC; angle_in: in signed(15 downto 0);
             sin_out, cos_out: out signed(15 downto 0); done: out STD_LOGIC);
    end component;
    component fp_adder
        Port(clk,rst,op,valid_i: in STD_LOGIC; a_in,b_in: in STD_LOGIC_VECTOR(15 downto 0);
             valid_o: out STD_LOGIC; p_out: out STD_LOGIC_VECTOR(15 downto 0));
    end component;

    signal sin_s, cos_s : signed(15 downto 0);
    signal sin_slv, cos_slv : STD_LOGIC_VECTOR(15 downto 0);
    signal d_sc : STD_LOGIC;
    signal d_negsin, d_negcos : STD_LOGIC;
	 signal zero : STD_LOGIC_VECTOR(15 downto 0);
begin
    zero <= x"0000";
    SC: cordic_sincos_16 port map(clk, rst, start, signed(theta), sin_s, cos_s, d_sc);

    sin_slv <= std_logic_vector(sin_s);
    cos_slv <= std_logic_vector(cos_s);
    sin_v   <= sin_slv;
    cos_v   <= cos_slv;

    NEG_SIN: fp_adder port map(clk,rst,'1',d_sc,zero,sin_slv,d_negsin,neg_sin_v); -- 0-sin
    NEG_COS: fp_adder port map(clk,rst,'1',d_sc,zero,cos_slv,d_negcos,neg_cos_v); -- 0-cos

    done <= d_negsin and d_negcos; -- misma profundidad (cordic + 1 resta) en ambos, AND directo seguro
end struct;