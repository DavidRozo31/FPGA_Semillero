library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

entity R03_Builder is
    Port (
        clk, rst, start : in  STD_LOGIC;
        theta1, theta2, theta3 : in  STD_LOGIC_VECTOR(15 downto 0);
        done : out STD_LOGIC;
        r03_11,r03_12,r03_13,r03_21,r03_22,r03_23,r03_31,r03_32,r03_33 : out STD_LOGIC_VECTOR(15 downto 0)
    );
end R03_Builder;

architecture struct of R03_Builder is
    component Angle_SinCos
        Port(clk,rst,start: in STD_LOGIC; theta: in STD_LOGIC_VECTOR(15 downto 0);
             done: out STD_LOGIC;
             cos_v,sin_v,neg_sin_v,neg_cos_v: out STD_LOGIC_VECTOR(15 downto 0));
    end component;
    component Mat3x3_Mult
        Port(clk,rst,start: in STD_LOGIC;
             a11,a12,a13,a21,a22,a23,a31,a32,a33: in STD_LOGIC_VECTOR(15 downto 0);
             b11,b12,b13,b21,b22,b23,b31,b32,b33: in STD_LOGIC_VECTOR(15 downto 0);
             done: out STD_LOGIC;
             c11,c12,c13,c21,c22,c23,c31,c32,c33: out STD_LOGIC_VECTOR(15 downto 0));
    end component;

    -- Constantes internas (ya no son puertos)
    signal zero, one : STD_LOGIC_VECTOR(15 downto 0);

    signal c1,s1,ns1,nc1 : STD_LOGIC_VECTOR(15 downto 0);
    signal c2,s2,ns2,nc2 : STD_LOGIC_VECTOR(15 downto 0);
    signal c3,s3,ns3,nc3 : STD_LOGIC_VECTOR(15 downto 0);
    signal d1,d2,d3, join_sc : STD_LOGIC;

    -- R01, R12, R23: pura conexion de cables, sin componentes
    signal r01_11,r01_12,r01_13,r01_21,r01_22,r01_23,r01_31,r01_32,r01_33 : STD_LOGIC_VECTOR(15 downto 0);
    signal r12_11,r12_12,r12_13,r12_21,r12_22,r12_23,r12_31,r12_32,r12_33 : STD_LOGIC_VECTOR(15 downto 0);
    signal r23_11,r23_12,r23_13,r23_21,r23_22,r23_23,r23_31,r23_32,r23_33 : STD_LOGIC_VECTOR(15 downto 0);

    signal r0102_11,r0102_12,r0102_13,r0102_21,r0102_22,r0102_23,r0102_31,r0102_32,r0102_33 : STD_LOGIC_VECTOR(15 downto 0);
    signal d_mm1, d_mm2 : STD_LOGIC;
begin
    zero <= x"0000";
    one  <= x"2000";

    SC1: Angle_SinCos port map(clk,rst,start,theta1,d1,c1,s1,ns1,nc1);
    SC2: Angle_SinCos port map(clk,rst,start,theta2,d2,c2,s2,ns2,nc2);
    SC3: Angle_SinCos port map(clk,rst,start,theta3,d3,c3,s3,ns3,nc3);

    -- Las 3 corren en paralelo desde el mismo 'start', misma profundidad -> AND directo seguro
    join_sc <= d1 and d2 and d3;

    -- R01 = [c1,0,s1; s1,0,-c1; 0,1,0]
    r01_11<=c1;    r01_12<=zero;  r01_13<=s1;
    r01_21<=s1;    r01_22<=zero;  r01_23<=nc1;
    r01_31<=zero;  r01_32<=one;   r01_33<=zero;

    -- R12 = [c2,-s2,0; s2,c2,0; 0,0,1]
    r12_11<=c2;    r12_12<=ns2;   r12_13<=zero;
    r12_21<=s2;    r12_22<=c2;    r12_23<=zero;
    r12_31<=zero;  r12_32<=zero;  r12_33<=one;

    -- R23 = [-s3,0,c3; c3,0,s3; 0,1,0]
    r23_11<=ns3;   r23_12<=zero;  r23_13<=c3;
    r23_21<=c3;    r23_22<=zero;  r23_23<=s3;
    r23_31<=zero;  r23_32<=one;   r23_33<=zero;

    -- R03 = (R01 * R12) * R23
    MM1: Mat3x3_Mult port map(clk,rst,join_sc,
        r01_11,r01_12,r01_13,r01_21,r01_22,r01_23,r01_31,r01_32,r01_33,
        r12_11,r12_12,r12_13,r12_21,r12_22,r12_23,r12_31,r12_32,r12_33,
        d_mm1,
        r0102_11,r0102_12,r0102_13,r0102_21,r0102_22,r0102_23,r0102_31,r0102_32,r0102_33);

    MM2: Mat3x3_Mult port map(clk,rst,d_mm1,
        r0102_11,r0102_12,r0102_13,r0102_21,r0102_22,r0102_23,r0102_31,r0102_32,r0102_33,
        r23_11,r23_12,r23_13,r23_21,r23_22,r23_23,r23_31,r23_32,r23_33,
        d_mm2,
        r03_11,r03_12,r03_13,r03_21,r03_22,r03_23,r03_31,r03_32,r03_33);

    done <= d_mm2;
end struct;