-- Copyright (C) 2018  Intel Corporation. All rights reserved.
-- Your use of Intel Corporation's design tools, logic functions 
-- and other software and tools, and its AMPP partner logic 
-- functions, and any output files from any of the foregoing 
-- (including device programming or simulation files), and any 
-- associated documentation or information are expressly subject 
-- to the terms and conditions of the Intel Program License 
-- Subscription Agreement, the Intel Quartus Prime License Agreement,
-- the Intel FPGA IP License Agreement, or other applicable license
-- agreement, including, without limitation, that your use is for
-- the sole purpose of programming logic devices manufactured by
-- Intel and sold by Intel or its authorized distributors.  Please
-- refer to the applicable agreement for further details.

-- PROGRAM		"Quartus Prime"
-- VERSION		"Version 18.1.0 Build 625 09/12/2018 SJ Lite Edition"
-- CREATED		"Wed May 06 13:28:48 2026"

LIBRARY ieee;
USE ieee.std_logic_1164.all; 

LIBRARY work;

ENTITY Block2R IS 
	PORT
	(
		clk :  IN  STD_LOGIC;
		Reset :  IN  STD_LOGIC;
		StartALL :  IN  STD_LOGIC;
		L1 :  IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
		L2 :  IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
		One :  IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
		Px :  IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
		Py :  IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
		Two :  IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
		Tetha1Ready :  OUT  STD_LOGIC;
		Theta2Ready :  OUT  STD_LOGIC;
		Tetha1 :  OUT  STD_LOGIC_VECTOR(15 DOWNTO 0);
		Tetha2 :  OUT  STD_LOGIC_VECTOR(15 DOWNTO 0)
	);
END Block2R;

ARCHITECTURE bdf_type OF Block2R IS 

COMPONENT fp_adder
	PORT(clk : IN STD_LOGIC;
		 rst : IN STD_LOGIC;
		 op : IN STD_LOGIC;
		 valid_i : IN STD_LOGIC;
		 a_in : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 b_in : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 valid_o : OUT STD_LOGIC;
		 p_out : OUT STD_LOGIC_VECTOR(15 DOWNTO 0)
	);
END COMPONENT;

COMPONENT fp_multiplier
	PORT(clk : IN STD_LOGIC;
		 rst : IN STD_LOGIC;
		 valid_i : IN STD_LOGIC;
		 a_in : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 b_in : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 valid_o : OUT STD_LOGIC;
		 p_out : OUT STD_LOGIC_VECTOR(15 DOWNTO 0)
	);
END COMPONENT;

COMPONENT fp_divider
	PORT(clk : IN STD_LOGIC;
		 rst : IN STD_LOGIC;
		 start : IN STD_LOGIC;
		 den_in : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 num_in : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 done : OUT STD_LOGIC;
		 quot_out : OUT STD_LOGIC_VECTOR(15 DOWNTO 0)
	);
END COMPONENT;

COMPONENT sqrt_q13
	PORT(clk : IN STD_LOGIC;
		 rst : IN STD_LOGIC;
		 start : IN STD_LOGIC;
		 x_in : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 done : OUT STD_LOGIC;
		 y_out : OUT STD_LOGIC_VECTOR(15 DOWNTO 0)
	);
END COMPONENT;

COMPONENT cordic_atan2
	PORT(clk : IN STD_LOGIC;
		 rst : IN STD_LOGIC;
		 start : IN STD_LOGIC;
		 x_in : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 y_in : IN STD_LOGIC_VECTOR(15 DOWNTO 0);
		 done : OUT STD_LOGIC;
		 angle : OUT STD_LOGIC_VECTOR(15 DOWNTO 0)
	);
END COMPONENT;

SIGNAL	SYNTHESIZED_WIRE_0 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_1 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_2 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_3 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_4 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_5 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_6 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_7 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_8 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_54 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_55 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_12 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_13 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_14 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_15 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_16 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_56 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_57 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_22 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_23 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_24 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_27 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_28 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_29 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_30 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_31 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_32 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_33 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_34 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_35 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_36 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_37 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_38 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_39 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_40 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_41 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_42 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_43 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_44 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_45 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_46 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_47 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_48 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_49 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_50 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_51 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_52 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_53 :  STD_LOGIC_VECTOR(15 DOWNTO 0);


BEGIN 
SYNTHESIZED_WIRE_0 <= '0';
SYNTHESIZED_WIRE_12 <= '1';
SYNTHESIZED_WIRE_22 <= '0';
SYNTHESIZED_WIRE_30 <= '1';
SYNTHESIZED_WIRE_46 <= '0';
SYNTHESIZED_WIRE_50 <= '1';



b2v_inst : fp_adder
PORT MAP(clk => clk,
		 rst => Reset,
		 op => SYNTHESIZED_WIRE_0,
		 valid_i => SYNTHESIZED_WIRE_1,
		 a_in => SYNTHESIZED_WIRE_2,
		 b_in => SYNTHESIZED_WIRE_3,
		 valid_o => SYNTHESIZED_WIRE_36,
		 p_out => SYNTHESIZED_WIRE_52);



b2v_inst10 : fp_multiplier
PORT MAP(clk => clk,
		 rst => Reset,
		 valid_i => SYNTHESIZED_WIRE_4,
		 a_in => SYNTHESIZED_WIRE_5,
		 b_in => L2,
		 valid_o => SYNTHESIZED_WIRE_39,
		 p_out => SYNTHESIZED_WIRE_7);


b2v_inst11 : fp_divider
PORT MAP(clk => clk,
		 rst => Reset,
		 start => SYNTHESIZED_WIRE_6,
		 den_in => SYNTHESIZED_WIRE_7,
		 num_in => SYNTHESIZED_WIRE_8,
		 done => SYNTHESIZED_WIRE_54,
		 quot_out => SYNTHESIZED_WIRE_55);


b2v_inst12 : fp_multiplier
PORT MAP(clk => clk,
		 rst => Reset,
		 valid_i => SYNTHESIZED_WIRE_54,
		 a_in => SYNTHESIZED_WIRE_55,
		 b_in => SYNTHESIZED_WIRE_55,
		 valid_o => SYNTHESIZED_WIRE_13,
		 p_out => SYNTHESIZED_WIRE_14);


b2v_inst13 : fp_adder
PORT MAP(clk => clk,
		 rst => Reset,
		 op => SYNTHESIZED_WIRE_12,
		 valid_i => SYNTHESIZED_WIRE_13,
		 a_in => One,
		 b_in => SYNTHESIZED_WIRE_14,
		 valid_o => SYNTHESIZED_WIRE_15,
		 p_out => SYNTHESIZED_WIRE_16);


b2v_inst14 : sqrt_q13
PORT MAP(clk => clk,
		 rst => Reset,
		 start => SYNTHESIZED_WIRE_15,
		 x_in => SYNTHESIZED_WIRE_16,
		 done => SYNTHESIZED_WIRE_56,
		 y_out => SYNTHESIZED_WIRE_57);


b2v_inst15 : cordic_atan2
PORT MAP(clk => clk,
		 rst => Reset,
		 start => SYNTHESIZED_WIRE_56,
		 x_in => SYNTHESIZED_WIRE_57,
		 y_in => SYNTHESIZED_WIRE_55,
		 done => Theta2Ready,
		 angle => Tetha2);


b2v_inst16 : cordic_atan2
PORT MAP(clk => clk,
		 rst => Reset,
		 start => StartALL,
		 x_in => Px,
		 y_in => Py,
		 done => SYNTHESIZED_WIRE_40,
		 angle => SYNTHESIZED_WIRE_32);


b2v_inst17 : fp_multiplier
PORT MAP(clk => clk,
		 rst => Reset,
		 valid_i => SYNTHESIZED_WIRE_54,
		 a_in => L2,
		 b_in => SYNTHESIZED_WIRE_55,
		 valid_o => SYNTHESIZED_WIRE_23,
		 p_out => SYNTHESIZED_WIRE_24);


b2v_inst18 : fp_adder
PORT MAP(clk => clk,
		 rst => Reset,
		 op => SYNTHESIZED_WIRE_22,
		 valid_i => SYNTHESIZED_WIRE_23,
		 a_in => L1,
		 b_in => SYNTHESIZED_WIRE_24,
		 valid_o => SYNTHESIZED_WIRE_34,
		 p_out => SYNTHESIZED_WIRE_28);


b2v_inst19 : fp_multiplier
PORT MAP(clk => clk,
		 rst => Reset,
		 valid_i => SYNTHESIZED_WIRE_56,
		 a_in => SYNTHESIZED_WIRE_57,
		 b_in => L2,
		 valid_o => SYNTHESIZED_WIRE_35,
		 p_out => SYNTHESIZED_WIRE_29);



b2v_inst20 : cordic_atan2
PORT MAP(clk => clk,
		 rst => Reset,
		 start => SYNTHESIZED_WIRE_27,
		 x_in => SYNTHESIZED_WIRE_28,
		 y_in => SYNTHESIZED_WIRE_29,
		 done => SYNTHESIZED_WIRE_41,
		 angle => SYNTHESIZED_WIRE_33);


b2v_inst21 : fp_adder
PORT MAP(clk => clk,
		 rst => Reset,
		 op => SYNTHESIZED_WIRE_30,
		 valid_i => SYNTHESIZED_WIRE_31,
		 a_in => SYNTHESIZED_WIRE_32,
		 b_in => SYNTHESIZED_WIRE_33,
		 valid_o => Tetha1Ready,
		 p_out => Tetha1);





SYNTHESIZED_WIRE_27 <= SYNTHESIZED_WIRE_34 AND SYNTHESIZED_WIRE_35;


SYNTHESIZED_WIRE_51 <= SYNTHESIZED_WIRE_36 AND SYNTHESIZED_WIRE_37;


SYNTHESIZED_WIRE_6 <= SYNTHESIZED_WIRE_38 AND SYNTHESIZED_WIRE_39;


SYNTHESIZED_WIRE_31 <= SYNTHESIZED_WIRE_40 AND SYNTHESIZED_WIRE_41;


b2v_inst3 : fp_multiplier
PORT MAP(clk => clk,
		 rst => Reset,
		 valid_i => StartALL,
		 a_in => Px,
		 b_in => Px,
		 valid_o => SYNTHESIZED_WIRE_42,
		 p_out => SYNTHESIZED_WIRE_2);



SYNTHESIZED_WIRE_1 <= SYNTHESIZED_WIRE_42 AND SYNTHESIZED_WIRE_43;


SYNTHESIZED_WIRE_47 <= SYNTHESIZED_WIRE_44 AND SYNTHESIZED_WIRE_45;


b2v_inst4 : fp_multiplier
PORT MAP(clk => clk,
		 rst => Reset,
		 valid_i => StartALL,
		 a_in => L1,
		 b_in => L1,
		 valid_o => SYNTHESIZED_WIRE_44,
		 p_out => SYNTHESIZED_WIRE_48);


b2v_inst5 : fp_multiplier
PORT MAP(clk => clk,
		 rst => Reset,
		 valid_i => StartALL,
		 a_in => Py,
		 b_in => Py,
		 valid_o => SYNTHESIZED_WIRE_43,
		 p_out => SYNTHESIZED_WIRE_3);


b2v_inst6 : fp_multiplier
PORT MAP(clk => clk,
		 rst => Reset,
		 valid_i => StartALL,
		 a_in => L2,
		 b_in => L2,
		 valid_o => SYNTHESIZED_WIRE_45,
		 p_out => SYNTHESIZED_WIRE_49);


b2v_inst7 : fp_adder
PORT MAP(clk => clk,
		 rst => Reset,
		 op => SYNTHESIZED_WIRE_46,
		 valid_i => SYNTHESIZED_WIRE_47,
		 a_in => SYNTHESIZED_WIRE_48,
		 b_in => SYNTHESIZED_WIRE_49,
		 valid_o => SYNTHESIZED_WIRE_37,
		 p_out => SYNTHESIZED_WIRE_53);


b2v_inst8 : fp_adder
PORT MAP(clk => clk,
		 rst => Reset,
		 op => SYNTHESIZED_WIRE_50,
		 valid_i => SYNTHESIZED_WIRE_51,
		 a_in => SYNTHESIZED_WIRE_52,
		 b_in => SYNTHESIZED_WIRE_53,
		 valid_o => SYNTHESIZED_WIRE_38,
		 p_out => SYNTHESIZED_WIRE_8);


b2v_inst9 : fp_multiplier
PORT MAP(clk => clk,
		 rst => Reset,
		 valid_i => StartALL,
		 a_in => Two,
		 b_in => L1,
		 valid_o => SYNTHESIZED_WIRE_4,
		 p_out => SYNTHESIZED_WIRE_5);


END bdf_type;