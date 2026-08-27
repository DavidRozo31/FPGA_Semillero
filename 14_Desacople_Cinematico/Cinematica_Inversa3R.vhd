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
-- CREATED		"Fri Aug 07 15:06:39 2026"

LIBRARY ieee;
USE ieee.std_logic_1164.all; 

LIBRARY work;

ENTITY Cinematica_Inversa3R IS 
	PORT
	(
		clk :  IN  STD_LOGIC;
		reset :  IN  STD_LOGIC;
		Start :  IN  STD_LOGIC;
		Px_d :  IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
		Py_d :  IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
		Pz_d :  IN  STD_LOGIC_VECTOR(15 DOWNTO 0);
		Ik3R_Ready :  OUT  STD_LOGIC;
		tetha_one :  OUT  STD_LOGIC_VECTOR(15 DOWNTO 0);
		tetha_three :  OUT  STD_LOGIC_VECTOR(15 DOWNTO 0);
		tetha_two :  OUT  STD_LOGIC_VECTOR(15 DOWNTO 0)
	);
END Cinematica_Inversa3R;

ARCHITECTURE bdf_type OF Cinematica_Inversa3R IS 

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

COMPONENT pulse_join2
	PORT(clk : IN STD_LOGIC;
		 rst : IN STD_LOGIC;
		 pulse_a : IN STD_LOGIC;
		 pulse_b : IN STD_LOGIC;
		 joined : OUT STD_LOGIC
	);
END COMPONENT;

SIGNAL	SYNTHESIZED_WIRE_84 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_85 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_86 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_87 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_6 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_7 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_8 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_9 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_10 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_11 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_12 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_13 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_14 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_15 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_88 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_18 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_19 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_20 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_21 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_22 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_23 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_24 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_25 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_26 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_89 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_90 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_30 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_31 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_32 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_33 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_34 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_35 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_91 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_92 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_40 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_41 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_42 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_43 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_44 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_51 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_52 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_53 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_54 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_55 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_56 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_57 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_58 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_59 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_60 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_61 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_62 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_63 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_64 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_65 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_66 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_67 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_68 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	SYNTHESIZED_WIRE_71 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_72 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_73 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_74 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_75 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_76 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_77 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_78 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_79 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_80 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_81 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_82 :  STD_LOGIC;
SIGNAL	SYNTHESIZED_WIRE_83 :  STD_LOGIC;
SIGNAL	L1 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	L2 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	L3_plus_L4 :  STD_LOGIC_VECTOR(15 DOWNTO 0);
SIGNAL	One :  STD_LOGIC_VECTOR(15 DOWNTO 0);

BEGIN 
SYNTHESIZED_WIRE_6 <= '0';
SYNTHESIZED_WIRE_10 <= '1';
SYNTHESIZED_WIRE_11 <= '1';
SYNTHESIZED_WIRE_18 <= '1';
SYNTHESIZED_WIRE_30 <= '1';
SYNTHESIZED_WIRE_40 <= '0';
SYNTHESIZED_WIRE_51 <= '0';
SYNTHESIZED_WIRE_59 <= '1';
SYNTHESIZED_WIRE_75 <= '0';
L1         <= x"01EC";  -- L1     = 0.06m = 492
L2         <= x"036D";  -- L2     = 0.107m = 877
L3_plus_L4 <= x"047B";  -- L3+L4  = 0.14m = 1147
One        <= x"2000";  -- One    = 1.0   = 8192



b2v_inst : cordic_atan2
PORT MAP(clk => clk,
		 rst => reset,
		 start => Start,
		 x_in => Px_d,
		 y_in => Py_d,
		 angle => tetha_one);


b2v_inst1 : fp_multiplier
PORT MAP(clk => clk,
		 rst => reset,
		 valid_i => Start,
		 a_in => Px_d,
		 b_in => Px_d,
		 valid_o => SYNTHESIZED_WIRE_78,
		 p_out => SYNTHESIZED_WIRE_42);


b2v_inst10 : fp_multiplier
PORT MAP(clk => clk,
		 rst => reset,
		 valid_i => SYNTHESIZED_WIRE_84,
		 a_in => SYNTHESIZED_WIRE_85,
		 b_in => SYNTHESIZED_WIRE_85,
		 valid_o => SYNTHESIZED_WIRE_76,
		 p_out => SYNTHESIZED_WIRE_9);


b2v_inst11 : fp_multiplier
PORT MAP(clk => clk,
		 rst => reset,
		 valid_i => SYNTHESIZED_WIRE_86,
		 a_in => SYNTHESIZED_WIRE_87,
		 b_in => SYNTHESIZED_WIRE_87,
		 valid_o => SYNTHESIZED_WIRE_77,
		 p_out => SYNTHESIZED_WIRE_8);


b2v_inst12 : fp_adder
PORT MAP(clk => clk,
		 rst => reset,
		 op => SYNTHESIZED_WIRE_6,
		 valid_i => SYNTHESIZED_WIRE_7,
		 a_in => SYNTHESIZED_WIRE_8,
		 b_in => SYNTHESIZED_WIRE_9,
		 valid_o => SYNTHESIZED_WIRE_67,
		 p_out => SYNTHESIZED_WIRE_68);


b2v_inst13 : fp_adder
PORT MAP(clk => clk,
		 rst => reset,
		 op => SYNTHESIZED_WIRE_10,
		 valid_i => Start,
		 a_in => Pz_d,
		 b_in => L1,
		 valid_o => SYNTHESIZED_WIRE_84,
		 p_out => SYNTHESIZED_WIRE_85);


b2v_inst14 : fp_multiplier
PORT MAP(clk => clk,
		 rst => reset,
		 valid_i => Start,
		 a_in => L3_plus_L4,
		 b_in => L3_plus_L4,
		 valid_o => SYNTHESIZED_WIRE_73,
		 p_out => SYNTHESIZED_WIRE_14);


b2v_inst15 : fp_adder
PORT MAP(clk => clk,
		 rst => reset,
		 op => SYNTHESIZED_WIRE_11,
		 valid_i => SYNTHESIZED_WIRE_12,
		 a_in => SYNTHESIZED_WIRE_13,
		 b_in => SYNTHESIZED_WIRE_14,
		 valid_o => SYNTHESIZED_WIRE_72,
		 p_out => SYNTHESIZED_WIRE_20);


b2v_inst16 : fp_multiplier
PORT MAP(clk => clk,
		 rst => reset,
		 valid_i => SYNTHESIZED_WIRE_15,
		 a_in => SYNTHESIZED_WIRE_88,
		 b_in => SYNTHESIZED_WIRE_88,
		 valid_o => SYNTHESIZED_WIRE_74,
		 p_out => SYNTHESIZED_WIRE_13);



b2v_inst18 : fp_multiplier
PORT MAP(clk => clk,
		 rst => reset,
		 valid_i => Start,
		 a_in => L2,
		 b_in => L2,
		 valid_o => SYNTHESIZED_WIRE_71,
		 p_out => SYNTHESIZED_WIRE_21);


b2v_inst19 : fp_adder
PORT MAP(clk => clk,
		 rst => reset,
		 op => SYNTHESIZED_WIRE_18,
		 valid_i => SYNTHESIZED_WIRE_19,
		 a_in => SYNTHESIZED_WIRE_20,
		 b_in => SYNTHESIZED_WIRE_21,
		 valid_o => SYNTHESIZED_WIRE_83,
		 p_out => SYNTHESIZED_WIRE_26);


b2v_inst2 : fp_multiplier
PORT MAP(clk => clk,
		 rst => reset,
		 valid_i => Start,
		 a_in => Py_d,
		 b_in => Py_d,
		 valid_o => SYNTHESIZED_WIRE_79,
		 p_out => SYNTHESIZED_WIRE_43);




b2v_inst22 : fp_multiplier
PORT MAP(clk => clk,
		 rst => reset,
		 valid_i => SYNTHESIZED_WIRE_22,
		 a_in => SYNTHESIZED_WIRE_23,
		 b_in => L3_plus_L4,
		 valid_o => SYNTHESIZED_WIRE_82,
		 p_out => SYNTHESIZED_WIRE_25);


b2v_inst23 : fp_divider
PORT MAP(clk => clk,
		 rst => reset,
		 start => SYNTHESIZED_WIRE_24,
		 den_in => SYNTHESIZED_WIRE_25,
		 num_in => SYNTHESIZED_WIRE_26,
		 done => SYNTHESIZED_WIRE_89,
		 quot_out => SYNTHESIZED_WIRE_90);


b2v_inst24 : fp_multiplier
PORT MAP(clk => clk,
		 rst => reset,
		 valid_i => SYNTHESIZED_WIRE_89,
		 a_in => SYNTHESIZED_WIRE_90,
		 b_in => SYNTHESIZED_WIRE_90,
		 valid_o => SYNTHESIZED_WIRE_31,
		 p_out => SYNTHESIZED_WIRE_32);


b2v_inst25 : fp_adder
PORT MAP(clk => clk,
		 rst => reset,
		 op => SYNTHESIZED_WIRE_30,
		 valid_i => SYNTHESIZED_WIRE_31,
		 a_in => One,
		 b_in => SYNTHESIZED_WIRE_32,
		 valid_o => SYNTHESIZED_WIRE_33,
		 p_out => SYNTHESIZED_WIRE_34);



b2v_inst27 : sqrt_q13
PORT MAP(clk => clk,
		 rst => reset,
		 start => SYNTHESIZED_WIRE_33,
		 x_in => SYNTHESIZED_WIRE_34,
		 done => SYNTHESIZED_WIRE_92,
		 y_out => SYNTHESIZED_WIRE_91);


b2v_inst28 : cordic_atan2
PORT MAP(clk => clk,
		 rst => reset,
		 start => SYNTHESIZED_WIRE_35,
		 x_in => SYNTHESIZED_WIRE_90,
		 y_in => SYNTHESIZED_WIRE_91,
		 done => SYNTHESIZED_WIRE_81,
		 angle => tetha_three);


b2v_inst29 : pulse_join2
PORT MAP(clk => clk,
		 rst => reset,
		 pulse_a => SYNTHESIZED_WIRE_92,
		 pulse_b => SYNTHESIZED_WIRE_89,
		 joined => SYNTHESIZED_WIRE_35);


b2v_inst3 : fp_adder
PORT MAP(clk => clk,
		 rst => reset,
		 op => SYNTHESIZED_WIRE_40,
		 valid_i => SYNTHESIZED_WIRE_41,
		 a_in => SYNTHESIZED_WIRE_42,
		 b_in => SYNTHESIZED_WIRE_43,
		 valid_o => SYNTHESIZED_WIRE_65,
		 p_out => SYNTHESIZED_WIRE_66);


b2v_inst30 : cordic_atan2
PORT MAP(clk => clk,
		 rst => reset,
		 start => SYNTHESIZED_WIRE_44,
		 x_in => SYNTHESIZED_WIRE_87,
		 y_in => SYNTHESIZED_WIRE_85,
		 done => SYNTHESIZED_WIRE_63,
		 angle => SYNTHESIZED_WIRE_61);


b2v_inst31 : fp_multiplier
PORT MAP(clk => clk,
		 rst => reset,
		 valid_i => SYNTHESIZED_WIRE_92,
		 a_in => L3_plus_L4,
		 b_in => SYNTHESIZED_WIRE_91,
		 valid_o => SYNTHESIZED_WIRE_57,
		 p_out => SYNTHESIZED_WIRE_56);


b2v_inst32 : fp_multiplier
PORT MAP(clk => clk,
		 rst => reset,
		 valid_i => SYNTHESIZED_WIRE_89,
		 a_in => L3_plus_L4,
		 b_in => SYNTHESIZED_WIRE_90,
		 valid_o => SYNTHESIZED_WIRE_52,
		 p_out => SYNTHESIZED_WIRE_53);


b2v_inst33 : fp_adder
PORT MAP(clk => clk,
		 rst => reset,
		 op => SYNTHESIZED_WIRE_51,
		 valid_i => SYNTHESIZED_WIRE_52,
		 a_in => SYNTHESIZED_WIRE_53,
		 b_in => L2,
		 valid_o => SYNTHESIZED_WIRE_58,
		 p_out => SYNTHESIZED_WIRE_55);



b2v_inst35 : cordic_atan2
PORT MAP(clk => clk,
		 rst => reset,
		 start => SYNTHESIZED_WIRE_54,
		 x_in => SYNTHESIZED_WIRE_55,
		 y_in => SYNTHESIZED_WIRE_56,
		 done => SYNTHESIZED_WIRE_64,
		 angle => SYNTHESIZED_WIRE_62);


b2v_inst36 : pulse_join2
PORT MAP(clk => clk,
		 rst => reset,
		 pulse_a => SYNTHESIZED_WIRE_57,
		 pulse_b => SYNTHESIZED_WIRE_58,
		 joined => SYNTHESIZED_WIRE_54);


b2v_inst37 : fp_adder
PORT MAP(clk => clk,
		 rst => reset,
		 op => SYNTHESIZED_WIRE_59,
		 valid_i => SYNTHESIZED_WIRE_60,
		 a_in => SYNTHESIZED_WIRE_61,
		 b_in => SYNTHESIZED_WIRE_62,
		 valid_o => SYNTHESIZED_WIRE_80,
		 p_out => tetha_two);



b2v_inst39 : pulse_join2
PORT MAP(clk => clk,
		 rst => reset,
		 pulse_a => SYNTHESIZED_WIRE_63,
		 pulse_b => SYNTHESIZED_WIRE_64,
		 joined => SYNTHESIZED_WIRE_60);


b2v_inst4 : sqrt_q13
PORT MAP(clk => clk,
		 rst => reset,
		 start => SYNTHESIZED_WIRE_65,
		 x_in => SYNTHESIZED_WIRE_66,
		 done => SYNTHESIZED_WIRE_86,
		 y_out => SYNTHESIZED_WIRE_87);



b2v_inst41 : sqrt_q13
PORT MAP(clk => clk,
		 rst => reset,
		 start => SYNTHESIZED_WIRE_67,
		 x_in => SYNTHESIZED_WIRE_68,
		 done => SYNTHESIZED_WIRE_15,
		 y_out => SYNTHESIZED_WIRE_88);


b2v_inst43 : pulse_join2
PORT MAP(clk => clk,
		 rst => reset,
		 pulse_a => SYNTHESIZED_WIRE_86,
		 pulse_b => SYNTHESIZED_WIRE_84,
		 joined => SYNTHESIZED_WIRE_44);


b2v_inst44 : pulse_join2
PORT MAP(clk => clk,
		 rst => reset,
		 pulse_a => SYNTHESIZED_WIRE_71,
		 pulse_b => SYNTHESIZED_WIRE_72,
		 joined => SYNTHESIZED_WIRE_19);


b2v_inst45 : pulse_join2
PORT MAP(clk => clk,
		 rst => reset,
		 pulse_a => SYNTHESIZED_WIRE_73,
		 pulse_b => SYNTHESIZED_WIRE_74,
		 joined => SYNTHESIZED_WIRE_12);


b2v_inst46 : fp_adder
PORT MAP(clk => clk,
		 rst => reset,
		 op => SYNTHESIZED_WIRE_75,
		 valid_i => Start,
		 a_in => L2,
		 b_in => L2,
		 valid_o => SYNTHESIZED_WIRE_22,
		 p_out => SYNTHESIZED_WIRE_23);


b2v_inst47 : pulse_join2
PORT MAP(clk => clk,
		 rst => reset,
		 pulse_a => SYNTHESIZED_WIRE_76,
		 pulse_b => SYNTHESIZED_WIRE_77,
		 joined => SYNTHESIZED_WIRE_7);


b2v_inst48 : pulse_join2
PORT MAP(clk => clk,
		 rst => reset,
		 pulse_a => SYNTHESIZED_WIRE_78,
		 pulse_b => SYNTHESIZED_WIRE_79,
		 joined => SYNTHESIZED_WIRE_41);


b2v_inst49 : pulse_join2
PORT MAP(clk => clk,
		 rst => reset,
		 pulse_a => SYNTHESIZED_WIRE_80,
		 pulse_b => SYNTHESIZED_WIRE_81,
		 joined => Ik3R_Ready);




b2v_inst9 : pulse_join2
PORT MAP(clk => clk,
		 rst => reset,
		 pulse_a => SYNTHESIZED_WIRE_82,
		 pulse_b => SYNTHESIZED_WIRE_83,
		 joined => SYNTHESIZED_WIRE_24);


END bdf_type;