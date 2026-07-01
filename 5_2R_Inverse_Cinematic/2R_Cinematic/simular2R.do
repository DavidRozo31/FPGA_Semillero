quit -sim

cd C:/intelFPGA_lite/18.1/Projects/2R_Cinematic

vdel -all
vlib work
vmap work work

vcom -93 -work work fp_multiplier.vhd
vcom -93 -work work fp_adder.vhd
vcom -93 -work work fp_divider.vhd
vcom -93 -work work sqrt_q13.vhd
vcom -93 -work work cordic_pkg.vhd
vcom -93 -work work cordic_atan2.vhd
vcom -93 -work work Block2RTest.vhd
vcom -93 -work work Block2R_tb.vhd

vsim -t 1ns work.Block2R_tb

# ================================================================
# CONTROL
# ================================================================
add wave -divider "=== CONTROL ==="
add wave -radix hexadecimal -label "clk"   sim:/Block2R_tb/clk_s
add wave -radix hexadecimal -label "rst"   sim:/Block2R_tb/rst_s
add wave -radix hexadecimal -label "Start" sim:/Block2R_tb/start_s

# ================================================================
# ENTRADAS
# ================================================================
add wave -divider "=== ENTRADAS ==="
add wave -radix decimal -label "Px"  sim:/Block2R_tb/Px_s
add wave -radix decimal -label "Py"  sim:/Block2R_tb/Py_s
add wave -radix decimal -label "L1"  sim:/Block2R_tb/L1_s
add wave -radix decimal -label "L2"  sim:/Block2R_tb/L2_s
add wave -radix decimal -label "One" sim:/Block2R_tb/One_s
add wave -radix decimal -label "Two" sim:/Block2R_tb/Two_s

# ================================================================
# PASO 1: Px2, Py2
# inst: b2v_inst (Px*Px), b2v_inst1 (Py*Py)
# ================================================================
add wave -divider "=== PASO 1: Px2 Py2 ==="
add wave -radix decimal -label "Px2"        sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_53
add wave -radix decimal -label "Py2"        sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_54
add wave -radix decimal -label "valid_Px2"  sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_47
add wave -radix decimal -label "valid_Py2"  sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_48
add wave -radix decimal -label "AND_Px2Py2" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_52

# ================================================================
# PASO 2: L1sq, L2sq
# inst: b2v_inst2 (L1*L1), b2v_inst3 (L2*L2)
# ================================================================
add wave -divider "=== PASO 2: L1sq L2sq ==="
add wave -radix decimal -label "L1sq"       sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_2
add wave -radix decimal -label "L2sq"       sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_3
add wave -radix decimal -label "valid_L1sq" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_49
add wave -radix decimal -label "valid_L2sq" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_50
add wave -radix decimal -label "AND_L1L2"   sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_1

# ================================================================
# PASO 3: b2 = Px2+Py2
# inst: b2v_inst6
# ================================================================
add wave -divider "=== PASO 3: b2 = Px2+Py2 ==="
add wave -radix decimal -label "b2"       sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_8
add wave -radix decimal -label "valid_b2" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_16

# ================================================================
# PASO 4: Lsum = L1sq+L2sq
# inst: b2v_inst10
# ================================================================
add wave -divider "=== PASO 4: Lsum = L1sq+L2sq ==="
add wave -radix decimal -label "Lsum"     sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_9
add wave -radix decimal -label "valid_Ls" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_17

# ================================================================
# PASO 5: AND valid_b2 AND valid_Ls -> start numerador
# ================================================================
add wave -divider "=== PASO 5: AND b2 AND Ls ==="
add wave -radix decimal -label "AND_b2_Ls" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_7

# ================================================================
# PASO 6: numerador = b2 - Lsum
# inst: b2v_inst12
# ================================================================
add wave -divider "=== PASO 6: numerador = b2-Lsum ==="
add wave -radix decimal -label "numerador" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_12
add wave -radix decimal -label "valid_num" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_21

# ================================================================
# PASO 7: Two*L1 y denominador = 2*L1*L2
# inst: b2v_inst9 (2*L1), b2v_inst11 (2*L1*L2)
# ================================================================
add wave -divider "=== PASO 7: den = 2*L1*L2 ==="
add wave -radix decimal -label "TwoL1"     sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_5
add wave -radix decimal -label "valid_2L1" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_4
add wave -radix decimal -label "den"       sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_11
add wave -radix decimal -label "valid_den" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_22

# ================================================================
# PASO 8: cos(t2) = num/den
# inst: b2v_inst14
# ================================================================
add wave -divider "=== PASO 8: cos(t2) ==="
add wave -radix decimal -label "AND_num_den" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_10
add wave -radix decimal -label "cos_t2"      sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_56
add wave -radix decimal -label "done_div"    sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_55

# ================================================================
# PASO 9: cos_t2^2
# inst: b2v_inst15
# ================================================================
add wave -divider "=== PASO 9: cos_t2^2 ==="
add wave -radix decimal -label "cos2_sq"    sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_20
add wave -radix decimal -label "valid_cos2" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_19

# ================================================================
# PASO 10: 1 - cos2
# inst: b2v_inst17
# ================================================================
add wave -divider "=== PASO 10: 1-cos2 ==="
add wave -radix decimal -label "one_mcos2" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_24
add wave -radix decimal -label "valid_1mc" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_23

# ================================================================
# PASO 11: sin(t2) = sqrt(1-cos2)
# inst: b2v_inst19
# ================================================================
add wave -divider "=== PASO 11: sin(t2) ==="
add wave -radix decimal -label "sin_t2"    sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_58
add wave -radix decimal -label "done_sqrt" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_57

# ================================================================
# PASO 12: theta2 = atan2(sin_t2, cos_t2)
# inst: b2v_inst21  [x_in=cos(WIRE_56), y_in=sin(WIRE_58)]
# done no conectado en este inst -> observar salida thetha2 directo
# ================================================================
add wave -divider "=== PASO 12: theta2 ==="
add wave -radix decimal -label "thetha2" sim:/Block2R_tb/thetha2_s

# ================================================================
# PASO 13: L2*cos_t2, L2*sin_t2
# inst: b2v_inst23 (L2*cos), b2v_inst24 (L2*sin)
# ================================================================
add wave -divider "=== PASO 13: L2*cos L2*sin ==="
add wave -radix decimal -label "L2cos_t2"    sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_34
add wave -radix decimal -label "valid_L2cos" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_33
add wave -radix decimal -label "L2sin_t2"    sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_37
add wave -radix decimal -label "valid_L2sin" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_39

# ================================================================
# PASO 14: k1 = L1 + L2*cos_t2
# inst: b2v_inst25
# ================================================================
add wave -divider "=== PASO 14: k1 = L1+L2cos ==="
add wave -radix decimal -label "k1"       sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_36
add wave -radix decimal -label "valid_k1" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_38

# ================================================================
# PASO 15: AND(valid_k1, valid_L2sin) -> start atan2 corr y base
# ================================================================
add wave -divider "=== PASO 15: AND k1 L2sin ==="
add wave -radix decimal -label "AND_k1_L2sin" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_59

# ================================================================
# PASO 16: angle_corr = atan2(L2*sin, k1)
# inst: b2v_inst27  [x_in=k1(WIRE_36), y_in=L2sin(WIRE_37)]
# ================================================================
add wave -divider "=== PASO 16: angle_corr ==="
add wave -radix decimal -label "angle_corr"      sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_43
add wave -radix decimal -label "done_angle_corr" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_45

# ================================================================
# PASO 17: angle_base = atan2(Px, Py)
# inst: b2v_inst30  [x_in=Py, y_in=Px]
# Nota: x_in=Py, y_in=Px -> atan2(Px,Py) correcto segun BDF
# ================================================================
add wave -divider "=== PASO 17: angle_base ==="
add wave -radix decimal -label "angle_base"      sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_44
add wave -radix decimal -label "done_angle_base" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_46

# ================================================================
# PASO 18: AND(done_corr, done_base) -> start resta final
# ================================================================
add wave -divider "=== PASO 18: AND corr base ==="
add wave -radix decimal -label "AND_corr_base" sim:/Block2R_tb/DUT/SYNTHESIZED_WIRE_42

# ================================================================
# PASO 19: theta1 = angle_corr - angle_base
# inst: b2v_inst31  [op=1 -> resta, a=corr(WIRE_43), b=base(WIRE_44)]
# ================================================================
add wave -divider "=== PASO 19: theta1 ==="
add wave -radix decimal -label "tetha1" sim:/Block2R_tb/tetha1_s

run 12000 ns
wave zoom full
