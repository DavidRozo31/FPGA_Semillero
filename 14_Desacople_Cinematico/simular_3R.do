quit -sim
cd "C:/intelFPGA_lite/18.1/Projects/Desacople_Cinematico_6R"
if {[file exists work]} {
    vdel -all
}
vlib work
vmap work work
vcom -93 cordic_pkg.vhd
vcom -93 cordic_atan2.vhd
vcom -93 fp_multiplier.vhd
vcom -93 fp_adder.vhd
vcom -93 fp_divider.vhd
vcom -93 sqrt_q13.vhd
vcom -93 pulse_join2.vhd
vcom -93 Cinematica_Inversa3R.vhd
vcom -93 tb_Cinematica_Inversa3R.vhd
vsim work.tb_Cinematica_Inversa3R
add wave -divider "Control"
add wave -color yellow -label clk    /tb_cinematica_inversa3r/clk_tb
add wave -color yellow -label reset  /tb_cinematica_inversa3r/reset_tb
add wave -color yellow -label start  /tb_cinematica_inversa3r/start_tb
add wave -color orange -label ready  /tb_cinematica_inversa3r/ready_tb
add wave -divider "Entradas"
add wave -color cyan -radix decimal -label Px /tb_cinematica_inversa3r/px_tb
add wave -color cyan -radix decimal -label Py /tb_cinematica_inversa3r/py_tb
add wave -color cyan -radix decimal -label Pz /tb_cinematica_inversa3r/pz_tb
add wave -divider "Salidas"
add wave -color orange -label Ik3R_Ready /tb_cinematica_inversa3r/ready_tb
add wave -color green -radix decimal -label theta1 /tb_cinematica_inversa3r/theta1_tb
add wave -color green -radix decimal -label theta2 /tb_cinematica_inversa3r/theta2_tb
add wave -color green -radix decimal -label theta3 /tb_cinematica_inversa3r/theta3_tb
run 80000000 ns
wave zoom full