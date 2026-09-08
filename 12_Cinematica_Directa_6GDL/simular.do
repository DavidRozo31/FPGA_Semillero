# =============================================================
#  simular.do  (nombre fijo — se sobrescribe en cada paso)
#  ModelSim — FK_7R_Geometrico_Con_Posicion, INTEGRACION COMPLETA
#  (metodo geometrico recursivo, correccion del profesor)
# =============================================================

if {[file exists work]} { vdel -all -lib work }
vlib work
vmap work work

echo "--- Compilando paquetes ---"
vcom -93 -work work cordic_pkg.vhd
vcom -93 -work work ik_pkg.vhd
vcom -93 -work work geom_pkg.vhd

echo "--- Compilando modulos ---"
vcom -93 -work work cordic_sincos_16.vhd
vcom -93 -work work cordic_atan2_16.vhd
vcom -93 -work work cordic_seq6.vhd
vcom -93 -work work fk_recursivo_core.vhd
vcom -93 -work work atan2_seq3.vhd
vcom -93 -work work FK_7R_Geometrico_Con_Posicion.vhd

echo "--- Compilando testbench ---"
vcom -93 -work work tb.vhd

echo "--- Iniciando simulacion ---"
vsim -t 1ns -lib work tb

add wave -divider "=== CONTROL ==="
add wave -radix unsigned sim:/tb/clk_in
add wave -radix unsigned sim:/tb/rst_in
add wave -radix unsigned sim:/tb/start_in
add wave -radix unsigned sim:/tb/done_out

add wave -divider "=== ENTRADAS (theta, Q2.13 rad) ==="
add wave -radix decimal sim:/tb/theta1_in
add wave -radix decimal sim:/tb/theta2_in
add wave -radix decimal sim:/tb/theta3_in
add wave -radix decimal sim:/tb/theta4_in
add wave -radix decimal sim:/tb/theta5_in
add wave -radix decimal sim:/tb/theta6_in

add wave -divider "=== SALIDA: POSICION (x,y,z) ==="
add wave -radix decimal sim:/tb/x_out
add wave -radix decimal sim:/tb/y_out
add wave -radix decimal sim:/tb/z_out

add wave -divider "=== SALIDA: ORIENTACION (yaw,pitch,roll) ==="
add wave -radix decimal sim:/tb/yaw_out
add wave -radix decimal sim:/tb/pitch_out
add wave -radix decimal sim:/tb/roll_out

run 30000 ns
wave zoom full

echo ""
echo "============================================"
echo "FK_7R_Geometrico_Con_Posicion -- integracion completa, 6 casos (A-F)"
echo "Los 6 casos coinciden con los del STM32 (leccion 13): A=Caso1 B=Caso2"
echo "C=Caso3 D=EXTRA-2 E=EXTRA-1 F=EXTRA-3 -- ver los ciclos hasta done_out"
echo "de cada uno arriba: deben ser identicos entre si (CORDIC es constante)."
echo "============================================"
