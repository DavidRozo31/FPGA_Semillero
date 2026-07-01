# =============================================================
#  simular_ik2r.do
#  ModelSim ASE 10.5b - Quartus 18.1
#
#  BLOQUE 1 CYAN   : fp_multiplier
#  BLOQUE 2 AZUL   : fp_adder
#  BLOQUE 3 LILA   : sqrt_q13
#  BLOQUE 4 NARANJA: cordic_atan2
#  BLOQUE 5 VERDE  : cordic_sincos_16
#
#  Uso: do simular_ik2r.do
# =============================================================

quit -sim

cd C:/intelFPGA_lite/18.1/Projects/2R_Cinematic

if {[file exists work]} {
    vmap work work
} else {
    vlib work
    vmap work work
}

# -------------------------------------------------------------
#  COMPILAR - orden obligatorio: paquetes primero
# -------------------------------------------------------------
echo ">>> Compilando cordic_pkg.vhd ..."
vcom -2008 -work work cordic_pkg.vhd

echo ">>> Compilando ik_pkg.vhd ..."
vcom -2008 -work work ik_pkg.vhd

echo ">>> Compilando fp_multiplier.vhd ..."
vcom -2008 -work work fp_multiplier.vhd

echo ">>> Compilando fp_adder.vhd ..."
vcom -2008 -work work fp_adder.vhd

echo ">>> Compilando sqrt_q13.vhd ..."
vcom -2008 -work work sqrt_q13.vhd

echo ">>> Compilando cordic_atan2.vhd ..."
vcom -2008 -work work cordic_atan2.vhd

echo ">>> Compilando cordic_sincos_16.vhd ..."
vcom -2008 -work work cordic_sincos_16.vhd

echo ">>> Compilando tb_inverse_kinematics.vhd ..."
vcom -2008 -work work tb_inverse_kinematics.vhd

# -------------------------------------------------------------
#  Cargar simulacion
# -------------------------------------------------------------
echo ">>> Cargando simulacion ..."
vsim -t 1ns -novopt work.tb_inverse_kinematics

# -------------------------------------------------------------
# -------------------------------------------------------------
#  Limpiar ondas anteriores

# =============================================================
#  BLOQUE 0 - CONTROL GENERAL
# =============================================================
add wave -divider "################################################################"
add wave -divider "  CONTROL GENERAL"
add wave -divider "################################################################"

add wave -color "White" \
         -radix decimal \
         -label "SECCION ACTIVA" \
         /tb_inverse_kinematics/seccion_activa

add wave -color "Gold" \
         -label "clk" \
         /tb_inverse_kinematics/clk

add wave -color "Tomato" \
         -label "rst" \
         /tb_inverse_kinematics/rst

# =============================================================
#  BLOQUE 1 - fp_multiplier   CYAN
# =============================================================
add wave -divider "################################################################"
add wave -divider "  BLOQUE 1 : fp_multiplier   A x B -> P   Q2.13"
add wave -divider "################################################################"

add wave -color "Cyan" \
         -radix decimal \
         -label "MUL | A_in raw" \
         /tb_inverse_kinematics/mul_a

add wave -color "Cyan" \
         -radix decimal \
         -label "MUL | B_in raw" \
         /tb_inverse_kinematics/mul_b

add wave -color "PaleTurquoise" \
         -label "MUL | valid_i" \
         /tb_inverse_kinematics/mul_vi

add wave -color "DarkKhaki" \
         -radix decimal \
         -label "MUL | prod_reg interno Q4.26" \
         /tb_inverse_kinematics/DUT_MUL/prod_reg

add wave -color "LimeGreen" \
         -radix decimal \
         -label "MUL | P_out raw" \
         /tb_inverse_kinematics/mul_p

add wave -color "PaleTurquoise" \
         -label "MUL | valid_o" \
         /tb_inverse_kinematics/mul_vo

# =============================================================
#  BLOQUE 2 - fp_adder   AZUL
# =============================================================
add wave -divider "################################################################"
add wave -divider "  BLOQUE 2 : fp_adder   op0 SUMA op1 RESTA   Q2.13"
add wave -divider "################################################################"

add wave -color "SkyBlue" \
         -radix decimal \
         -label "ADD | A_in raw" \
         /tb_inverse_kinematics/add_a

add wave -color "SkyBlue" \
         -radix decimal \
         -label "ADD | B_in raw" \
         /tb_inverse_kinematics/add_b

add wave -color "Orange" \
         -label "ADD | op 0=suma 1=resta" \
         /tb_inverse_kinematics/add_op

add wave -color "LightSteelBlue" \
         -label "ADD | valid_i" \
         /tb_inverse_kinematics/add_vi

add wave -color "DodgerBlue" \
         -radix decimal \
         -label "ADD | P_out raw" \
         /tb_inverse_kinematics/add_p

add wave -color "LightSteelBlue" \
         -label "ADD | valid_o" \
         /tb_inverse_kinematics/add_vo

# =============================================================
#  BLOQUE 3 - sqrt_q13   LILA
# =============================================================
add wave -divider "################################################################"
add wave -divider "  BLOQUE 3 : sqrt_q13   latencia 18 ciclos   Q2.13"
add wave -divider "################################################################"

add wave -color "Plum" \
         -radix decimal \
         -label "SQRT | X_in raw" \
         /tb_inverse_kinematics/sq_x

add wave -color "Violet" \
         -label "SQRT | start" \
         /tb_inverse_kinematics/sq_start

add wave -color "DarkKhaki" \
         -radix unsigned \
         -label "SQRT | rem_s interno" \
         /tb_inverse_kinematics/DUT_SQRT/rem_s

add wave -color "DarkKhaki" \
         -radix unsigned \
         -label "SQRT | root_s interno" \
         /tb_inverse_kinematics/DUT_SQRT/root_s

add wave -color "Orchid" \
         -radix decimal \
         -label "SQRT | cnt iteracion" \
         /tb_inverse_kinematics/DUT_SQRT/cnt

add wave -color "Magenta" \
         -radix decimal \
         -label "SQRT | Y_out raw" \
         /tb_inverse_kinematics/sq_y

add wave -color "Violet" \
         -label "SQRT | done" \
         /tb_inverse_kinematics/sq_done

# =============================================================
#  BLOQUE 4 - cordic_atan2   NARANJA
# =============================================================
add wave -divider "################################################################"
add wave -divider "  BLOQUE 4 : cordic_atan2   atan2 Y X -> rad   Q2.13"
add wave -divider "################################################################"

add wave -color "Orange" \
         -radix decimal \
         -label "ATAN2 | X_in raw" \
         /tb_inverse_kinematics/at_x

add wave -color "Orange" \
         -radix decimal \
         -label "ATAN2 | Y_in raw" \
         /tb_inverse_kinematics/at_y

add wave -color "Gold" \
         -label "ATAN2 | start" \
         /tb_inverse_kinematics/at_start

add wave -color "DarkKhaki" \
         -radix decimal \
         -label "ATAN2 | z_pipe 0 inicial" \
         /tb_inverse_kinematics/DUT_ATAN2/z_pipe(0)

add wave -color "DarkKhaki" \
         -radix decimal \
         -label "ATAN2 | z_pipe 6 mitad" \
         /tb_inverse_kinematics/DUT_ATAN2/z_pipe(6)

add wave -color "Coral" \
         -radix decimal \
         -label "ATAN2 | angle_out raw rad" \
         /tb_inverse_kinematics/at_angle

add wave -color "Gold" \
         -label "ATAN2 | done" \
         /tb_inverse_kinematics/at_done

# =============================================================
#  BLOQUE 5 - cordic_sincos_16   VERDE
# =============================================================
add wave -divider "################################################################"
add wave -divider "  BLOQUE 5 : cordic_sincos_16   sin cos del angulo   Q2.13"
add wave -divider "################################################################"

add wave -color "MediumSeaGreen" \
         -radix decimal \
         -label "SINCOS | angle_in raw rad" \
         /tb_inverse_kinematics/sc_angle

add wave -color "SpringGreen" \
         -label "SINCOS | start" \
         /tb_inverse_kinematics/sc_start

add wave -color "DarkKhaki" \
         -radix decimal \
         -label "SINCOS | x_pipe 0 K inicial" \
         /tb_inverse_kinematics/DUT_SINCOS/x_pipe(0)

add wave -color "DarkKhaki" \
         -radix decimal \
         -label "SINCOS | z_pipe 0 angulo reducido" \
         /tb_inverse_kinematics/DUT_SINCOS/z_pipe(0)

add wave -color "DarkKhaki" \
         -radix decimal \
         -label "SINCOS | z_pipe 6 mitad pipeline" \
         /tb_inverse_kinematics/DUT_SINCOS/z_pipe(6)

add wave -color "LimeGreen" \
         -radix decimal \
         -label "SINCOS | sin_out raw" \
         /tb_inverse_kinematics/sc_sin

add wave -color "MediumSpringGreen" \
         -radix decimal \
         -label "SINCOS | cos_out raw" \
         /tb_inverse_kinematics/sc_cos

add wave -color "SpringGreen" \
         -label "SINCOS | done" \
         /tb_inverse_kinematics/sc_done

# -------------------------------------------------------------
#  Correr
# -------------------------------------------------------------
echo ">>> Corriendo simulacion ..."
run -all
wave zoom full

echo ""
echo "################################################################"
echo "  Listo."
echo "  BLANCO  -> Seccion activa 1 a 5"
echo "  CYAN    -> BLOQUE 1 fp_multiplier"
echo "  AZUL    -> BLOQUE 2 fp_adder"
echo "  LILA    -> BLOQUE 3 sqrt_q13"
echo "  NARANJA -> BLOQUE 4 cordic_atan2"
echo "  VERDE   -> BLOQUE 5 cordic_sincos_16"
echo "################################################################"
