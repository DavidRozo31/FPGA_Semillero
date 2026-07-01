transcript on
if {[file exists rtl_work]} {
	vdel -lib rtl_work -all
}
vlib rtl_work
vmap work rtl_work

vcom -93 -work work {C:/intelFPGA_lite/18.1/Projects/2R_Cinematic/Block2RTest.vhd}
vcom -93 -work work {C:/intelFPGA_lite/18.1/Projects/2R_Cinematic/sqrt_q13.vhd}
vcom -93 -work work {C:/intelFPGA_lite/18.1/Projects/2R_Cinematic/fp_multiplier.vhd}
vcom -93 -work work {C:/intelFPGA_lite/18.1/Projects/2R_Cinematic/fp_divider.vhd}
vcom -93 -work work {C:/intelFPGA_lite/18.1/Projects/2R_Cinematic/fp_adder.vhd}
vcom -93 -work work {C:/intelFPGA_lite/18.1/Projects/2R_Cinematic/cordic_pkg.vhd}
vcom -93 -work work {C:/intelFPGA_lite/18.1/Projects/2R_Cinematic/cordic_atan2.vhd}

vcom -93 -work work {C:/intelFPGA_lite/18.1/Projects/2R_Cinematic/Block2R_tb.vhd}

vsim -t 1ps -L altera -L lpm -L sgate -L altera_mf -L altera_lnsim -L cycloneive -L rtl_work -L work -voptargs="+acc"  Block2R_tb

do C:/intelFPGA_lite/18.1/Projects/2R_Cinematic/simular2R.do
