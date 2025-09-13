vlib work
vlog +acc main.v testbench.v
vsim -novopt testbench
add wave -r /*
run 20 ms
