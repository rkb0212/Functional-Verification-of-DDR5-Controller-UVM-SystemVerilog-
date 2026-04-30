#!/bin/bash
set -e

echo "=== [1/3] Compiling DPI-C golden model ==="
g++ -fPIC -shared \
    -std=c++11 \
    -I"$RIVIERA_HOME/interfaces/include" \
    -o ddr5_dpi_model.so \
    ddr5_dpi_model.cpp

echo "=== [2/3] Compiling SystemVerilog ==="
vlib work

vlog -sv -timescale 1ns/1ns \
    +incdir+$RIVIERA_HOME/vlib/uvm-1.2/src \
    +incdir+. \
    -l uvm_1_2 \
    design.sv \
    testbench.sv

echo "=== [3/3] Running simulation ==="
vsim -c -do run.do