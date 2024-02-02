#!/bin/bash

# Define the output executable name
OUTPUT="cpu_simulation"

# Compile the Verilog files with iverilog
iverilog -o $OUTPUT cpu.v reg.v bus.v addr_bus.v memory.v pc_reg.v
compile_status=$?

if [ $compile_status -ne 0 ]; then
    echo "Compilation failed with status $compile_status. Exiting..."
    exit $compile_status
fi


# Optional: Run the simulation automatically
vvp $OUTPUT

# Optional: Open the waveform with a viewer like GTKWave (if you have generated a VCD file)
# gtkwave waveform.vcd
