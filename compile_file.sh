#!/bin/bash

# Check if at least one file name was provided
if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <Main Verilog file> [Other Verilog files...]"
    exit 1
fi

echo "Starting compilation..."
# Compile the Verilog code (including all specified files)
iverilog -o output.vvp "$@"
compile_status=$?

# Check if iverilog compilation was successful
if [ $compile_status -ne 0 ]; then
    echo "Compilation failed with status $compile_status. Exiting..."
    exit $compile_status
fi

echo "Compilation successful. Starting simulation..."
# Run the simulation
vvp output.vvp
sim_status=$?

# Check if the simulation was successful
if [ $sim_status -ne 0 ]; then
    echo "Simulation failed with status $sim_status. Exiting..."
    exit $sim_status
fi

echo "Simulation successful. Opening GTKWave..."
# Open the results in GTKWave
gtkwave output.vcd &
