/*
8-bit bus for the 8 bit CPU. 
*/

`timescale 1ps/1ps
module bus (in,enable,data); //used for the data bus. 
inout [7:0] data;
input [7:0] in;
input enable;
//Drive the bus when enabled
assign data = enable ? in : 8'bz; //High-impedence if nothing driving the bus. 
endmodule

/*
module bus_test;

// Inputs
reg clk;
reg [7:0] data_in;
reg out_enable;
reg in_enable;

// Outputs
wire [7:0] data_out;

// Instantiate the register module
register my_register (
    .clk(clk), 
    .in(data_in), 
    .out_enable(out_enable), 
    .in_enable(in_enable), 
    .outp(data_out)
);

// Instantiate the bus module
bus my_bus (
    .in(data_in), 
    .out(data_out), 
    .enable(out_enable)
);

// Clock generation
initial begin
    clk = 0;
    forever #10 clk = ~clk; // 50MHz clock
end

// Test sequence
initial begin
    // Initialize Inputs
    data_in = 0;
    out_enable = 0;
    in_enable = 0;

    // Wait for global reset
    #100;

    // Write data to the register
    data_in = 8'hAA; // Example data
    in_enable = 1; 
    #20;
    in_enable = 0; 

    // Read data from the register
    #40;
    out_enable = 1;
    #20;
    out_enable = 0;

    // Complete the test
    #100;
    $finish;
end

initial begin
$dumpfile("output.vcd");
$dumpvars(0,bus_test);
#1000;
$finish;
end

endmodule
*/