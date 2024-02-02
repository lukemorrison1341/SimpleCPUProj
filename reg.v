/*
8-bit register. 
*/

`timescale 1ns/1ns
/*
module register(clk,in,out_enable,in_enable,outp);
input clk;
input [7:0] in;
input out_enable;
input in_enable;
output [7:0] outp; //output contents of reg to bus, or anything else

reg [7:0] data;
reg [7:0] out_temp;


initial begin 
    data = 8'b0;
    out_temp = 8'b0;
end

always @ (out_enable or in_enable) begin //Output priority, changes output based on  
    if(out_enable) out_temp <= data;
    if(in_enable) data = in;
end

assign outp = out_temp;
endmodule
*/

module register(clk, in, out_enable, in_enable, outp);
    input clk;
    input [7:0] in;
    input out_enable;
    input in_enable;
    output [7:0] outp;

    reg [7:0] data; //The actual register value

    initial begin
        data = 8'b0;
    end

    // Use a tri-state buffer approach for output
    assign outp = out_enable ? data : 8'bz;

    always @(out_enable or in_enable) begin
        if (in_enable) begin
            data <= in; // Load data into the register
        end
        // No need to handle out_enable here as it's used in the assign statement
    end
endmodule


/*

module reg_test();
reg clk;
reg in_enable;
reg out_enable;
wire [7:0] outp;
reg [7:0] data;


initial begin
$dumpfile("output.vcd");
$dumpvars(0,reg_test);
data = 8'b00000000;
clk = 0;
in_enable = 0;
out_enable = 0;

#1000;
$finish;
end
register uut(.clk(clk),.in(data),.out_enable(out_enable),.in_enable(in_enable),.outp(outp));


always #5 clk = ~clk;
always #10 in_enable = ~in_enable;
always #15 out_enable = ~out_enable;
always #25 data = data + 1;


endmodule


*/