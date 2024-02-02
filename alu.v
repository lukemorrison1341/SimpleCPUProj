/*
Control 00 = add
Control 01 = subtract (using 2's complement)
Control 10 = AND
Control 11 = OR 
*/
`timescale 1ns/1ns
module alu (clk, a, b,control,outp,enable,carry_out,zero_flag,neg_flag);
input clk;
input [7:0] a; //8 bit input
input [7:0] b;
input [1:0] control; //2 bit op-code
input enable; //enable bit 
output [7:0] outp;
output carry_out;
output zero_flag;
output neg_flag;

reg [8:0] outp_temp;
reg carry_out_temp;
reg zero_flag_temp;
reg neg_flag_temp;

always @ (posedge clk) begin
    if(enable) begin
        case (control) 
            2'b00: begin 
                outp_temp <= {1'b0,a} + {1'b0,b}; //add
                carry_out_temp <= outp_temp[8]; 
                zero_flag_temp <= 0;
                neg_flag_temp <= 0;
            end
            2'b01: begin 
                outp_temp <= {1'b0,a} - {1'b0,b}; // subtract
                zero_flag_temp <= (outp_temp[7:0] == 0);
                carry_out_temp <= 0;
                neg_flag_temp <= outp_temp[8]; 
            end
            2'b10: begin
                outp_temp <= a & b; //AND
                zero_flag_temp <= 0;
                carry_out_temp <= 0;
                neg_flag_temp <= 0;
            end
            2'b11: begin
                outp_temp <= a | b; //OR
                zero_flag_temp <= 0;
                carry_out_temp <= 0;
                neg_flag_temp <= 0;
            end
            default: begin
                zero_flag_temp <= 1'bx;
                carry_out_temp <= 1'bx;
                neg_flag_temp <= 1'bx;
                outp_temp <= 9'bx;
            end
        endcase
    end
end

assign outp = outp_temp[7:0];
assign carry_out = carry_out_temp;
assign zero_flag = zero_flag_temp;
assign neg_flag = neg_flag_temp;

endmodule


/*
module alu_test();
reg clk;
reg [7:0] a;
reg [7:0] b;
wire [1:0] control;
wire enable;
assign enable = 1'b1;
wire [7:0] outp;

wire carry_out, neg_flag, zero_flag;

alu uut (.clk(clk),.a(a),.b(b),.control(2'b01),.outp(outp),.enable(enable),.carry_out(carry_out),.zero_flag(zero_flag),.neg_flag(neg_flag));




initial begin
$dumpfile("output.vcd");
$dumpvars(0,alu_test);
clk <= 0;
a <= 0;
b <= 0;
#1000;
$finish;
end

always #5 clk = ~clk;
always #5 a = a + 1;
always #2 b = b + 1;
endmodule


*/