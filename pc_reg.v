module pc_register(clk,in,out_enable,in_enable,outp);
input clk;
input [7:0] in;
input out_enable;
input in_enable;
output [7:0] outp; //output contents of reg to bus, or anything else

reg [7:0] data;
reg [7:0] out_temp;


initial begin 
    data = 8'b0;
    out_temp = 8'bx;
end

always @ (posedge clk) begin //Output priority 
    if(out_enable) out_temp <= data;
    if(in_enable) data = in;
end

assign outp = out_temp;
endmodule
