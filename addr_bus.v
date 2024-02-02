module addr_bus (in,enable,out); //16 bit address bus as opposed to 8 bit data bus
input wire [15:0] in;
input enable;
output wire [15:0] out;
assign out = enable ? in : 16'bz; //High-impedence if nothing driving the bus. 
endmodule
