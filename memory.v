module memory(clk,R_W,addr,data,outp,enable);
input clk;
input R_W; //1 = Read, 0 = Write
input [15:0] addr;
input [7:0] data;
output reg [7:0] outp;
input enable;


reg [7:0]ram[0:65535]; //represents all of possible RAM (64kbytes)

//reg [7:0] outp_temp;
integer i;
initial begin
//integer counter = 0;

    ram[0] = 8'hf1;
    ram[1] = 8'hff;
    for(i = 2; i< 65536; i = i + 1)
    begin 
        //counter = counter + 1;
        ram[i] = 8'hff;
    end

end

always @ (R_W or enable or addr) begin
    if(enable)
        if(R_W) outp <= ram[addr]; //Want to read
        else begin ram[addr] <= data;
            outp <= 8'bz; //Disconnect outp to not drive the bus when only writing to memory.
        end
    else outp <= 8'bz;
end

//assign outp = outp_temp;


endmodule