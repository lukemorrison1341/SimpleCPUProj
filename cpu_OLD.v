/*
Multi-cycle 8 bit CPU with 8 internal registers, 
Program counter high, Program counter low, an instruction register, accumulator register, flags register, stack register, and general purpose register's X and Y.

Flags register takes format of :

CZNH IV x x x

C = carry flag
Z = zero flag
N = negative flag
H = halt flag. Halt's processor if set to 1.
IV = Invalid Instruction/Opcode 

INSTRUCTION REGISTER for first cycle takes format of :
oooo ooaa;
o= opcode
a = addressing mode. Either immediate, implied, register or direct address.


64 possible opcodes: (1st cycle of decoding instruction)

ADD - 0000 00
SUB - 0000 01
AND - 0000 10
OR

BEQ
BNEG
BZ
BIV ; Branch on invalid op-code instruction ran. Could be used for error prevention

JMP 

SEC ; Set carry flag to 1
SNEG ; Set negative flag to 1
SEZ ;  Set zero flag to 1
SEH ; Set halt flag to 1. Effectively ends a program

CLC ; Clear the carry flag
CNEG ; Clear the negative flag
CEZ ; Clear the zero flag
CIV ; Clear invalid op-code flag 

SHA ; Shift Accumulator register
SHX ; Shift X register
SHY ; Shift Y register

PHA ;Push A onto stack
PHX ; Push B onto stack
PHY ; Push Y onto stack

POA ; Pop from stack into A 
POX ; Pop from stack into X
POY ; Pop from stack into Y

MOVX ;Move X to mem address
MOVY ;Move Y to mem address

LDX
LDY
LDA

TXA ; Transfer X to A
TYA ; Transfer Y to A

TXY ; Transfer X to Y
TYX ; Transfer Y to X

TAX ; Transfer A to X
TAY ; Transfer A to Y


4 addressing modes supported : 

Immediate - a = 00
Implied - a = 01
Register - a = 10
Direct Address - a = 11


For immediate:
One extra cycle for immediate data. 
iiii iiii is all 8 bits of immediate data.

For implied: No extra cycle needed.

For register: 
One extra cycle needed to index register.
xxxx xrrr
r = register index.

For Memory: 
Two extra cycle's needed to retrieve mem address.
hhhh hhhh - High address bits. - 2nd cycle
llll llll - Low address bits. - 3rd cycle
*/


module cpu(clk,data_in,address,data_out,R_W);
input clk;
input wire [7:0] data_in;
output [15:0] address;
output [7:0] data_out;
output R_W; // Read or write signal for memory.

//All register enable bits used for bu  

reg PC_high_out_enable;
reg PC_high_in_enable;
reg PC_low_out_enable;
reg PC_low_in_enable;

reg IR_out_enable;
reg IR_in_enable;
//Accumulator register
reg A_out_enable;
reg A_in_enable;
// X register
reg X_out_enable;
reg X_in_enable;
//Y register
reg Y_out_enable;
reg Y_in_enable;
//Flags register
reg FL_out_enable;
reg FL_in_enable;
//Stack register
reg ST_out_enable;
reg ST_in_enable;


initial begin //set all control signals to 0
    PC_high_out_enable <= 1'b0;
    PC_high_in_enable <= 1'b0;
    PC_low_out_enable <= 1'b0;
    PC_low_in_enable <= 1'b0;
    IR_out_enable <= 1'b0;
    IR_in_enable <= 1'b0;
    IR_out_enable <= 1'b0;
    A_out_enable <= 1'b0;
    A_in_enable <= 1'b0;
    X_out_enable <= 1'b0;
    X_in_enable <= 1'b0;
    Y_out_enable <= 1'b0;
    Y_in_enable <= 1'b0;
    FL_out_enable <= 1'b0;
    FL_in_enable <= 1'b0;
    ST_out_enable <= 1'b0;
    ST_in_enable <= 1'b0;
end



//Bus variables

wire [7:0] data_bus_in;

wire [7:0] data_bus; //Acts as intermediary to support bi-directionality of data bus.

wire [15:0] addr_bus_in;
wire [15:0] addr_bus_out;


wire DRIVING_d_bus;
wire DRIVING_addr_bus;


//bus d_bus(.in(data_bus_in),.enable(DRIVING_d_bus),.data(data_bus));


addr_bus a_bus(.in(addr_bus_in),.enable(1'b1),.out(addr_bus_out));

//Memory
reg R_W_reg;
reg MEM_ENABLE;
wire [7:0] mem_outp;
memory mem(.clk(clk),.R_W(R_W),.addr(addr_bus_out),.data(data_bus),.outp(mem_outp),.enable(MEM_ENABLE));


//All registers
register PC_high(.clk(clk),.in(data_bus),.out_enable(PC_high_out_enable),.in_enable(PC_high_in_enable),.outp(data_bus));
register PC_low(.clk(clk),.in(data_bus),.out_enable(PC_low_out_enable),.in_enable(PC_low_in_enable),.outp(data_bus));


register INST_REG(.clk(clk),.in(data_bus),.out_enable(IR_out_enable),.in_enable(IR_in_enable),.outp(data_bus));
register ACC_REG(.clk(clk),.in(data_bus),.out_enable(A_out_enable),.in_enable(A_in_enable),.outp(data_bus));
register X_REG(.clk(clk),.in(data_bus),.out_enable(X_out_enable),.in_enable(X_in_enable),.outp(data_bus));
register Y_REG(.clk(clk),.in(data_bus),.out_enable(Y_out_enable),.in_enable(Y_in_enable),.outp(data_bus));
register FL_REG(.clk(clk),.in(data_bus),.out_enable(FL_out_enable),.in_enable(FL_in_enable),.outp(data_bus));
register ST_REG(.clk(clk),.in(data_bus),.out_enable(ST_out_enable),.in_enable(ST_in_enable),.outp(data_bus));


//Create MUX for address bus and data bus.

parameter 
NUM_ADDRESS_BUS_SELECTIONS = 2, SELECT_PC_ADDR_BUS = 2'b00,SELECT_SP_ADDR_BUS = 2'b01, SELECT_MEM_ADDR_BUS = 2'b10;

reg [15:0] FULL_PC_ADDR;
reg [15:0] FULL_SP_ADDR;
reg [15:0] FULL_MEM_ADDR;
reg [NUM_ADDRESS_BUS_SELECTIONS-1:0] ADDRESS_SELECT_MUX;
//address bus MUX

assign addr_bus_in = (ADDRESS_SELECT_MUX == SELECT_PC_ADDR_BUS) ? FULL_PC_ADDR : (ADDRESS_SELECT_MUX == SELECT_SP_ADDR_BUS) ? FULL_SP_ADDR : (ADDRESS_SELECT_MUX == SELECT_MEM_ADDR_BUS) ? FULL_MEM_ADDR : 16'b0;
/*

TODO: Add more lines to the data bus MUX. More things need to go on the data bus

*/
//data bus MUX
parameter 
NUM_DATA_BUS_SELECTIONS = 8, SELECT_A_DATA_BUS = 3'b000, SELECT_X_DATA_BUS = 3'b001, SELECT_Y_DATA_BUS = 3'b010, SELECT_FL_DATA_BUS = 3'b011, SELECT_IR_DATA_BUS = 3'b100, SELECT_PC_LOW_DATA_BUS = 3'b101, SELECT_PC_HIGH_DATA_BUS = 3'b110;

reg [7:0] ACC_REG_TEMP; 
reg [7:0] X_REG_TEMP; 
reg [7:0] Y_REG_TEMP; 
reg [7:0] FL_REG_TEMP; //Add more as needed.  
reg [7:0] INST_REG_TEMP;
reg [7:0] PC_HIGH_REG_TEMP;
reg [7:0] PC_LOW_REG_TEMP;
initial begin
    //data_bus_temp <= 0;
    MEM_ENABLE <= 0;
    DATA_MUX_ENABLE <= 0;
    //DATA_SELECT_MUX <= 0;
    ACC_REG_TEMP <= 8'b0;
    X_REG_TEMP <= 8'b0;
    Y_REG_TEMP <= 8'b0;
    FL_REG_TEMP <= 8'b0;
    INST_REG_TEMP <= 8'b0;
    PC_HIGH_REG_TEMP <= 8'b0;
    PC_LOW_REG_TEMP <= 8'b0;

end

reg [NUM_DATA_BUS_SELECTIONS-1:0] DATA_SELECT_MUX;
reg DATA_MUX_ENABLE; //Is high when wanting to drive a register/something else onto the data bus. Has to wait for MEM_ENABLE And R_W_reg to both be low.
reg ADDR_MUX_ENABLE; 

assign data_bus_in = (DATA_SELECT_MUX == SELECT_A_DATA_BUS) ? ACC_REG_TEMP : (DATA_SELECT_MUX == SELECT_X_DATA_BUS) ? X_REG_TEMP : (DATA_SELECT_MUX == SELECT_Y_DATA_BUS) ? Y_REG_TEMP 
: (DATA_SELECT_MUX == SELECT_FL_DATA_BUS) ? FL_REG_TEMP : (DATA_SELECT_MUX == SELECT_IR_DATA_BUS) ? INST_REG_TEMP : 
(DATA_SELECT_MUX == SELECT_PC_HIGH_DATA_BUS) ? PC_HIGH_REG_TEMP: (DATA_SELECT_MUX == SELECT_PC_LOW_DATA_BUS) ? PC_LOW_REG_TEMP : 8'bz; 
//Leave data_bus_in Unconnected from data_bus if not trying to input into the bus.

reg [7:0] data_bus_temp;

initial data_bus_temp <= data_bus;


always @* begin
    if(DATA_MUX_ENABLE)
        data_bus_temp <= data_bus_in;
end
assign data_bus = data_bus_temp;


//assign data_bus = (MEM_ENABLE == 1 && R_W_reg == READ) ? mem_outp: DATA_MUX_ENABLE ? data_bus_in : 8'bz; 


parameter //FETCH STATES
    FETCH_INIT = 4'b0000, FETCH_READ = 4'b0001, FETCH_LOAD_IR = 4'b0010, FETCH_INCR_IR = 4'b0011, FETCH_OUTP_HIGH_PCBYTE = 4'b0100,
    FETCH_OUTP_LOW_PCBYTE = 4'b0101;
parameter //Memory states
    READ = 1'b1, WRITE = 1'b0;


reg [4:0] curr_state; 
initial curr_state <= FETCH_INIT;

reg [5:0] OPCODE;
reg [1:0] ADDR_MODE;
reg [1:0] ALU_OP;

reg [7:0] CURR_DATA;

//Perform Fetch-Decode-Execute

//Fetch

always @ (posedge clk) begin
    case(curr_state) 
    FETCH_INIT: begin
        PC_high_out_enable <= 1'b1;
        curr_state <= FETCH_READ;
    end
    FETCH_READ: begin
        PC_high_out_enable <= 1'b0;
        PC_low_out_enable <= 1'b1;
        FULL_PC_ADDR[15:8] <= data_bus;
        curr_state <= FETCH_LOAD_IR;
    end
    FETCH_LOAD_IR: begin
        IR_in_enable <= 1'b1;
        MEM_ENABLE <= 1'b1;
        R_W_reg <= READ;
        PC_low_out_enable <= 1'b0;
        FULL_PC_ADDR[7:0] <= data_bus;
        ADDRESS_SELECT_MUX <= SELECT_MEM_ADDR_BUS;
        FULL_MEM_ADDR <= FULL_PC_ADDR;
        curr_state <= FETCH_INCR_IR;

    end
    FETCH_INCR_IR: begin
        IR_in_enable <= 1'b0;
        MEM_ENABLE <= 1'b1;
        FULL_PC_ADDR <= FULL_PC_ADDR + 1;
        DATA_MUX_ENABLE <= 1'b1;
        DATA_SELECT_MUX <= SELECT_PC_HIGH_DATA_BUS;
        PC_high_in_enable <= 1'b1;
        curr_state <= FETCH_OUTP_HIGH_PCBYTE;
    end
    FETCH_OUTP_HIGH_PCBYTE: begin
        PC_HIGH_REG_TEMP <= FULL_PC_ADDR[15:8];
        DATA_SELECT_MUX <= SELECT_PC_LOW_DATA_BUS;
        PC_high_in_enable <= 1'b0;
        PC_low_in_enable <= 1'b1;
        curr_state <= FETCH_OUTP_LOW_PCBYTE;

    end
    FETCH_OUTP_LOW_PCBYTE: begin
        PC_LOW_REG_TEMP <= FULL_PC_ADDR[7:0];
        PC_low_in_enable <= 1'b0;
        DATA_MUX_ENABLE <= 1'b0;
        curr_state <= FETCH_INIT;
    end

    endcase

end

//Decode
always @ (posedge clk) begin

end
//Execute
always @ (posedge clk) begin

end



assign address = addr_bus_in;
assign data_out = data_bus_in;
assign data_in = data_bus;

//add more things that can drive the data bus
assign DRIVING_d_bus = (R_W_reg == READ && MEM_ENABLE) | IR_out_enable | A_out_enable | X_out_enable | Y_out_enable | FL_out_enable | ST_out_enable; //all of these drive to dbus
//assign DRIVING_addr_bus = PC_high_out_enable | PC_low_out_enable; //all of these drive to addr bus
assign R_W = R_W_reg;
endmodule



module cpu_test();

reg clk;
wire R_W;
reg [7:0] data_in;
wire [15:0] address;
wire [7:0] data_out;

cpu c(.clk(clk),.data_in(data_in),.address(address),.data_out(data_out),.R_W(R_W));

initial begin
    clk = 0;
    $monitor("Clock: %b ADDR_BUS_in %h ADDR_BUS_OUT %h, FULL_MEM_ADDR %h FULL_PC_ADDR %h data_bus_in %h data_bus_temp %h data_bus %h MEM_INST_OUTP %h, MEM_ENABLE: %b MEM_ADDR %h MUX ENABLE:%b MEM R/W%b DRIVING_DBUS %b IR_REG%h\n",clk,c.addr_bus_in,c.addr_bus_out,c.FULL_MEM_ADDR,c.FULL_PC_ADDR,c.data_bus_in,c.data_bus_temp,c.data_bus,c.mem_outp,c.MEM_ENABLE,c.mem.addr,c.DATA_MUX_ENABLE,c.mem.R_W,c.DRIVING_d_bus,c.INST_REG.data);
end


initial begin
#12;
$finish;
end
always #1 clk = ~clk;
always #1 $monitor("Clock: %b ADDR_BUS_in %h ADDR_BUS_OUT %h, FULL_MEM_ADDR %h FULL_PC_ADDR %h data_bus_in %h data_bus_temp %h data_bus %h MEM_INST_OUTP %h, MEM_ENABLE: %b MEM_ADDR %h MUX ENABLE:%b MEM R/W%b DRIVING_DBUS %b IR_REG%h\n",clk,c.addr_bus_in,c.addr_bus_out,c.FULL_MEM_ADDR,c.FULL_PC_ADDR,c.data_bus_in,c.data_bus_temp,c.data_bus,c.mem_outp,c.MEM_ENABLE,c.mem.addr,c.DATA_MUX_ENABLE,c.mem.R_W,c.DRIVING_d_bus,c.INST_REG.data);



endmodule