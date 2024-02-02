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
OR  - 0001 00

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

wire [15:0] addr_bus;

wire [7:0] data_bus; //Acts as intermediary to support bi-directionality of data bus.


//addr_bus a_bus(.in(addr_bus_in),.enable(1'b1),.out(addr_bus_out));

//Memory
reg R_W_reg;
reg MEM_ENABLE;
wire [7:0] mem_outp;
wire [7:0] PC_high_outp;
wire [7:0] PC_low_outp;
wire [7:0] INST_REG_outp;
wire [7:0] ACC_REG_outp; 
wire [7:0] X_REG_outp;
wire [7:0] Y_REG_outp;
wire [7:0] FL_REG_outp;
wire [7:0] ST_REG_outp;
memory mem(.clk(clk),.R_W(R_W),.addr(addr_bus),.data(data_bus),.outp(mem_outp),.enable(MEM_ENABLE));


//All registers
register PC_high(.clk(clk),.in(data_bus),.out_enable(PC_high_out_enable),.in_enable(PC_high_in_enable),.outp(PC_high_outp));
register PC_low(.clk(clk),.in(data_bus),.out_enable(PC_low_out_enable),.in_enable(PC_low_in_enable),.outp(PC_low_outp));


register INST_REG(.clk(clk),.in(data_bus),.out_enable(IR_out_enable),.in_enable(IR_in_enable),.outp(INST_REG_outp));
register ACC_REG(.clk(clk),.in(data_bus),.out_enable(A_out_enable),.in_enable(A_in_enable),.outp(ACC_REG_outp));
register X_REG(.clk(clk),.in(data_bus),.out_enable(X_out_enable),.in_enable(X_in_enable),.outp(X_REG_outp));
register Y_REG(.clk(clk),.in(data_bus),.out_enable(Y_out_enable),.in_enable(Y_in_enable),.outp(Y_REG_outp));
register FL_REG(.clk(clk),.in(data_bus),.out_enable(FL_out_enable),.in_enable(FL_in_enable),.outp(FL_REG_outp));
register ST_REG(.clk(clk),.in(data_bus),.out_enable(ST_out_enable),.in_enable(ST_in_enable),.outp(ST_REG_outp));


reg DATA_MUX_ENABLE;
reg ADDR_MUX_ENABLE;
reg [NUM_DATA_BUS_SELECTIONS-1:0]DATA_SELECT_MUX;
reg ADDRESS_SELECT_MUX;

reg [7:0] CUSTOM_VALUE_DATA_BUS; //put specific value on the data bus. Same as "data_out" of the CPU.

parameter 
NUM_DATA_BUS_SELECTIONS = 10, SELECT_A_DATA_BUS = 4'b0000, SELECT_X_DATA_BUS = 4'b0001, SELECT_Y_DATA_BUS = 4'b0010, SELECT_FL_DATA_BUS = 4'b0011, SELECT_IR_DATA_BUS = 4'b0100, SELECT_PC_LOW_DATA_BUS = 4'b0101, SELECT_PC_HIGH_DATA_BUS = 4'b0110, SELECT_MEM_DATA_BUS = 4'b0111, SELECT_CUSTOM_VAL_DATA_BUS = 4'b1000;


assign data_bus = DATA_MUX_ENABLE ?  (DATA_SELECT_MUX == SELECT_A_DATA_BUS ? ACC_REG_outp :
DATA_SELECT_MUX == SELECT_X_DATA_BUS  ? X_REG_outp : DATA_SELECT_MUX == SELECT_Y_DATA_BUS ? Y_REG_outp : 
DATA_SELECT_MUX == SELECT_FL_DATA_BUS ? FL_REG_outp : DATA_SELECT_MUX == SELECT_IR_DATA_BUS ? INST_REG_outp : 
DATA_SELECT_MUX == SELECT_PC_HIGH_DATA_BUS ? PC_high_outp : DATA_SELECT_MUX == SELECT_PC_LOW_DATA_BUS ? PC_low_outp : 
DATA_SELECT_MUX == SELECT_MEM_DATA_BUS ? mem_outp : DATA_SELECT_MUX == SELECT_CUSTOM_VAL_DATA_BUS ? CUSTOM_VALUE_DATA_BUS : 8'bz) : 8'bz; 
//Leave bus unconnected if nothing driving to it. 

//Long statement that acts as a multiplexer into the bus. 


parameter //FETCH STATES
    FETCH_INIT = 4'b0000, FETCH_READ = 4'b0001, FETCH_READ_LOW_PCBYTE = 4'b0101, FETCH_LOAD_IR = 4'b0010, FETCH_INCR_IR = 4'b0011, FETCH_OUTP_HIGH_PCBYTE = 4'b0100,FETCH_OUTP_LOW_PCBYTE = 4'b0101;
parameter //Memory states
    READ = 1'b1, WRITE = 1'b0;


reg [4:0] curr_state; 
reg [1:0] full_state; //Used for full state of fetch-decode-execute. 00 is fetch, 01 is decode, 10 is execute.

parameter
    FETCH = 2'b00, DECODE = 2'b01, EXECUTE = 2'b10;

initial begin
curr_state <= FETCH_INIT;
full_state <= FETCH;
end

parameter IS_IMMEDIATE_ADDR_MODE = 2'b00, IS_IMPLIED_ADDR_MODE = 2'b01, IS_REGISTER_ADDR_MODE = 2'b10, IS_MEMORY_ADDR_MODE = 2'b11;

reg [5:0] OPCODE; //is first 6 bits of instruction register for operation
reg [1:0] ADDR_MODE; //is last 2 bits of instruction register for opertaion
reg [7:0] OPERAND; //next byte that follows opcode + addr_mode pair. Unless is implied addressing mode in which it is skipped. 
reg [1:0] ALU_OP;

reg [3:0] REG_INDEX;

reg [1:0] HIGH_LOW_MEM_ADDR_STATE; //awful name, but is used to determine whether the current byte is for the high-byte of memory address in memory-addressing mode or low byte.


reg [7:0] HIGH_MEM_ADDR_IMMEDIATE;
initial HIGH_LOW_MEM_ADDR_STATE = 1'b1; //set to do high-byte of mem addr first
reg [7:0] LOW_MEM_ADDR_IMMEDIATE;

reg [7:0] IMMEDIATE_DATA;

reg [15:0] FULL_PC_ADDR;
reg [15:0] FULL_MEM_ADDR;


assign addr_bus = FULL_MEM_ADDR;


always @ (*) begin // things that should happen continuously based on certain internal states (fetch-decode-execute)
    if(full_state == FETCH) begin
        if(PC_high_out_enable && DATA_SELECT_MUX == SELECT_PC_HIGH_DATA_BUS) begin 
            FULL_PC_ADDR[15:8] <= data_bus; 
            end
        if(PC_low_out_enable && DATA_SELECT_MUX == SELECT_PC_LOW_DATA_BUS) begin FULL_PC_ADDR[7:0] <= data_bus; end

        case(curr_state)
            FETCH_INCR_IR:
            //grab op-codes and operands
            case(OP_OR_OPERAND)
            IS_OPERATION: begin
                OPCODE <= data_bus[7:2];
                ADDR_MODE <= data_bus[1:0]; 
            end
            IS_OPERAND: begin //grab specific data depending on the current addressing mode. 
                    case(ADDR_MODE)
                    IS_IMMEDIATE_ADDR_MODE: begin
                        IMMEDIATE_DATA <= data_bus;
                    end
                    IS_IMPLIED_ADDR_MODE: begin //maybe do something ? right now don't do anything
                    
                    end
                    IS_REGISTER_ADDR_MODE: begin
                        REG_INDEX <= data_bus[2:0]; //lower 3 bits indexes which register. 
                    end
                    IS_MEMORY_ADDR_MODE: begin
                        if(HIGH_LOW_MEM_ADDR_STATE) begin
                            //HIGH_LOW_MEM_ADDR_STATE <= ~HIGH_LOW_MEM_ADDR_STATE;
                            HIGH_MEM_ADDR_IMMEDIATE <= data_bus;
                        end
                        else begin
                            //HIGH_LOW_MEM_ADDR_STATE <= ~HIGH_LOW_MEM_ADDR_STATE;
                            LOW_MEM_ADDR_IMMEDIATE <= data_bus;
                        end

                end
                endcase

        end
        endcase
        FETCH_OUTP_HIGH_PCBYTE: begin//check if extra clock cycle needed, no extra clock cycle neede for imm addr mode
            case(ADDR_MODE)
                IS_MEMORY_ADDR_MODE: begin //next 2 bytes are operands. 
                    OP_OR_OPERAND <= IS_OPERAND;
                end
                IS_IMMEDIATE_ADDR_MODE: begin //next byte is operand.
                    OP_OR_OPERAND <= IS_OPERAND;
                end
                IS_IMPLIED_ADDR_MODE: begin //next byte is new instruction. 
                    OP_OR_OPERAND <= IS_OPERATION;
                end
                IS_REGISTER_ADDR_MODE: begin //next byte is operand.
                    OP_OR_OPERAND <= IS_OPERAND;
                end
            endcase
                
        end
        endcase

    end
    if(full_state == DECODE) begin

    end
end

always @ (posedge clk) begin
    case(curr_state)

    FETCH_INIT: begin
        PC_high_in_enable <= 1'b0;
        PC_low_in_enable <= 1'b0;
        DATA_MUX_ENABLE <= 1'b1;
        DATA_SELECT_MUX <= SELECT_PC_HIGH_DATA_BUS;
        PC_high_out_enable <= 1'b1;
        curr_state <= FETCH_READ;
    end

    FETCH_READ: begin //Here value goes out on the data bus yet FULL_PC_ADDR[15:8] doesn't update yet for some reason.
        PC_low_out_enable <= 1'b1;
        DATA_SELECT_MUX <= SELECT_PC_LOW_DATA_BUS;
        PC_high_out_enable <= 1'b0;
        curr_state <= FETCH_READ_LOW_PCBYTE;
    end
    FETCH_READ_LOW_PCBYTE: begin
        FULL_MEM_ADDR <= FULL_PC_ADDR;
        MEM_ENABLE <= 1'b1;
        R_W_reg <= READ;
        DATA_SELECT_MUX <= SELECT_MEM_DATA_BUS;
        PC_low_out_enable <= 1'b0; 
        curr_state <= FETCH_INCR_IR;

    end
    FETCH_INCR_IR: begin
        IR_in_enable <= 1'b1;
        FULL_PC_ADDR <= FULL_PC_ADDR + 1;
        CUSTOM_VALUE_DATA_BUS <= FULL_PC_ADDR[15:8]; 
        DATA_SELECT_MUX <= SELECT_CUSTOM_VAL_DATA_BUS;
        curr_state <= FETCH_OUTP_HIGH_PCBYTE;
    end
    FETCH_OUTP_HIGH_PCBYTE: begin
        IR_in_enable <= 1'b0;
        PC_high_in_enable <= 1'b1;
        CUSTOM_VALUE_DATA_BUS <= FULL_PC_ADDR[7:0];
        curr_state <= FETCH_OUTP_LOW_PCBYTE;
    end
    FETCH_OUTP_LOW_PCBYTE: begin
        PC_low_in_enable <= 1'b1;
        curr_state <= FETCH_INIT;
        //full_state <= DECODE;
    end
    endcase

end
//DECODE 

parameter DECODE_INIT = 4'b0110;
parameter IS_OPERATION = 1'b0, IS_OPERAND = 1'b1; 

reg OP_OR_OPERAND;//How to interpret value currently in IR. Is either an op-code or an operand. 

initial OP_OR_OPERAND = IS_OPERATION; //first byte ever should be interpreted as an op-code w/ addressing mode. 


always @ (posedge clk) begin
    case(curr_state)

    DECODE_INIT: begin
        IR_in_enable <= 1'b0;

    end

    endcase

end


assign R_W = R_W_reg; 

assign data_in = R_W == READ ? data_bus : 8'bz;
assign data_out = R_W == WRITE ? data_bus : 8'bz;
assign address = addr_bus; 

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
    $monitor("Clock: %b ADDR_BUS %h, FULL_MEM_ADDR %h FULL_PC_ADDR %h data_bus %h MEM_INST_OUTP %h, MEM_ENABLE: %b MUX ENABLE:%b MEM R/W%b IR_REG%h IR_REG_OUTP %h IR_IN_ENABLE: %b DATA_SELECT_MUX%h\n",clk,c.addr_bus,c.FULL_MEM_ADDR,c.FULL_PC_ADDR,c.data_bus,c.mem_outp,c.MEM_ENABLE,c.DATA_MUX_ENABLE,c.mem.R_W,c.INST_REG_outp,c.INST_REG.outp,c.IR_in_enable,c.DATA_SELECT_MUX);
end


initial begin
#36;
$finish;
end
always #1 clk = ~clk;
always #1 $monitor("Clock: %b ADDR_BUS %h, ADDR_MODE %h FULL_MEM_ADDR %h FULL_PC_ADDR %h data_bus %h MEM_INST_OUTP %h, REG_INDEX: %h OPCODE:%h MEM R/W%b IR_REG%h HIGH_MEM_ADDR_IMM %h LOW_MEM_ADDR_IMM %h IR_IN_ENABLE: %b IMM_DATA%h,HIGH_LOW_MEM_ADDR_STATE%b\n",clk,c.addr_bus,c.ADDR_MODE,c.FULL_MEM_ADDR,c.FULL_PC_ADDR,c.data_bus,c.mem_outp,c.REG_INDEX,c.OPCODE,c.mem.R_W,c.INST_REG.data,c.HIGH_MEM_ADDR_IMMEDIATE,c.LOW_MEM_ADDR_IMMEDIATE,c.IR_in_enable,c.IMMEDIATE_DATA,c.HIGH_LOW_MEM_ADDR_STATE);



endmodule