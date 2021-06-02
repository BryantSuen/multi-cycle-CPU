`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Class: Fundamentals of Digital Logic and Processor
// Designer: Shulin Zeng
//
// Create Date: 2021/04/30
// Design Name: MultiCycleCPU
// Module Name: MultiCycleCPU
// Project Name: Multi-cycle-cpu
// Target Devices:
// Tool Versions:
// Description:
//
// Dependencies:
//
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
//
//////////////////////////////////////////////////////////////////////////////////

module MultiCycleCPU (reset, clk);
//Input Clock Signals
input reset;
input clk;

//--------------Your code below-----------------------

//PC
wire [31:0]PC_cur;
wire [31:0]PC_next;
wire PC_write_en;

wire [31:0]PC_0;
wire [31:0]PC_1;
wire [31:0]PC_2;

assign PC_0 = Result;
assign PC_1 = ALU_out;
assign PC_2 = {PC_cur[31:28],rs,rt,rd,Shamt,Funct,2'b00};

assign PC_next = (PCSource == 2'b01)?PC_1:(PCSource == 2'b10)?PC_2:PC_0;
assign PC_write_en = (PCWriteCond && Zero) || PCWrite;
PC pc(.reset(reset),.clk(clk),.PCWrite(PC_write_en),.PC_i(PC_next),.PC_o(PC_cur));

//InstAndDataMemory
wire [31:0]Mem_data;
wire [31:0]Mem_addr;
assign Mem_addr = (IorD == 1)?ALU_out:PC_cur;
InstAndDataMemory IDM(.reset(reset),.clk(clk),.Address(Mem_addr),
                      .Write_data(B_out),.MemRead(MemRead),
                      .MemWrite(MemWrite),.Mem_data(Mem_data));

//InstReg
wire [4:0]rs;
wire [4:0]rt;
wire [4:0]rd;
wire [4:0]Shamt;
wire [5:0]Funct;
wire [5:0]OpCode;
InstReg IR(.reset(reset),.clk(clk),.IRWrite(IRWrite),.Instruction(Mem_data),
           .OpCode(OpCode),.rs(rs),.rt(rt),.rd(rd),
           .Shamt(Shamt),.Funct(Funct));

//MDR
wire [31:0]MDR_data;
RegTemp MDR(.reset(reset),.clk(clk),.Data_i(Mem_data),.Data_o(MDR_data));

//register_A
wire [31:0]A_out;
RegTemp Reg_A(.reset(reset),.clk(clk),.Data_i(Read_data1),.Data_o(A_out));
//register_B
wire [31:0]B_out;
RegTemp Reg_B(.reset(reset),.clk(clk),.Data_i(Read_data2),.Data_o(B_out));
//ALUOut
wire [31:0]ALU_out;
RegTemp ALU_out_reg(.reset(reset),.clk(clk),.Data_i(Result),.Data_o(ALU_out));


//RF
wire [4:0]Write_register;
wire [31:0] Read_data1;
wire [31:0] Read_data2;
wire [31:0] Write_data;
assign Write_register = (RegDst == 2'b01)?rd:(RegDst == 2'b10)?5'b11111:rt;
assign Write_data = (MemtoReg == 2'b01)?Mem_data:(MemtoReg == 2'b00)?ALU_out:PC_next;
RegisterFile RF(.reset(reset),.clk(clk),.RegWrite(RegWrite),
                .Read_register1(rs),.Read_register2(rt),
                .Write_register(Write_register),.Write_data(Write_data),
                .Read_data1(Read_data1),.Read_data2(Read_data2));

//controller
wire PCWrite;
wire PCWriteCond;
wire IorD;
wire MemWrite;
wire MemRead;
wire IRWrite;
wire [1:0]MemtoReg;
wire [1:0]RegDst;
wire [1:0]RegWrite;
wire ExtOp;
wire LuiOp;
wire [1:0] ALUSrcA;
wire [1:0] ALUSrcB;
wire [3:0] ALUOp;
wire [1:0] PCSource;
Controller controller(.reset(reset),.clk(clk),.OpCode(OpCode),
                      .Funct(Funct),.PCWrite(PCWrite),.PCWriteCond(PCWriteCond),
                      .IorD(IorD),.MemWrite(MemWrite),.MemRead(MemRead),
                      .IRWrite(IRWrite),.MemtoReg(MemtoReg),.RegDst(RegDst),
                      .RegWrite(RegWrite),.ExtOp(ExtOp),.LuiOp(LuiOp),.ALUSrcA(ALUSrcA),
                      .ALUSrcB(ALUSrcB),.ALUOp(ALUOp),.PCSource(PCSource));

//ALUControl
wire [4:0]ALUConf;
wire Sign;
ALUControl ALU_control(.ALUOp(ALUOp),.Funct(Funct),.ALUConf(ALUConf),.Sign(Sign));

//ALU
wire [31:0]In1;
wire [31:0]In2;

assign In1 = (ALUSrcA == 2'b00)?PC_cur:(ALUSrcA == 2'b10)?Shamt:A_out;
assign In2 = (ALUSrcB == 2'b01)?4:(ALUSrcB == 2'b10)?ImmExtOut:(ALUSrcB == 2'b11)?ImmExtShift:B_out;
wire Zero;
wire [31:0]Result;
ALU alu(.ALUConf(ALUConf),.Sign(Sign),.In1(In1),.In2(In2),
        .Zero(Zero),.Result(Result));

//ImmProcess
wire [31:0]ImmExtOut;
wire [31:0]ImmExtShift;
ImmProcess immprocess(.ExtOp(ExtOp),.LuiOp(LuiOp),.Immediate({rd,Shamt,Funct}),
                      .ImmExtOut(ImmExtOut),.ImmExtShift(ImmExtShift));

//--------------Your code above-----------------------

endmodule
