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

//---PC
reg [31:0]PC_cur;
wire [31:0]PC_next;
wire PCWrite;    //TODO:PCWrite

assign PC_next = PC_cur + 32'd4;
PC pc(.reset(reset),.clk(clk),.PCWrite(PCWrite),.PC_i(PC_next),.PC_o(PC_cur));
//---end PC

//---InstAndDataMemory
wire [31:0]Mem_data;
InstAndDataMemory IDM(.reset(reset),.clk(clk),.Address(PC_cur),
                      .Write_data(Read_data2),.MemRead(MemRead),
                      .MemWrite(MemWrite),.Mem_data(Mem_data));
//---end InstAndDataMemory

//---InstReg
wire [4:0]rs;
wire [4:0]rt;
wire [4:0]rd;
wire [4:0]Shamt;
wire [4:0]Funct;
InstReg IR(.reset(reset),.clk(clk),.IRWrite(IRWrite),.Instruction(Mem_data),
           .OpCode(Opcode),.rs(rs),.rt(rt),.rd(rd),
           .Shamt(Shamt),.Funct(Funct));
//---end InstReg

//---MDR
wire [31:0]Data_o;
RegTemp MDR(.reset(reset),.clk(clk),.Data_i(Mem_data),.Data_o(Data_o));
//---end MDR

//---RF
wire RegWrite;
wire [4:0]Write_register;
wire [31:0] Read_data1;
wire [31:0] Read_data2;
wire [31:0] Write_data;
assign Write_register = (RegDst == 1)?rd:rt;
assign Write_data = (MemtoReg == 1)?Data_o:ALU_out;
RegisterFile RF(.reset(reset),.clk(clk),.RegWrite(RegWrite),
                .Read_register1(rs),.Read_register2(rt),
                .Write_register(Write_register),.Write_data(Write_data),
                .Read_data1(Read_data1),.Read_data2(Read_data2));
//---end RF

//---controller
wire [5:0]Opcode;
wire [5:0]Funct;
wire PCWriteCond;
wire IorD;
wire MemWrite;
wire MemRead;
wire IRWrite;
wire MemtoReg;
wire [1:0]RegDst;
wire ExtOp;
wire LuiOp;
wire [1:0] ALUSrcA;
wire [1:0] ALUSrcB;
wire [3:0] ALUOp;
wire [1:0] PCSource;
Controller controller(.reset(reset),.clk(clk),.Opcode(Opcode),
                      .Funct(Funct),.PCWrite(PCWrite),.PCWriteCond(PCWriteCond),
                      .IorD(IorD),.MemWrite(MemWrite),.MemRead(MemRead),
                      .IRWrite(IRWrite),.MemtoReg(MemtoReg),.RegDst(RegDst),
                      .RegWrite(RegWrite),.ExtOp(ExtOp),.LuiOp(LuiOp),.ALUSrcA(ALUSrcA),
                      .ALUSrcB(ALUSrcB),.ALUOp(ALUOp),.PCSource(PCSource));
//---end controller

//---ALUControl
wire [4:0]ALUConf;
wire Sign;
ALUControl ALU_control(.ALUOp(ALUOp),.Funct(Funct),.ALUConf(ALUConf),.Sign(Sign));
//---end ALUControl

//---ALU
wire [31:0]In1;
wire [31:0]In2;
wire Zero;
wire [31:0]Result;
ALU alu(.ALUConf(ALUConf),.Sign(Sign),.In1(In1),.In2(In2),
        .Zero(Zero),.Result(Result));
//---end ALU

//---ImmProcess
wire [31:0]ImmExtOut;
wire [31:0]ImmExtShift;
ImmProcess immprocess(.ExtOp(ExtOp),.LuiOp(LuiOp),.Immediate({rd,Shamt,Funct}),
                      .ImmExtOut(ImmExtOut),.ImmExtShift(ImmExtShift));
//---end ImmProcess

// ...

//--------------Your code above-----------------------

endmodule
