`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Class: Fundamentals of Digital Logic and Processor
// Designer: Shulin Zeng
//
// Create Date: 2021/04/30
// Design Name: MultiCycleCPU
// Module Name: Controller
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


module Controller(reset, clk, OpCode, Funct,
                  PCWrite, PCWriteCond, IorD, MemWrite, MemRead,
                  IRWrite, MemtoReg, RegDst, RegWrite, ExtOp, LuiOp,
                  ALUSrcA, ALUSrcB, ALUOp, PCSource);
//Input Clock Signals
input reset;
input clk;
//Input Signals
input  [5:0] OpCode;
input  [5:0] Funct;
//Output Control Signals
output reg PCWrite;
output reg PCWriteCond;
output reg IorD;
output reg MemWrite;
output reg MemRead;
output reg IRWrite;
output reg [1:0] MemtoReg;
output reg [1:0] RegDst;
output reg RegWrite;
output reg ExtOp;
output reg LuiOp;
output reg [1:0] ALUSrcA;
output reg [1:0] ALUSrcB;
output reg [3:0] ALUOp;
output reg [1:0] PCSource;

//--------------Your code below-----------------------
parameter [2:0]sIF = 3'b000;
parameter [2:0]sID = 3'b001;
parameter [2:0]EX = 3'b010;
parameter [2:0]MEM = 3'b011;
parameter [2:0]WB = 3'b100;

parameter [5:0]lw = 6'h23;
parameter [5:0]sw = 6'h2b;
parameter [5:0]lui = 6'h0f;
parameter [5:0]R = 6'h0;
parameter [5:0]J = 6'h02;
parameter [5:0]beq = 6'h04;
parameter [5:0]addi = 6'h08;
parameter [5:0]addiu = 6'h09;
parameter [5:0]andi = 6'h0c;
parameter [5:0]slti = 6'h0a;
parameter [5:0]sltiu = 6'h0b;
parameter [5:0]jal = 6'h03;

reg [2:0]state;
reg [2:0]state_next;

always @(posedge reset or posedge clk)
  begin
    if(reset)
      state <= sIF;
    else
      state <= state_next;
  end

always @(state)
  begin
    case (state)
      sIF:
        begin
          state_next <= sID;
          MemRead <= 1'b1;
          IRWrite <= 1'b1;
          PCWrite <= 1'b1;
          PCSource <= 2'b00;
          ALUSrcA <= 2'b00;
          ALUSrcB <= 2'b01;
          IorD <= 1'b0;
          state_next <= sID;

          PCWriteCond <= 1'b0;
          RegWrite <= 1'b0;
          MemWrite <= 1'b0;
        end
      sID:
        begin
          ALUSrcA <= 2'b00;
          ALUSrcB <= 2'b11;
          state_next <= EX;

          IRWrite <= 1'b0;
          MemRead <= 1'b0;
          PCWrite <= 1'b0;
        end
      EX:
        begin
          case (OpCode)
            J,jal:
              begin
                PCWrite <= 1'b1;
                PCSource <= 2'b10;
                state_next <= sIF;
                if(OpCode == jal)
                  begin
                    RegDst <= 2'b10;
                    MemtoReg <= 2'b10;
                    RegWrite <= 1'b1;
                  end
              end
            beq:
              begin
                PCWriteCond <= 1'b1;
                ALUSrcA <= 2'b01;
                ALUSrcB <= 2'b00;
                PCSource <= 2'b01;
                state_next <= sIF;
              end
            R:
              begin
                if((Funct == 6'h0) || (Funct == 6'h02) || (Funct == 6'h03))
                  ALUSrcA <= 2'b10;
                else
                  ALUSrcA <= 2'b01;
                ALUSrcB <= 2'b00;
                if(Funct == 6'h8)
                  begin
                    state_next <= sIF;
                    PCWrite = 1'b1;
                    PCSource = 2'b00;
                  end
                else
                  state_next <= WB;
              end
            lw,sw:
              begin
                ALUSrcA <= 2'b01;
                ALUSrcB <= 2'b10;
                state_next <= MEM;
              end
            addi,addiu,andi,slti,sltiu,lui:
              begin
                ALUSrcA <= 2'b01;
                ALUSrcB <= 2'b10;
                ExtOp <= 1'b1;
                if(OpCode == lui)
                  LuiOp <= 1'b1;
                else
                  LuiOp <= 1'b0;
                state_next <= WB;
              end
            default:
              state_next <= sIF;
          endcase
        end
      MEM:
        begin
          case (OpCode)
            sw:
              begin
                MemWrite <= 1'b1;
                IorD <= 1'b1;
                state_next <= sIF;
              end
            lw:
              begin
                MemRead <= 1'b1;
                IorD <= 1'b1;
                state_next <= WB;
              end
            default:
              state_next <= sIF;
          endcase
        end
      WB:
        begin
          RegWrite <= 1'b1;
          state_next <= sIF;
          case (OpCode)
            R:
              begin
                RegDst <= 2'b01;
                MemtoReg <= 2'b00;
              end
            lw:
              begin
                RegDst <= 2'b00;
                MemtoReg <= 2'b01;
              end
            addi,addiu,andi,slti,sltiu,lui:
              begin
                RegDst <= 2'b00;
                MemtoReg <= 2'b00;
              end
            default:
              state_next <= sIF;
          endcase
        end
      default:
        state_next <= sIF;
    endcase
  end

//--------------Your code above-----------------------


//ALUOp
always @(*)
  begin
    ALUOp[3] = OpCode[0];
    if (state == sIF || state == sID)
      begin
        ALUOp[2:0] = 3'b000;
      end
    else if (OpCode == 6'h00)
      begin
        ALUOp[2:0] = 3'b010;
      end
    else if (OpCode == 6'h04)
      begin
        ALUOp[2:0] = 3'b001;
      end
    else if (OpCode == 6'h0c)
      begin
        ALUOp[2:0] = 3'b100;
      end
    else if (OpCode == 6'h0a || OpCode == 6'h0b)
      begin
        ALUOp[2:0] = 3'b101;
      end
    else
      begin
        ALUOp[2:0] = 3'b000;
      end
  end

endmodule
