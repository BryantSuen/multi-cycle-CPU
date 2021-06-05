module ExceptionProcess (clk,exp_write,
                         PC,ALU_A,ALU_B,ALU_out,
                         OpCode,Funct,rt,rd,
                         epc,errtarget,flag);
input clk;
input exp_write;
input [31:0] PC;
input ALU_A;
input ALU_B;
input ALU_out;
input [5:0]Funct;
input [5:0]OpCode;
input [4:0]rt;
input [4:0]rd;

reg [31:0]epc_reg;
reg [4:0]errtarget_reg;
output [31:0]epc;
output [4:0]errtarget;
output flag;

assign epc = epc_reg;
assign errtarget = errtarget_reg;
assign flag = ((OpCode == 6'h0) && (Funct == 6'h20) && ((ALU_A && ALU_B && ~ALU_out) || (~ALU_A && ~ALU_B && ALU_out)))?1'b1:
       ((OpCode == 6'h0) && (Funct == 6'h22) && ((ALU_A && ~ALU_B && ~ALU_out) || (~ALU_A && ALU_B && ALU_out)))?1'b1:
       ((OpCode == 6'h08) && ((ALU_A && ALU_B && ~ALU_out) || (~ALU_A && ~ALU_B && ALU_out)))?1'b1:1'b0;

always @(posedge clk)
  begin
    if (flag && exp_write)
      begin
        epc_reg <= PC;
        errtarget_reg <= rd;
      end
  end



endmodule
