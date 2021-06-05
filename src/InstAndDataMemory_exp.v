`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Class: Fundamentals of Digital Logic and Processor
// Designer: Shulin Zeng
// 
// Create Date: 2021/04/30
// Design Name: MultiCycleCPU
// Module Name: InstAndDataMemory
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


module InstAndDataMemory_exp(reset, clk, Address, Write_data, MemRead, MemWrite, Mem_data);
	//Input Clock Signals
	input reset;
	input clk;
	//Input Data Signals
	input [31:0] Address;
	input [31:0] Write_data;
	//Input Control Signals
	input MemRead;
	input MemWrite;
	//Output Data
	output [31:0] Mem_data;
	
	parameter RAM_SIZE = 256;
	parameter RAM_SIZE_BIT = 8;
	parameter RAM_INST_SIZE = 32;
	
	reg [31:0] RAM_data[RAM_SIZE - 1: 0];

	//read data
	assign Mem_data = MemRead? RAM_data[Address[RAM_SIZE_BIT + 1:2]]: 32'h00000000;
	
	//write data
	integer i;
	always @(posedge reset or posedge clk) begin
		if (reset) begin
		    // init instruction memory
            // lui $a0 0x7fff
            RAM_data[8'd0] <= {6'h0f,5'd0,5'd4,16'h7fff};
            // addi $a0 $a0 0x1234
            RAM_data[8'd1] <= {6'h08,5'd4,5'd4,16'h1234};
			// lui $a1 0x7fff
            RAM_data[8'd2] <= {6'h0f,5'd0,5'd5,16'h7fff};
            // addi $a1 $a1 0x1234
            RAM_data[8'd3] <= {6'h08,5'd5,5'd5,16'h1234};
			// add $a0 $a0 $a1
			RAM_data[8'd4] <= {6'h0,5'd4,5'd5,5'd4,5'h0,6'h20};
            // addi $a2 $0 5
            RAM_data[8'd5] <= {6'h08,5'd0,5'd6,16'd5};
            // addi $a2 $a2 4
            RAM_data[8'd6] <= {6'h08,5'd6,5'd6,16'd4};
            // Loop:
            // beq $zero $zero Loop
            RAM_data[8'd7] <= {6'h04,5'd0,5'd0,16'hffff};

            RAM_data[8'd31] <= {6'h05,26'h0};
            //init instruction memory
            //reset data memory		  
			for (i = RAM_INST_SIZE ; i < RAM_SIZE; i = i + 1)
				RAM_data[i] <= 32'h00000000;
		end else if (MemWrite) begin
			RAM_data[Address[RAM_SIZE_BIT + 1:2]] <= Write_data;
		end
	end

endmodule