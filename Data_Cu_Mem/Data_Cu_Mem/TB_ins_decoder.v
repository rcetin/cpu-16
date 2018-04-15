`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   22:34:26 04/20/2017
// Design Name:   ins_decoder
// Module Name:   C:/Xilinx/Tutorial/Data_Cu_Mem/TB_ins_decoder.v
// Project Name:  Data_Cu_Mem
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: ins_decoder
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module TB_ins_decoder;

	// Inputs
	reg [15:0] instruction;

	// Outputs
	wire [3:0] alu_out;
	wire arith_mux;

	// Instantiate the Unit Under Test (UUT)
	ins_decoder uut (
		.alu_out(alu_out), 
		.arith_mux(arith_mux), 
		.instruction(instruction)
	);

	initial begin
		// Initialize Inputs
		   instruction = 16'b0000000001010001;
	#100	instruction = 16'b1001000000010010;

	#100 instruction = 16'b0000000010110111;
		
       

	end
      
endmodule

