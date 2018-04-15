`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   21:12:32 04/21/2017
// Design Name:   Sign_Ex_Test
// Module Name:   C:/Xilinx/Tutorial/Data_Cu_Mem/TB_Sign_Ex_Test.v
// Project Name:  Data_Cu_Mem
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: Sign_Ex_Test
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module TB_Sign_Ex_Test;

	// Inputs
	reg [3:0] immLow;
	reg [3:0] immHigh;

	// Outputs
	wire [15:0] imm_sign_extended;
wire [15:0] left_shifted_value;
	wire [15:0] zero_extended_value;
	// Instantiate the Unit Under Test (UUT)
	Sign_Ex_Test uut (
		.immLow(immLow), 
		.immHigh(immHigh), 
		.imm_sign_extended(imm_sign_extended),
		.left_shifted_value(left_shifted_value),
		.zero_extended_value(zero_extended_value)
	);

	initial begin
		// Initialize Inputs
		
				immHigh = 4; immLow = 1;
		  #20  immHigh = 15; immLow = 6;
		  #20 immHigh = 3; immLow = 7;

		

	end
      
endmodule

