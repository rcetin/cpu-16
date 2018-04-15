`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   21:31:33 04/20/2017
// Design Name:   instruction_register
// Module Name:   C:/Xilinx/Tutorial/Data_Cu_Mem/TB_instruction_register.v
// Project Name:  Data_Cu_Mem
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: instruction_register
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module TB_instruction_register;

	// Inputs
	reg clk;
	reg [15:0] d;
	reg rst;

	// Outputs
	wire [15:0] q;
	wire [15:0] qinv;

	// Instantiate the Unit Under Test (UUT)
	instruction_register uut (
		.q(q), 
		.qinv(qinv), 
		.clk(clk), 
		.d(d), 
		.rst(rst)
	);

	initial begin
		// Initialize Inputs
		clk = 0;
		d = 0;
		rst = 0;

		// Wait 100 ns for global reset to finish
		#100 clk = 1; rst =0; d =16;
		#100 clk = 0; rst =1; d =16;
      #100 clk = 1; rst =1; d =16;
		#100 clk = 0; rst =1; d =10;
		#100 clk = 1; rst =1; d =10;
		#100 clk = 0; rst =1; d =10;
		#100 clk = 1; rst =1; d =10;
		#100 clk = 0; rst =1; d =10;
		// Add stimulus here

	end
      
endmodule

