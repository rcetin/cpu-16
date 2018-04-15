`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   19:36:28 03/31/2017
// Design Name:   debouncer
// Module Name:   C:/Xilinx/Tutorial/Program-Counter/TB_debouncer.v
// Project Name:  Program-Counter
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: debouncer
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module TB_debouncer;

	// Inputs
	reg clk;
	reg PB;

	// Outputs
	wire PB_state;

	// Instantiate the Unit Under Test (UUT)
	debouncer uut (
		.clk(clk), 
		.PB(PB), 
		.PB_state(PB_state)
	);

	initial begin
		// Initialize Inputs
		clk = 0;
		PB = 0;

		// Wait 100 ns for global reset to finish
		#100;
        
		// Add stimulus here

	end
	 always 
      			#10 clk = ~clk;    // every ten nanoseconds invert the clock
always 
		begin
			#400 PB = 1'b1;
			
			
		end
endmodule

