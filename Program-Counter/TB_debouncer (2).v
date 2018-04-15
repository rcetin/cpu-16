`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   20:31:49 03/29/2017
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
	reg clock;
	reg PB;
	reg PB_state;

	// Outputs
	wire clean;

	// Instantiate the Unit Under Test (UUT)
	debouncer uut (
		.clock(clock),
		.PB(PB), 
		.PB_state(PB_state)
	);

	initial begin
		// Initialize Inputs
		clock = 0;
		PB = 0;
		// Wait 100 ns for global reset to finish
		
        
		// Add stimulus here

	end
      always 
			#10 clock = ~clock;    // every ten nanoseconds invert the clock
always 
		begin
			#40000 PB = 1'b1;
			
			#400 PB = 1'b0;		
			
			#800 PB = 1'b1;	
			
			#800 PB = 1'b0;				
			
			#800 PB = 1'b1;

			#40000 PB = 1'b0;
			
			#4000 PB = 1'b1;		
			
			#40000 PB = 1'b0;

			#400 PB = 1'b1;
			
			#800 PB = 1'b0;		
			
			#800 PB = 1'b1;

			#800 PB = 1'b0;
			
			#40000 PB = 1'b1;		
			
			#4000 PB = 1'b0;

		end

endmodule

