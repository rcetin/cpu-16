`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   21:10:09 03/28/2017
// Design Name:   seven_segment_led
// Module Name:   C:/Xilinx/Tutorial/Program-Counter/tb_seven_segment_led.v
// Project Name:  Program-Counter
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: seven_segment_led
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb_seven_segment_led;

	// Inputs
	reg clock;
	reg reset;
	reg  [3:0]in0;
	reg  [3:0]in1;
	reg  [3:0]in2;
	reg  [3:0]in3;

	// Outputs
	wire a;
	wire b;
	wire c;
	wire d;
	wire e;
	wire f;
	wire g;
	wire dp;
	wire [3:0] an;

	// Instantiate the Unit Under Test (UUT)
	seven_segment_led uut (
		.clock(clock), 
		.reset(reset), 
		.in0(in0), 
		.in1(in1), 
		.in2(in2), 
		.in3(in3), 
		.a(a), 
		.b(b), 
		.c(c), 
		.d(d), 
		.e(e), 
		.f(f), 
		.g(g), 
		.dp(dp), 
		.an(an)
	);

	initial begin
		// Initialize Inputs
		clock = 0;
		reset = 1;
		in0 = 15;
		in1 = 0;
		in2 = 0;
		in3 = 0;
		
		
		#10 clock =1;
		#10 clock =0;reset =0;
		#10 clock =1; 
end
		// Wait 100 ns for global reset to finish
		always 
			#10 clock = ~clock;    // every ten nanoseconds invert the clock

		// Add stimulus here

	
      
endmodule

