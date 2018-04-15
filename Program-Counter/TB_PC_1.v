`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   11:26:32 03/30/2017
// Design Name:   PC_1
// Module Name:   C:/Xilinx/Tutorial/Program-Counter/TB_PC_1.v
// Project Name:  Program-Counter
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: PC_1
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module TB_PC_1;

	// Inputs
	reg clock;
	reg btns;
	reg btnu;
	reg btnd;
	reg btnr;
	reg btnl;
	reg [7:0] new_count;

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
	PC_1 uut (
		.clock(clock), 
		.btns(btns), 
		.btnu(btnu), 
		.btnd(btnd), 
		.btnr(btnr), 
		.btnl(btnl), 
		.new_count(new_count), 
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
		 clock = 0; btns = 0; btnu = 0; btnd = 0; btnr = 0; btnl=0;    
	#50 clock = 1; btns = 0; btnu = 0; btnd = 0; btnr = 0; btnl=0; 
	
	#50 clock = 0; btns = 1; btnu = 0; btnd = 0; btnr = 1; btnl=0; new_count = 7;		
	#50 clock = 1; btns = 1; btnu = 0; btnd = 0; btnr = 1; btnl=0; new_count = 7; 	

	#50 clock = 0; btns = 1; btnu = 0; btnd = 0; btnr = 0; btnl=1; new_count = 5; 		
	#50 clock = 1; btns = 1; btnu = 0; btnd = 0; btnr = 0; btnl=1; new_count = 5;
 	
	#50 clock = 0; btns = 1; btnu = 0; btnd = 0; btnr = 0; btnl=0; new_count = 3; 	
	#50 clock = 1; btns = 1; btnu = 0; btnd = 0; btnr = 0; btnl=0; new_count = 3;
	
	#50 clock = 0; btns = 1; btnu = 1; btnd = 0; btnr = 0; btnl=0; 
	#50 clock = 1; btns = 1; btnu = 1; btnd = 0; btnr = 0; btnl=0;
	
	#50 clock = 0; btns = 1; btnu = 1; btnd = 0; btnr = 0; btnl=0; 
	#50 clock = 1; btns = 1; btnu = 1; btnd = 0; btnr = 0; btnl=0;
	
	#50 clock = 0; btns = 1; btnu = 0; btnd = 1; btnr = 0; btnl=0;
	#50 clock = 1; btns = 1; btnu = 0; btnd = 1; btnr = 0; btnl=0;
	
	#50 clock = 0; btns = 1; btnu = 0; btnd = 1; btnr = 0; btnl=0;
	#50 clock = 1; btns = 1; btnu = 0; btnd = 1; btnr = 0; btnl=0;
	
	end
      
endmodule

