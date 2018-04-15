`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   18:33:13 03/31/2017
// Design Name:   PC_Branch
// Module Name:   C:/Xilinx/Tutorial/Program-Counter/TB_PC_Branch.v
// Project Name:  Program-Counter
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: PC_Branch
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module TB_PC_Branch;

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
	wire ff;
	wire g;
	wire dp;
	wire [3:0] an;

	// Instantiate the Unit Under Test (UUT)
	PC_Branch uut (
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
		.ff(ff), 
		.g(g), 
		.dp(dp), 
		.an(an)
	);

	
	initial begin
		  clock = 0; btns = 0; btnu = 0; btnd = 0; btnr = 0; btnl=0;    
	#15000 clock = 1; btns = 0; btnu = 0; btnd = 0; btnr = 0; btnl=0; 
	
	#25000 clock = 0; btns = 1; btnu = 1; btnd = 0; btnr = 0; btnl=0; 
	#25000 clock = 1; btns = 1; btnu = 1; btnd = 0; btnr = 0; btnl=0;
	
	/*#15000 btns = 1; btnu = 1; btnd = 0; btnr = 0; btnl=0; 
	#15000 btns = 1; btnu = 1; btnd = 0; btnr = 0; btnl=0;
	
	#15000 btns = 1; btnu = 0; btnd = 1; btnr = 0; btnl=0;
	#15000 btns = 1; btnu = 0; btnd = 1; btnr = 0; btnl=0;
	
	#15000 btns = 1; btnu = 0; btnd = 1; btnr = 0; btnl=0;
	#15000 btns = 1; btnu = 0; btnd = 1; btnr = 0; btnl=0;
	
	#15000 btns = 1; btnu = 0; btnd = 0; btnr = 1; btnl=0; new_count = 7;		
	#15000 btns = 1; btnu = 0; btnd = 0; btnr = 1; btnl=0; new_count = 7; 	

	#15000 btns = 1; btnu = 0; btnd = 0; btnr = 0; btnl=1; new_count = 5; 		
	#15000 btns = 1; btnu = 0; btnd = 0; btnr = 0; btnl=1; new_count = 5;
 	
	#15000 btns = 1; btnu = 0; btnd = 0; btnr = 0; btnl=0; new_count = 3; 	
	#15000 btns = 1; btnu = 0; btnd = 0; btnr = 0; btnl=0; new_count = 3;*/

	end
      
		always begin
		#10 clock =~clock;
		end
      
endmodule

