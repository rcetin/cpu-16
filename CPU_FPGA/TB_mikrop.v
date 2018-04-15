`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   15:22:34 05/01/2017
// Design Name:   mikrop
// Module Name:   C:/Xilinx/Tutorial/CPU_FPGA/TB_mikrop.v
// Project Name:  CPU_FPGA
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: mikrop
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module TB_mikrop;

	// Inputs
	reg reset;
	reg SW2;
	reg board_clk;

	// Outputs
	wire [15:0] result;
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
	mikrop uut (
		.reset(reset), 
		.SW2(SW2), 
		.board_clk(board_clk), 
		.result(result), 
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
	$monitor  ("%d ns:      board_clk=%1'b result=%16'b",$time,board_clk,result);
		     reset = 1; SW2 = 0; board_clk = 0;
		#100 reset = 0; SW2 = 1; board_clk = 1;
		#100 reset = 1; SW2 = 0; board_clk = 0;
		#100 reset = 1; SW2 = 1; board_clk = 1;
		#100 reset = 1; SW2 = 0; board_clk = 0;
		#100 reset = 1; SW2 = 1; board_clk = 1;
		#100 reset = 1; SW2 = 0; board_clk = 0;
		#100 reset = 1; SW2 = 1; board_clk = 1;
	#100 reset = 1; SW2 = 0; board_clk = 0;
		#100 reset = 1; SW2 = 1; board_clk = 1;
		#100 reset = 1; SW2 = 0; board_clk = 0;
		#100 reset = 1; SW2 = 1; board_clk = 1;
		/*
		// Initialize Inputs
		rst = 0;
		clk = 0;

		// Wait 100 ns for global reset to finish
		#100;
      #100 clk = 1; rst = 0; 
		
		#100 clk = 0; rst = 1; 
		#100 clk = 1;//load
		#100 clk = 0;
		#100 clk = 1;
		#100 clk = 0;
		#100 clk = 1;
	   
		#100 clk = 0;
      #100 clk = 1;
		#100 clk = 0;
		#100 clk = 1;
		#100 clk = 0;
		#100 clk = 1; 
	
		#100 clk = 0;
		#100 clk = 1;
		#100 clk = 0;
		#100 clk = 1; 
		
		#100 clk = 0;
		#100 clk = 1;
		#100 clk = 0;
		#100 clk = 1; 
		
		#100 clk = 0;
		#100 clk = 1;
		#100 clk = 0;
		#100 clk = 1; 
        
		// Add stimulus here*/

	end
      
endmodule