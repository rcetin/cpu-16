`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   21:02:00 04/22/2017
// Design Name:   CPU
// Module Name:   C:/Xilinx/Tutorial/Data_Cu_Mem/TB_CPU.v
// Project Name:  Data_Cu_Mem
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: CPU
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module TB_CPU;

	// Inputs
	reg [15:0] inst;
	reg rst;
	reg clk;
	// Outputs
	wire [15:0] result;

	// Instantiate the Unit Under Test (UUT)
	CPU uut (
		.result(result), 
		.inst(inst), 
		.rst(rst), 
		.clk(clk)
	);

	initial begin
		// Initialize Inputs
		$monitor  ("%d ns:   inst=%16'b   rst=%1'b   clk=%1'b result=%16'b",$time,inst,rst,clk,result);
		inst = 0;
		rst = 0;
		clk = 0;

		// Wait 100 ns for global reset to finish
		#100;
       #100 clk = 1; rst = 0; 
		
		#100 clk = 0; rst = 1; 
		#100 clk = 1;inst = 16'b1111000000001010;//load
		#100 clk = 0;
		#100 clk = 1;
		#100 clk = 0;
		#100 clk = 1;
	   
		#100 clk = 0; rst = 1; inst = 16'b1111000100001111;//load
      #100 clk = 1; rst = 1; inst = 16'b1111000100001111;//load
		#100 clk = 0;
		#100 clk = 1;
		#100 clk = 0;
		#100 clk = 1; 
		
		#100 clk = 0; rst = 1; inst = 16'b0000000001010001;	//sum
		 #100 clk = 1; rst = 1; inst = 16'b0000000001010001;
#100 clk = 0;
		#100 clk = 1;

/*
		 #100 clk = 0; rst = 1; inst = 16'b0101000001010001;	//sumi
		 #100 clk = 1; rst = 1; inst = 16'b0101000001010001;
		 #100 clk = 0;
		#100 clk = 1;
		 
		 #100 clk = 1; rst = 1; inst = 16'b1001000001010001;	//subi
		 #100 clk = 0; rst = 1; inst = 16'b1001000001010001;

		 #100 clk = 1; rst = 1; inst = 16'b1011000001010001;	//cmpi
		 #100 clk = 0; rst = 1; inst = 16'b1011000001010001;

		 #100 clk = 1; rst = 1; inst = 16'b0001000001010001;	//andi
		 #100 clk = 0; rst = 1; inst = 16'b0001000001010001;

		 #100 clk = 1; rst = 1; inst = 16'b0010000001010001;	//ori
		 #100 clk = 0; rst = 1; inst = 16'b0010000001010001;
		 
		 #100 clk = 0; rst = 1; inst = 16'b0011000001010001;	//xori
		 #100 clk = 1; rst = 1; inst = 16'b0011000001010001;

			 #100 clk = 1; rst = 1; inst = 16'b1101000001010001;	//movi
		 #100 clk = 0; rst = 1; inst = 16'b1101000001010001;

		#100 clk = 0;
		#100 clk = 1;*/ 
		//#100 clk = 0;
		//#100 clk = 1; 
	end
      
endmodule

