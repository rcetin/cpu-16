`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   15:52:30 04/30/2017
// Design Name:   CPU
// Module Name:   C:/Users/asus/Desktop/Data_Cu_Mem/Data_Cu_Mem/TB_cpu_im.v
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

module TB_cpu_im;

	// Inputs
	reg rst;
	reg clk;

	// Outputs
	wire [15:0] result;

	// Instantiate the Unit Under Test (UUT)
	CPU uut (
		.result(result), 
		.rst(rst), 
		.clk(clk)
	);

	initial begin
	$monitor  ("%d ns:   rst=%1'b   clk=%1'b result=%16'b",$time,rst,clk,result);
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
        
		// Add stimulus here

	end
      
endmodule

