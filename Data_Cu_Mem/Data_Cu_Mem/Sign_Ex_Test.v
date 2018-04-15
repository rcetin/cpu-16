`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    21:08:36 04/21/2017 
// Design Name: 
// Module Name:    Sign_Ex_Test 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
 
    

module Sign_Ex_Test(immLow, immHigh, imm_sign_extended);

	
	
      
	output [15:0] imm_sign_extended; // 16-bit output
   input [3:0] immLow ;
	input [3:0] immHigh;
	
	wire [7:0]a;
	assign a = {immHigh,immLow};
	assign imm_sign_extended ={{8{a[7]}}, a};
	
endmodule