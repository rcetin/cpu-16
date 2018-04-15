`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    21:22:44 04/20/2017 
// Design Name: 
// Module Name:    instruction_register 
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
module instruction_register(q, qinv, clk, d, rst);
 input rst,clk;
    input [15:0] d;
    output [15:0] q,qinv;
   
genvar i;
generate
  for(i=0; i<16; i=i+1) 
  begin : rega
    DFF dff1(q[i],qinv[i],clk,d[i],rst);
  end
endgenerate

endmodule

//////////////////////////////////////////////////////////////////////////////////
module DFF(q,qinv,clk,d,rst);
    input  clk;
	 input  d;
    input rst;
	 output q;
	 output qinv;
	 wire x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12;
    
	 My_NAND nand1(x3,x1,x2);
	 My_NAND nand2(x4,x3,clk);
	 My_NAND nand3(x5,x4,x4);
	 My_NAND nand4(x2,x5,rst);
	 My_NAND nand5(x6,x2,clk);
	 My_NAND nand6(x7,x6,x6);
	 My_NAND nand7(x8,x7,x1);
	 My_NAND nand8(x9,x8,d);
	 My_NAND nand9(x10,x9,x9);
	 My_NAND nand10(x1,x10,rst);
	 My_NAND nand11(q,x2,qinv);
	 My_NAND nand12(x11,q,x8);
	 My_NAND nand13(x12,x11,x11);
	 My_NAND nand14(qinv,x12,rst);
	 
	endmodule
//////////////////////////////////////////////////////////////////////

module My_NAND(out,in1,in2);
    input in1;
    input in2;
    output out;
    assign out=~(in1&in2);

endmodule

