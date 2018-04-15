`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   20:46:35 03/25/2017
// Design Name:   clk_div
// Module Name:   C:/Xilinx/Tutorial/Program-Counter/tb_clk_div.v
// Project Name:  Program-Counter
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: clk_div
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb_clk_div;
reg clk,reset;
  wire clk_out;
 
     clk_div t1(clk,reset,clk_out);
        initial
          clk= 1'b0;
     always
        #5  clk=~clk; 
        initial
            begin
               #5 reset=1'b1;
               #10 reset=1'b0;
               #50000 $finish;
            end
 
        initial
               $monitor("clk=%b,reset=%b,clk_out=%b",clk,reset,clk_out);
 
        initial
            begin
              $dumpfile("tb_clk_div.vcd");
              $dumpvars(0,tb_clk_div);
             end
      
endmodule

