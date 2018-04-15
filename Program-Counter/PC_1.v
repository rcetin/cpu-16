`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    10:33:17 03/30/2017 
// Design Name: 
// Module Name:    PC_1 
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

    module PC_1(clock, btns, btnu, btnd, btnr, btnl, new_count, a,b,c,d,e,ff,g,dp,an );
	 input clock;
	 input btns; // Synchronous reset; active low
	 input btnu; // Only increment the counter when this signal is high
	 input btnd;
	 input btnr;
	 input btnl; // When this signal is high, the counter loads new_count into the counter
	 input [7:0] new_count; // New value to set the counter to
	 
	 output a;
	 output b;
	 output c;
	 output d;
	 output e;
	 output ff;
	 output g;
	 output dp;
    output [3:0] an;
	 
	 reg [15:0] count; // Output address of the program counter
    
	 wire [3:0] first = count[3:0];
	 wire [3:0] second = count[7:4];
	 wire [3:0] third = count[11:8];
	 wire [3:0] fourth = count[15:12];
	 
	 seven_segment_led ss(clock, btns, first, second, third, fourth, a,b,c,d,e,ff,g,dp, an);
	 
	 
	   reg count_flag;

	   reg [7:0] leftval=8'b00000000;
		reg [7:0] rightval = 8'b00000000;
		wire [15:0] finalval;
		reg kont = 1'b0;
		
	 assign finalval = {leftval,rightval};

	 // Clocked operation
	 always @(posedge clock) begin
	 if(~btns) begin // If the reset line is low, then zero the counter
	 count <= 0;
	 end
	 
	 else if(btnr) begin // If set is high, then load a new value into the counter
	 rightval <= new_count;
	 
	 end
	 
	 else if(btnl) begin // If set is high, then load a new value into the counter
	 leftval <= new_count;
	 kont = 1;
	 end
	 else if(kont) begin
	 
	 count <= finalval;
	 kont =0;
	 end
	 else if(btnu) begin // Otherwise, if increment is high, add one to the counter
	 count_flag<=1;
	 count <= count + 1;
	 end
	 
	 else if(btnd) begin
	 count_flag<=0;
	 end
	 
	 else if(count_flag) begin
	 count<=count+1;
	 end
	 
	 end // END always
    
	 endmodule
	 
	 module seven_segment_led(clock,reset,in0,in1,in2,in3,a,b,c,d,e,f,g,dp,an);
    input clock;
    input reset;
    input [3:0]in0;
	 input [3:0]in1;
	 input [3:0]in2;
	 input [3:0]in3;
    output a;
	 output b;
	 output c;
	 output d;
	 output e;
	 output f;
	 output g;
	 output dp;
    output [3:0] an;
    
	 
	 

localparam N = 18;
 
reg [N-1:0]count; //the 18 bit counter which allows us to multiplex at 1000Hz
 
always @ (posedge clock or posedge reset)
 begin
  if (reset)
   count <= 0;
  else
   count <= count + 1;
 end
 
reg [6:0]sseg; //the 7 bit register to hold the data to output
reg [3:0]an_temp; //register for the 4 bit enable
 
always @ (*)
 begin
  case(count[N-1:N-2]) //using only the 2 MSB's of the counter 
    
   2'b00 :  //When the 2 MSB's are 00 enable the fourth display
    begin
     sseg = in0;
     an_temp = 4'b1110;
    end
    
   2'b01:  //When the 2 MSB's are 01 enable the third display
    begin
     sseg = in1;
     an_temp = 4'b1101;
    end
    
   2'b10:  //When the 2 MSB's are 10 enable the second display
    begin
     sseg = in2;
     an_temp = 4'b1011;
    end
     
   2'b11:  //When the 2 MSB's are 11 enable the first display
    begin
     sseg = in3;
     an_temp = 4'b0111;
    end
  endcase
 end
assign an = an_temp;
 
 
reg [6:0] sseg_temp; // 7 bit register to hold the binary value of each input given
 
always @ (*)
 begin
  case(sseg)
   4'd0 : sseg_temp = 7'b1000000; //to display 0
   4'd1 : sseg_temp = 7'b1111001; //to display 1
   4'd2 : sseg_temp = 7'b0100100; //to display 2
   4'd3 : sseg_temp = 7'b0110000; //to display 3
   4'd4 : sseg_temp = 7'b0011001; //to display 4
   4'd5 : sseg_temp = 7'b0010010; //to display 5
   4'd6 : sseg_temp = 7'b0000010; //to display 6
   4'd7 : sseg_temp = 7'b1111000; //to display 7
   4'd8 : sseg_temp = 7'b0000000; //to display 8
   4'd9 : sseg_temp = 7'b0010000; //to display 9
	4'd10: sseg_temp = 7'b0001000;
	4'd11: sseg_temp = 7'b0000011;
	4'd12: sseg_temp = 7'b1000110;
	4'd13: sseg_temp = 7'b0100001;
	4'd14: sseg_temp = 7'b0000110;
	4'd15: sseg_temp = 7'b0001110;
   default : sseg_temp = 7'b0111111; //dash
  endcase
 end
assign {g, f, e, d, c, b, a} = sseg_temp; //concatenate the outputs to the register, this is just a more neat way of doing this.
// I could have done in the case statement: 4'd0 : {g, f, e, d, c, b, a} = 7'b1000000; 
// its the same thing.. write however you like it
 
assign dp = 1'b1; //since the decimal point is not needed, all 4 of them are turned off
 
 
endmodule



    

