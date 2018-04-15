`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    12:47:48 03/28/2017 
// Design Name: 
// Module Name:    PC_Branch 
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
    
	
	 
	 module PC_Branch(clock, btns, btnu, btnd, btnr, btnl, new_count, a,b,c,d,e,ff,g,dp,an,led0,led1,led2,led3,led4,led5,led6,led7);
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
	 
	 output reg led0;
	 output reg led1;
	 output reg led2;
	 output reg led3;
	 output reg led4;
	 output reg led5;
	 output reg led6;
	 output reg led7;
	 
	 reg [15:0] count; // Output address of the program counter
	 
	 wire [3:0] first = count[3:0];
	 wire [3:0] second = count[7:4];
	 wire [3:0] third = count[11:8];
	 wire [3:0] fourth = count[15:12];
	 
	 wire btns_o;
	 wire btnl_o;
	 wire btnr_o;
	 wire btnd_o;
	 wire btnu_o;
	 
	 debounce db1(clock, btns, btns_o);
	 debounce db2(clock, btnl, btnl_o);
	 debounce db3(clock, btnr, btnr_o);
	 debounce db4(clock, btnd, btnd_o);
	 debounce db5(clock, btnu, btnu_o);
	 
	 seven_segment_led ss(clock, 1'b0, first, second, third, fourth, a,b,c,d,e,ff,g,dp, an);
	
	reg count_flag;
	 
	   reg [7:0] leftval=8'b00000000;
		reg [7:0] rightval = 8'b00000000;
		wire [15:0] finalval;
		reg kont = 1'b0;
		wire [15:0] countbef;
		assign finalval = {leftval,rightval};
		
		
		wire clk_out;
		clk_div clkk(clock, 1'b0, clk_out);
		
	 // Clocked operation
	 always @(posedge clk_out) begin
	 
	 if(btns) begin // If the reset line is low, then zero the counter
	 count <= 16'b0000000000000000;
	 end
	 
	 else if(btnr_o) begin // If set is high, then load a new value into the counter
	 rightval <= new_count;
	 led0 <= new_count[0];
	 led1 <= new_count[1];
	 led2 <= new_count[2];
	 led3 <= new_count[3];
	 led4 <= new_count[4];
	 led5 <= new_count[5];
	 led6 <= new_count[6];
	 led7 <= new_count[7];
	 
	 end
	 
	 else if(btnl_o) begin // If set is high, then load a new value into the counter
	 leftval <= new_count;
	 led0 <= new_count[0];
	 led1 <= new_count[1];
	 led2 <= new_count[2];
	 led3 <= new_count[3];
	 led4 <= new_count[4];
	 led5 <= new_count[5];
	 led6 <= new_count[6];
	 led7 <= new_count[7];
	 kont = 1;
	 end
	 else if(kont) begin
	 
	 count <= finalval;
	 kont =0;
	 end
	 else if(btnu_o) begin // Otherwise, if increment is high, add one to the counter
	 count_flag<=1;
	 count <= count + 1;
	 end
	 
	 else if(btnd_o) begin
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
    
	 
	 

localparam N1 = 18;
 
reg [N1-1:0]count; //the 18 bit counter which allows us to multiplex at 1000Hz
 
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
  case(count[N1-1:N1-2]) //using only the 2 MSB's of the counter 
    
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


    
module debounce(clk,button_in,DB_out);
    input clk;
    //input n_reset,
    input button_in;
    output reg DB_out;
    
	 
	 //// ---------------- internal constants --------------
	parameter N = 11 ;		// (2^ (21-1) )/ 38 MHz = 32 ms debounce time
////---------------- internal variables ---------------
	reg  [N-1 : 0]	q_reg;							// timing regs
	reg  [N-1 : 0]	q_next;
	reg DFF1, DFF2;									// input flip-flops
	wire q_add;											// control flags
	wire q_reset;
//// ------------------------------------------------------

	reg n_reset = 0;
////contenious assignment for counter control
	assign q_reset = (DFF1  ^ DFF2);		// xor input flip flops to look for level chage to reset counter
	assign  q_add = ~(q_reg[N-1]);			// add to counter when q_reg msb is equal to 0
	
//// combo counter to manage q_next	
	always @ ( q_reset, q_add, q_reg)
		begin
			case( {q_reset , q_add})
				2'b00 :
						q_next <= q_reg;
				2'b01 :
						q_next <= q_reg + 1;
				default :
						q_next <= { N {1'b0} };
			endcase 	
		end
	
//// Flip flop inputs and q_reg update
	always @ ( posedge clk )
		begin
			if(n_reset ==  1'b0)
				begin
					DFF1 <= 1'b0;
					DFF2 <= 1'b0;
					q_reg <= { N {1'b0} };
					n_reset <=1;
				end
			else
				begin
					DFF1 <= button_in;
					DFF2 <= DFF1;
					q_reg <= q_next;
				end
		end
	
//// counter control
	always @ ( posedge clk )
		begin
			if(q_reg[N-1] == 1'b1)
					DB_out <= DFF2;
			else
					DB_out <= DB_out;
		end

	endmodule




