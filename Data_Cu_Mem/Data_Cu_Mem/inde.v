`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    20:02:08 04/21/2017 
// Design Name: 
// Module Name:    inde 
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

module inde(alu_out, wr_en, arith_mux, op, op_ex);
input [3:0] op;
input [3:0] op_ex;
output reg [2:0] alu_out;
output reg arith_mux;
output reg wr_en;

always @(*)
begin
	if(op == 4'b0000)
	begin
	arith_mux <= 1'b0;				//0->reg-reg, 1->reg-imm
	wr_en <= 1;
				case(op_ex)
				4'b0101:alu_out <= 4'b000;	//sum
				4'b1001:alu_out <= 4'b001;	//sub
				4'b1011:alu_out <= 4'b110;	//compare
				4'b0001:alu_out <= 4'b010;	//and
				4'b0010:alu_out <= 4'b011;	//or
				4'b0011:alu_out <= 4'b100;	//xor
				//4'b1101:alu_out <= 4'b0111;	//mov
				endcase
	end
/*
	else if(op == 4'b0100)
	begin
	arith_mux <= 1'b0;				//0->reg-reg, 1->reg-imm
				case(op_ex)
				//4'b0000:alu_out <= 4'b0011;	//LOAD
				//4'b0100:alu_out <= 4'b0110;	//STOR
				//4'b1100:alu_out <= 4'b1011;	//Jcond
				//4'b1000:alu_out <= 4'b1100;	//JAL
				endcase
	end

	*/
	
	
	
	else if(op == 4'b0101)
	begin
	wr_en <= 1;
	arith_mux <= 1'b1;				//0->reg-reg, 1->reg-imm
	alu_out <= 4'b000;	//sum
	end
	else if(op == 4'b1001)
	begin
	wr_en <= 1;
	arith_mux <= 1'b1;				//0->reg-reg, 1->reg-imm
	alu_out <= 4'b001;	//sub
	end
	else if(op == 4'b1011)
	begin
	wr_en <= 1;
	arith_mux <= 1'b1;				//0->reg-reg, 1->reg-imm
	alu_out <= 4'b110;	//compare
	end
	else if(op == 4'b0001)
	begin
	wr_en <= 1;
	arith_mux <= 1'b1;				//0->reg-reg, 1->reg-imm
	alu_out <= 4'b010;	//and
	end
	else if(op == 4'b0010)
	begin
	wr_en <= 1;
	arith_mux <= 1'b1;				//0->reg-reg, 1->reg-imm
	alu_out <= 4'b011;	//or
	end
	else if(op == 4'b0011)
	begin
	wr_en <= 1;
	arith_mux <= 1'b1;				//0->reg-reg, 1->reg-imm
	alu_out <= 4'b100;	//xor
	end
	/*else if(op == 4'b1101)
	begin
	arith_mux <= 1'b1;				//0->reg-reg, 1->reg-imm
	alu_out <= 4'b0111;	//mov
	end

	else if(op == 4'b1000) //LSH
	begin
	arith_mux <= 1'b0;				//0->reg-reg, 1->reg-imm
	if(op_ex==1000) begin
	alu_out <= 4'b0001;
	end
	end
	
	else if(op == 4'b1000)
	begin
	arith_mux <= 1'b1;				//0->reg-reg, 1->reg-imm
	alu_out <= 4'b0001;	//LSHI
	end
	else if(op == 4'b1111)
	begin
	arith_mux <= 1'b1;				//0->reg-reg, 1->reg-imm
	alu_out <= 4'b0010;	//LUI
	end
	else if(op == 4'b1100)
	begin
	arith_mux <= 1'b1;				//0->reg-reg, 1->reg-imm
	alu_out <= 4'b1001;	//BCOND
	end
*/

	
end

endmodule

//////////////7/sign extender


	
	module Sign_Ex_Test(immLow, immHigh, imm_sign_extended);

   output [15:0] imm_sign_extended; // 16-bit output
   input [3:0] immLow ;
	input [3:0] immHigh;
	
	wire [7:0]a;
	assign a = {immHigh,immLow};
	assign imm_sign_extended ={{8{a[7]}}, a};
	
endmodule

/////////////2x1 Multiplexer

module mux2x1(imm_sign_extended, Rsrc, sel, z);
    
    input imm_sign_extended, Rsrc, sel;
    output z;
    assign z = (sel==1'b1)?imm_sign_extended:Rsrc;
    
endmodule

//////////////////////ALU Module

module ALU(Y,C,V,N,Z,A,B,Op);
    output [15:0] Y;
    output C; //Carry flag
    output N; //Negative flag
    output V; //Overflow flag
    output Z; //Zero lag
    input [15:0] A;
    input [15:0] B;
    input [2:0] Op;
    wire  [15:0] 	 AS, And, Or, Xor, Not, Xnor;
    wire 	 s; 
    wire 	 Vas;
    wire 	 Cas;
   
   //wire LSH, LUI, LOAD, STOR, MOV, Bcond, Jcond, JAL, CMP;  
	// The operations
   carry_select_adder_subtractor addsub(AS, Cas, Vas, A, B, Op[0]);      // Op == 3'b000, 3'b001 The operation: 0 => Add, 1=>Subtract
   andop aluand(And, A, B);                                              // Op == 3'b010  	 
   orop aluor(Or, A, B);                                                 // Op == 3'b011	 
   xorop aluxor(Xor, A, B);                                              // Op == 3'b100	 
   notop alunot(Not, A);                                                 // Op == 3'b101	 
   xnorop aluxnor(Xnor, A, B);                                           // Op == 3'b110	 
	
   
multiplexer_8_1 muxy(Y, AS, AS, And, Or, Xor, Not, Xnor, 16'b0, Op); // Select the result.

//multiplexer_16_1 muxxx(Y,Xor,LSH,LUI,LOAD,AS,AS,STOR,MOV,And,Bcond,Or,Jcond,JAL,CMP,16'b0,16'b0,Op);

   nor(s, Op[1], Op[2]);   // s == 0 => a logical operation, otherwise and arithmetic operation.
   and(C, Cas, s);
   and(V, Vas, s);
   and(N, Y[15], s);       // Most significant bit is the sign bit in 2's complement.   
   zero z(Z, Y);           // All operations can set the Zero status bit.
endmodule // alu

module andop(Y, A, B);
   output [15:0] Y;  // Result.
   input [15:0]  A;  // Operand.
   input [15:0]  B;  // Operand.

   and(Y[0], A[0], B[0]);
   and(Y[1], A[1], B[1]);
   and(Y[2], A[2], B[2]);
   and(Y[3], A[3], B[3]);
   and(Y[4], A[4], B[4]);
   and(Y[5], A[5], B[5]);
   and(Y[6], A[6], B[6]);
   and(Y[7], A[7], B[7]);
   and(Y[8], A[8], B[8]);
   and(Y[9], A[9], B[9]);
   and(Y[10], A[10], B[10]);
   and(Y[11], A[11], B[11]);
   and(Y[12], A[12], B[12]);
   and(Y[13], A[13], B[13]);
   and(Y[14], A[14], B[14]);
   and(Y[15], A[15], B[15]);
endmodule // andop

module orop(Y, A, B);
   output [15:0] Y; // Result.
   input [15:0]  A; // Operand.
   input [15:0]  B; // Operand.

   or(Y[0], A[0], B[0]);
   or(Y[1], A[1], B[1]);
   or(Y[2], A[2], B[2]);
   or(Y[3], A[3], B[3]);
   or(Y[4], A[4], B[4]);
   or(Y[5], A[5], B[5]);
   or(Y[6], A[6], B[6]);
   or(Y[7], A[7], B[7]);
   or(Y[8], A[8], B[8]);
   or(Y[9], A[9], B[9]);
   or(Y[10], A[10], B[10]);
   or(Y[11], A[11], B[11]);
   or(Y[12], A[12], B[12]);
   or(Y[13], A[13], B[13]);
   or(Y[14], A[14], B[14]);
   or(Y[15], A[15], B[15]);
endmodule // orop

module xorop(Y, A, B);
   output [15:0] Y; // Result.
   input [15:0]  A; // Operand.
   input [15:0]  B; // Operand.

   xor(Y[0], A[0], B[0]);
   xor(Y[1], A[1], B[1]);
   xor(Y[2], A[2], B[2]);
   xor(Y[3], A[3], B[3]);
   xor(Y[4], A[4], B[4]);
   xor(Y[5], A[5], B[5]);
   xor(Y[6], A[6], B[6]);
   xor(Y[7], A[7], B[7]);
   xor(Y[8], A[8], B[8]);
   xor(Y[9], A[9], B[9]);
   xor(Y[10], A[10], B[10]);
   xor(Y[11], A[11], B[11]);
   xor(Y[12], A[12], B[12]);
   xor(Y[13], A[13], B[13]);
   xor(Y[14], A[14], B[14]);
   xor(Y[15], A[15], B[15]);
endmodule // xorop

module notop(Y, A);
   output [15:0] Y; // Result.
   input [15:0]  A; // Operand.

   not(Y[0], A[0]);
   not(Y[1], A[1]);
   not(Y[2], A[2]);
   not(Y[3], A[3]);
   not(Y[4], A[4]);
   not(Y[5], A[5]);
   not(Y[6], A[6]);
   not(Y[7], A[7]);
   not(Y[8], A[8]);
   not(Y[9], A[9]);
   not(Y[10], A[10]);
   not(Y[11], A[11]);
   not(Y[12], A[12]);
   not(Y[13], A[13]);
   not(Y[14], A[14]);
   not(Y[15], A[15]);
endmodule // notop

module xnorop(Y, A, B);
   output [15:0] Y; // Result.
   input [15:0]  A; // Operand.
   input [15:0]  B; // Operand.

   xnor(Y[0], A[0], B[0]);
   xnor(Y[1], A[1], B[1]);
   xnor(Y[2], A[2], B[2]);
   xnor(Y[3], A[3], B[3]);
   xnor(Y[4], A[4], B[4]);
   xnor(Y[5], A[5], B[5]);
   xnor(Y[6], A[6], B[6]);
   xnor(Y[7], A[7], B[7]);
   xnor(Y[8], A[8], B[8]);
   xnor(Y[9], A[9], B[9]);
   xnor(Y[10], A[10], B[10]);
   xnor(Y[11], A[11], B[11]);
   xnor(Y[12], A[12], B[12]);
   xnor(Y[13], A[13], B[13]);
   xnor(Y[14], A[14], B[14]);
   xnor(Y[15], A[15], B[15]);
endmodule // xnorop

module zero(Z, A);
   output Z;        // Result. 
   input [15:0]  A; // Operand.

   wire [15:0] 	 Y; // Temp result.
   
   xnor(Y[0], A[0], 0);
   xnor(Y[1], A[1], 0);
   xnor(Y[2], A[2], 0);
   xnor(Y[3], A[3], 0);
   xnor(Y[4], A[4], 0);
   xnor(Y[5], A[5], 0);
   xnor(Y[6], A[6], 0);
   xnor(Y[7], A[7], 0);
   xnor(Y[8], A[8], 0);
   xnor(Y[9], A[9], 0);
   xnor(Y[10], A[10], 0);
   xnor(Y[11], A[11], 0);
   xnor(Y[12], A[12], 0);
   xnor(Y[13], A[13], 0);
   xnor(Y[14], A[14], 0);
   xnor(Y[15], A[15], 0);
   and(Z, Y[0], Y[1], Y[2], Y[3], Y[4],
       Y[5], Y[6], Y[7], Y[8],
       Y[9], Y[10], Y[11], Y[12],
       Y[13], Y[14], Y[15]);
endmodule // zero
      
module carry_select_adder_subtractor(S, C, V, A, B, Op);
   output [15:0] S;   // The 16-bit sum/difference.
   output 	 C;   // The 1-bit carry/borrow status.
   output 	 V;   // The 1-bit overflow status.
   input [15:0]  A;   // The 16-bit augend/minuend.
   input [15:0]  B;   // The 16-bit addend/subtrahend.
   input 	 Op;  // The operation: 0 => Add, 1=>Subtract.
   
   wire 	 C15; // The carry out bit of adder/subtractor, used to generate final carry/borrrow.   
   wire [15:0] 	 Bx;
   
   // Looking at the truth table for not we see that  
   // B xor 0 = B, and
   // B xor 1 = not(B).
   // So, if Op==1 means we are subtracting, then
   // adding A and B xor Op alog with setting the first
   // carry bit to Op, will give us a result of
   // A+B when Op==0, and A+not(B)+1 when Op==1.
   // Note that not(B)+1 is the 2's complement of B, so
   // this gives us subtraction.     
   
   xor(Bx[0], B[0], Op);
   xor(Bx[1], B[1], Op);
   xor(Bx[2], B[2], Op);
   xor(Bx[3], B[3], Op);
   xor(Bx[4], B[4], Op);
   xor(Bx[5], B[5], Op);
   xor(Bx[6], B[6], Op);
   xor(Bx[7], B[7], Op);
   xor(Bx[8], B[8], Op);
   xor(Bx[9], B[9], Op);
   xor(Bx[10], B[10], Op);
   xor(Bx[11], B[11], Op);
   xor(Bx[12], B[12], Op);
   xor(Bx[13], B[13], Op);
   xor(Bx[14], B[14], Op);
   xor(Bx[15], B[15], Op);
   xor(C, C15, Op);            // Carry = C15 for addition, Carry = not(C15) for subtraction.
   carry_select_adder csa(S, C15, V, A, Bx, Op);   
endmodule // carry_select_adder_subtractor

module carry_select_adder(S, C, V, A, B, Cin);
   output [15:0] S;   // The 16-bit sum.
   output 	 C;   // The 1-bit carry.
   output 	 V;   // The 1-bit overflow status.
   input [15:0]  A;   // The 16-bit augend.
   input [15:0]  B;   // The 16-bit addend.
   input 	 Cin; // The initial carry in.

   wire [3:0] 	S1_0;   // Nibble 1 sum output with carry input 0.
   wire [3:0] 	S1_1;   // Nibble 1 sum output with carry input 1.
   wire [3:0] 	S2_0;   // Nibble 2 sum output with carry input 0.
   wire [3:0] 	S2_1;   // Nibble 2 sum output with carry input 1.
   wire [3:0] 	S3_0;   // Nibble 3 sum output with carry input 0.
   wire [3:0] 	S3_1;   // Nibble 3 sum output with carry input 1.
   wire 	C1_0;   // Nibble 1 carry output with carry input 0.
   wire 	C1_1;   // Nibble 1 carry output with carry input 1.
   wire 	C2_0;   // Nibble 2 carry output with carry input 0.
   wire 	C2_1;   // Nibble 2 carry output with carry input 1.
   wire 	C3_0;   // Nibble 3 carry output with carry input 0.
   wire 	C3_1;   // Nibble 3 carry output with carry input 1.
   wire 	C0;     // Nibble 0 carry output used to select multiplexer output.
   wire 	C1;     // Nibble 1 carry output used to select multiplexer output.
   wire 	C2;     // Nibble 2 carry output used to select multiplexer output.
   wire         V0;     // Nibble 0 overflow output.
   wire 	V1_0;   // Nibble 1 overflow output with carry input 0.
   wire 	V1_1;   // Nibble 1 overflow output with carry input 1.
   wire 	V2_0;   // Nibble 2 overflow output with carry input 0.
   wire 	V2_1;   // Nibble 2 overflow output with carry input 1.
   wire 	V3_0;   // Nibble 3 overflow output with carry input 0.
   wire 	V3_1;   // Nibble 3 overflow output with carry input 1.
   
   ripple_carry_adder rc_nibble_0(S[3:0], C0, V0, A[3:0], B[3:0], Cin);              // Calculate S nibble 0.
   ripple_carry_adder rc_nibble_1_carry_0(S1_0, C1_0, V1_0, A[7:4], B[7:4], 0);      // Calculate S nibble 1 with carry input 0.
   ripple_carry_adder rc_nibble_1_carry_1(S1_1, C1_1, V1_1, A[7:4], B[7:4], 1);      // Calculate S nibble 1 with carry input 1.
   ripple_carry_adder rc_nibble_2_carry_0(S2_0, C2_0, V2_0, A[11:8], B[11:8], 0);    // Calculate S nibble 2 with carry input 0.
   ripple_carry_adder rc_nibble_2_carry_1(S2_1, C2_1, V2_1, A[11:8], B[11:8], 1);    // Calculate S nibble 2 with carry input 1.
   ripple_carry_adder rc_nibble_3_carry_0(S3_0, C3_0, V3_0, A[15:12], B[15:12], 0);  // Calculate S nibble 3 with carry input 0.
   ripple_carry_adder rc_nibble_3_carry_1(S3_1, C3_1, V3_1, A[15:12], B[15:12], 1);  // Calculate S nibble 3 with carry input 1.

   multiplexer_2_1 #(1) muxc1(C1, C1_0, C1_1, C0); // C0 selects the carry output for nibble 1.
   multiplexer_2_1 #(1) muxc2(C2, C2_0, C2_1, C1); // C1 selects the carry output for nibble 2.
   multiplexer_2_1 #(1) muxc(C, C3_0, C3_1, C2);   // C2 selects the carry output for nibble 3 which is the global carry output.
   multiplexer_2_1 #(1) muxv(V, V3_0, V3_1, C2);   // C2 selects the overflow output for nibble 3 which is the global overflow output.
   
   multiplexer_2_1 #(4) muxs1(S[7:4], S1_0, S1_1, C0);    // C0 selects the result for nibble 1.
   multiplexer_2_1 #(4) muxs2(S[11:8], S2_0, S2_1, C1);   // C1 selects the result for nibble 2.
   multiplexer_2_1 #(4) muxs3(S[15:12], S3_0, S3_1, C2);  // C2 selects the result for nibble 3.
endmodule // carry_select_adder

module ripple_carry_adder(S, C, V, A, B, Cin);
   output [3:0] S;   // The 4-bit sum.
   output 	C;   // The 1-bit carry.
   output       V;   // The 1-bit overflow status.   
   input [3:0] 	A;   // The 4-bit augend.
   input [3:0] 	B;   // The 4-bit addend.
   input 	Cin; // The carry input.
 	
   wire 	C0; // The carry out bit of fa0, the carry in bit of fa1.
   wire 	C1; // The carry out bit of fa1, the carry in bit of fa2.
   wire 	C2; // The carry out bit of fa2, the carry in bit of fa3.
	
   full_adder fa0(S[0], C0, A[0], B[0], Cin);    // Least significant bit.
   full_adder fa1(S[1], C1, A[1], B[1], C0);
   full_adder fa2(S[2], C2, A[2], B[2], C1);
   full_adder fa3(S[3], C, A[3], B[3], C2);    // Most significant bit.
   xor(V, C, C2);  // Overflow   
endmodule // ripple_carry_adder

module full_adder(S, Cout, A, B, Cin);
   output S;
   output Cout;
   input  A;
   input  B;
   input  Cin;
   
   wire   w1;
   wire   w2;
   wire   w3;
   wire   w4;
   
   xor(w1, A, B);
   xor(S, Cin, w1);
   and(w2, A, B);   
   and(w3, A, Cin);
   and(w4, B, Cin);   
   or(Cout, w2, w3, w4);
endmodule // full_adder

module multiplexer_2_1(X, A0, A1, S);
   parameter WIDTH=16;     // How many bits wide are the lines

   output [WIDTH-1:0] X;   // The output line

   input [WIDTH-1:0]  A1;  // Input line with id 1'b1
   input [WIDTH-1:0]  A0;  // Input line with id 1'b0
   input 	      S;  // Selection bit
   
   assign X = (S == 1'b0) ? A0 : A1;
endmodule // multiplexer_2_1

/*module multiplexer_8_1(X, A0, A1, A2, A3, A4, A5, A6, A7, S);
   parameter WIDTH=16;     // How many bits wide are the lines

   output [WIDTH-1:0] X;   // The output line

   input [WIDTH-1:0]  A7;  // Input line with id 3'b111
   input [WIDTH-1:0]  A6;  // Input line with id 3'b110
   input [WIDTH-1:0]  A5;  // Input line with id 3'b101
   input [WIDTH-1:0]  A4;  // Input line with id 3'b100
   input [WIDTH-1:0]  A3;  // Input line with id 3'b011
   input [WIDTH-1:0]  A2;  // Input line with id 3'b010
   input [WIDTH-1:0]  A1;  // Input line with id 3'b001
   input [WIDTH-1:0]  A0;  // Input line with id 3'b000
   input [2:0]	      S;   
endmodule*/

/*   assign X = (S[2] == 0 
	       ? (S[1] == 0 
		  ? (S[0] == 0 
		     ? A0       // {S2,S1,S0} = 3'b000
		     : A1)      // {S2,S1,S0} = 3'b001
		  : (S[0] == 0 
		     ? A2       // {S2,S1,S0} = 3'b010
		     : A3))     // {S2,S1,S0} = 3'b011
	       : (S[1] == 0 
		  ? (S[0] == 0 
		     ? A4       // {S2,S1,S0} = 3'b100
		     : A5)      // {S2,S1,S0} = 3'b101
		  : (S[0] == 0 
		     ? A6       // {S2,S1,S0} = 3'b110
		     : A7)));   // {S2,S1,S0} = 3'b111

endmodule // multiplexer_8_1*/

/*module multiplexer_16_1 (X,A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15,S);
	input[3:0]S;
	parameter WIDTH=16;     // How many bits wide are the lines

   output [WIDTH-1:0] X;   // The output line
	reg [15:0] X;
   

   input [WIDTH-1:0]  A0;  
   input [WIDTH-1:0]  A1;  
   input [WIDTH-1:0]  A2;  
   input [WIDTH-1:0]  A3;   
   input [WIDTH-1:0]  A4;  
   input [WIDTH-1:0]  A5;  
   input [WIDTH-1:0]  A6;  
   input [WIDTH-1:0]  A7;  
   input [WIDTH-1:0]  A8;
   input [WIDTH-1:0]  A9;
   input [WIDTH-1:0]  A10;
   input [WIDTH-1:0]  A11;
   input [WIDTH-1:0]  A12;
   input [WIDTH-1:0]  A13;
   input [WIDTH-1:0]  A14;
   input [WIDTH-1:0]  A15;
	
	//multiplexer_16_1 muxxx(Y,Xor,LSH,LUI,LOAD,AS,AS,STOR,MOV,And,Bcond,Or,Jcond,JAL,CMP,16'b0,16'b0,Op);


   always@(S or A0 or A1 or A2 or A3 or A4 or A5 or A6 or A7 or A8 or A9 or A10 or A11 or A12 or A13 or A14 or A15)
	begin
   case(S)
	4'b0000: X=A0;   //Xor
	4'b0001: X=A1;   //LSH
	4'b0010: X=A2;   //LUI
	4'b0011: X=A3;   //LOAD
	4'b0100: X=A4;   //AS
	4'b0101: X=A5;   //AS
	4'b0110: X=A6;   //STOR
	4'b0111: X=A7;   //MOV
	4'b1000: X=A8;   //And
	4'b1001: X=A9;   //Bcond
	4'b1010: X=A10;  //Or
	4'b1011: X=A11;  //Jcond
	4'b1100: X=A12;  //JAL
	4'b1101: X=A13;  //CMP
	4'b1110: X=A14;  //16'b0
	4'b1111: X=A15;  //16'b0
endcase
end
endmodule
    
*/

module RegisterFile_16x16(Adat,Bdat,clk,Ra,Rb,Rw,wr_en,Wdat);
    output [15:0] Adat;
    output [15:0] Bdat;
    input clk;
    input [3:0] Ra;
    input [3:0] Rb;
    input [3:0] Rw;
    input wr_en;
    input [15:0] Wdat;
   

wire [15:0]demux_out;
wire [15:0]q0;
wire [15:0]q1;
wire [15:0]q2;
wire [15:0]q3;
wire [15:0]q4;
wire [15:0]q5;
wire [15:0]q6;
wire [15:0]q7;
wire [15:0]q8;
wire [15:0]q9;
wire [15:0]q10;
wire [15:0]q11;
wire [15:0]q12;
wire [15:0]q13;
wire [15:0]q14;
wire [15:0]q15;
wire [15:0]qinv0;
wire [15:0]qinv1;
wire [15:0]qinv2;
wire [15:0]qinv3;
wire [15:0]qinv4;
wire [15:0]qinv5;
wire [15:0]qinv6;
wire [15:0]qinv7;
wire [15:0]qinv8;
wire [15:0]qinv9;
wire [15:0]qinv10;
wire [15:0]qinv11;
wire [15:0]qinv12;
wire [15:0]qinv13;
wire [15:0]qinv14;
wire [15:0]qinv15;

DEMUX_1x16 demux1(demux_out, wr_en, Rw);

Register16 reg0(q0, qinv0, clk, Wdat, demux_out[0]);
Register16 reg1(q1, qinv1, clk, Wdat, demux_out[1]);
Register16 reg2(q2, qinv2, clk, Wdat, demux_out[2]);
Register16 reg3(q3, qinv3, clk, Wdat, demux_out[3]);
Register16 reg4(q4, qinv4, clk, Wdat, demux_out[4]);
Register16 reg5(q5, qinv5, clk, Wdat, demux_out[5]);
Register16 reg6(q6, qinv6, clk, Wdat, demux_out[6]);
Register16 reg7(q7, qinv7, clk, Wdat, demux_out[7]);
Register16 reg8(q8, qinv8, clk, Wdat, demux_out[8]);
Register16 reg9(q9, qinv9, clk, Wdat, demux_out[9]);
Register16 reg10(q10, qinv10, clk, Wdat, demux_out[10]);
Register16 reg11(q11, qinv11, clk, Wdat, demux_out[11]);
Register16 reg12(q12, qinv12, clk, Wdat, demux_out[12]);
Register16 reg13(q13, qinv13, clk, Wdat, demux_out[13]);
Register16 reg14(q14, qinv14, clk, Wdat, demux_out[14]);
Register16 reg15(q15, qinv15, clk, Wdat, demux_out[15]);


MUX16X1_32 muxA(Adat, Ra, q0, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12, q13, q14, q15 );
MUX16X1_32 muxB(Bdat,Rb,q0,q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,q11,q12,q13,q14,q15);

endmodule

// Modules needed to design Register File

// 1) Demultiplexer
module DEMUX_1x16(out, wr_en, RW);

    input wr_en;
    input [3:0] RW;
    output [15:0] out;

 assign out[0]=(wr_en & ~RW[3] & ~RW[2] & ~RW[1] & ~RW[0]),
			 out[1]=(wr_en & ~RW[3] & ~RW[2] & ~RW[1] & RW[0]),
			 out[2]=(wr_en & ~RW[3] & ~RW[2] & RW[1] & ~RW[0]),
			 out[3]=(wr_en & ~RW[3] & ~RW[2] & RW[1] & RW[0]),
			 out[4]=(wr_en & ~RW[3] & RW[2] & ~RW[1] & ~RW[0]),
			 out[5]=(wr_en & ~RW[3] & RW[2] & ~RW[1] & RW[0]),
			 out[6]=(wr_en & ~RW[3] & RW[2] & RW[1] & ~RW[0]),
			 out[7]=(wr_en & ~RW[3] & RW[2] & RW[1] & RW[0]),
			 out[8]=(wr_en & RW[3] & ~RW[2] & ~RW[1] & ~RW[0]),
			 out[9]=(wr_en & RW[3] & ~RW[2] & ~RW[1] & RW[0]),
			 out[10]=(wr_en & RW[3] & ~RW[2] & RW[1] & ~RW[0]),
			 out[11]=(wr_en & RW[3] & ~RW[2] & RW[1] & RW[0]),
			 out[12]=(wr_en & RW[3] & RW[2] & ~RW[1] & ~RW[0]),
			 out[13]=(wr_en & RW[3] & RW[2] & ~RW[1] & RW[0]),
			 out[14]=(wr_en & RW[3] & RW[2] & RW[1] & ~RW[0]),
			 out[15]=(wr_en & RW[3] & RW[2] & RW[1] & RW[0]);
endmodule

// 2) 16-bit Register

module Register16(q, qinv, clk, d, rst); 
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

// 3) D Flip Flop

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
	
// 4) NAND GATE WITH tplh=tphl=2ns

module My_NAND(out,in1,in2);
    input in1;
    input in2;
    output out;
    assign #(2,2) out=~(in1&in2);

endmodule

// 5) 16x1 Multiplexer

module MUX16X1(out,address,in);
    input [3:0] address;
    input [15:0] in;
    output out;
	 reg out;
    
	 
always@(address or in)
begin
case(address)
4'b0000:out=in[0];
4'b0001:out=in[1];
4'b0010:out=in[2];
4'b0011:out=in[3];
4'b0100:out=in[4];
4'b0101:out=in[5];
4'b0110:out=in[6];
4'b0111:out=in[7];
4'b1000:out=in[8];
4'b1001:out=in[9];
4'b1010:out=in[10];
4'b1011:out=in[11];
4'b1100:out=in[12];
4'b1101:out=in[13];
4'b1110:out=in[14];
4'b1111:out=in[15];
endcase
end
endmodule

// 5) 16x1 Multiplexer (for 16-bit inputs and 16-bit output)

module MUX16X1_32(out, address, input0, input1, input2, input3, input4, input5, input6, input7, input8, input9, 
input10, input11, input12, input13, input14, input15);
    output [15:0] out;
    input [3:0] address;
    input [15:0] input0,input1,input2,input3,input4,input5,input6;
    input [15:0] input7,input8,input9,input10,input11,input12;
    input [15:0] input13,input14,input15;
    
	 
  wire[15:0] MUX16X1[15:0];         // Create a 2D array of wires
  assign MUX16X1[0] = input0;       // Connect the sources of the array
  assign MUX16X1[1] = input1;
  assign MUX16X1[2] = input2;
  assign MUX16X1[3] = input3;
  assign MUX16X1[4] = input4;
  assign MUX16X1[5] = input5;
  assign MUX16X1[6] = input6;
  assign MUX16X1[7] = input7;
  assign MUX16X1[8] = input8;
  assign MUX16X1[9] = input9;
  assign MUX16X1[10] = input10;
  assign MUX16X1[11] = input11;
  assign MUX16X1[12] = input12;
  assign MUX16X1[13] = input13;
  assign MUX16X1[14] = input14;
  assign MUX16X1[15] = input15;
  assign out = MUX16X1[address];    // Connect the output of the array

endmodule


