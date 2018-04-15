`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    21:42:00 04/20/2017 
// Design Name: 
// Module Name:    ins_decoder 
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
module ins_decoder(alu_out, arith_mux, instruction);
input [15:0] instruction;
output reg [3:0] alu_out;
output reg arith_mux;

wire [3:0] op;
wire [3:0] rdest;
wire [3:0] op_ex ;
wire [3:0] rsrc ;
wire [3:0] immLow ;

assign op =  instruction[15:12];
assign rdest = instruction[11:8];
assign op_ex=  instruction[7:4];
assign rsrc =  instruction[3:0];
assign immLow= instruction[3:0];




always @(*)
begin
	if(op == 4'b0000)
	begin
	arith_mux <= 1'b0;				//0->reg-reg, 1->reg-imm
				case(op_ex)
				4'b0101:alu_out <= 4'b0100;	//sum
				4'b1001:alu_out <= 4'b0101;	//sub
				4'b1011:alu_out <= 4'b1111;	//compare
				4'b0001:alu_out <= 4'b1000;	//and
				4'b0010:alu_out <= 4'b1010;	//or
				4'b0011:alu_out <= 4'b0000;	//xor
				4'b1101:alu_out <= 4'b0111;	//mov
				endcase
				end
	else if(op == 4'b0101)
	begin
	arith_mux <= 1'b1;				//0->reg-reg, 1->reg-imm
	alu_out <= 4'b0100;	//sum
	end
	else if(op == 4'b1001)
	begin
	arith_mux <= 1'b1;				//0->reg-reg, 1->reg-imm
	alu_out <= 4'b0101;	//sub
	end
	else if(op == 4'b1011)
	begin
	arith_mux <= 1'b1;				//0->reg-reg, 1->reg-imm
	alu_out <= 4'b1111;	//compare
	end
	else if(op == 4'b0001)
	begin
	arith_mux <= 1'b1;				//0->reg-reg, 1->reg-imm
	alu_out <= 4'b1000;	//and
	end
	else if(op == 4'b0010)
	begin
	arith_mux <= 1'b1;	//0->reg-reg, 1->reg-imm
	alu_out <= 4'b1010;	//or
	end
	else if(op == 4'b0011)
	begin
	arith_mux <= 1'b1;				//0->reg-reg, 1->reg-imm
	alu_out <= 4'b0000;	//xor
	end
	else if(op == 4'b1101)
	begin
	arith_mux <= 1'b1;				//0->reg-reg, 1->reg-imm
	alu_out <= 4'b0111;	//mov
	end
end

endmodule
