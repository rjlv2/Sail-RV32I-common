/*
	Authored 2018-2019, Ryan Voo.

	All rights reserved.
	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/



`include "../include/rv32i-defines.v"
`include "../include/sail-core-defines.v"



/*
 *	Description:
 *
 *		This module implements the ALU for the RV32I.
 */



/*
 *	Not all instructions are fed to the ALU. As a result, the ALUctl
 *	field is only unique across the instructions that are actually
 *	fed to the ALU.
 */
module alu(ALUctl, A, B, sign, ALUOut, Branch_Enable);
	input [6:0]		ALUctl;
	input [31:0]		A;
	input [31:0]		B;
	input 			sign;
	output reg [31:0]	ALUOut;
	output reg		Branch_Enable;

	reg [31:0] alu_and_result;
	reg [31:0] alu_or_result;
	reg [31:0] alu_add_result;
	reg [31:0] alu_sub_result;
	reg [31:0] alu_slt_result;
	reg [31:0] alu_srl_result;
	reg [31:0] alu_sra_result;
	reg [31:0] alu_sll_result;
	reg [31:0] alu_xor_result;

	assign alu_and_result = A & B;
	assign alu_or_result = A | B;
	assign alu_add_result = A + B;
	assign alu_sub_result = A - B; //TODO: sub is just add but using the 2's complement of B
	assign alu_slt_result = $signed({A[31]&sign, A}) < $signed({B[31]&sign, B}) ? 32'b1 : 32'b0;
	assign alu_srl_result = A >> B[4:0];
	assign alu_sra_result = A >>> B[4:0];
	assign alu_sll_result = A << B[4:0];
	assign alu_xor_result = A ^ B;

	always @(ALUctl, A, B) begin
		case (ALUctl[3:0])
			/*
			 *	AND (the fields also match ANDI and LUI)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_AND:	ALUOut = alu_and_result;

			/*
			 *	OR (the fields also match ORI)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_OR:	ALUOut = alu_or_result;

			/*
			 *	ADD (the fields also match AUIPC, all loads, all stores, and ADDI)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_ADD:	ALUOut = alu_add_result;

			/*
			 *	SUBTRACT (the fields also matches all branches)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SUB:	ALUOut = alu_sub_result;

			/*
			 *	SLT (the fields also matches all the other SLT variants)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SLT:	ALUOut = alu_slt_result;

			/*
			 *	SRL (the fields also matches the other SRL variants)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SRL:	ALUOut = alu_srl_result;

			/*
			 *	SRA (the fields also matches the other SRA variants)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SRA:	ALUOut = alu_sra_result;

			/*
			 *	SLL (the fields also match the other SLL variants)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_SLL:	ALUOut = alu_sll_result;

			/*
			 *	XOR (the fields also match other XOR variants)
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_XOR:	ALUOut = alu_xor_result;

			/*
			 *	Should never happen.
			 */
			default:					ALUOut = 0;
		endcase
	end

	always @(*) begin
		case (ALUctl[6:4])
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BEQ:	Branch_Enable = (alu_sub_result == 0);
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BNE:	Branch_Enable = !(alu_sub_result == 0);
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLT:	Branch_Enable = ($signed(A) < $signed(B)); //TODO: could just reuse SLT result
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGE:	Branch_Enable = ($signed(A) >= $signed(B));
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BLTU:	Branch_Enable = ($unsigned(A) < $unsigned(B));
			`kSAIL_MICROARCHITECTURE_ALUCTL_6to4_BGEU:	Branch_Enable = ($unsigned(A) >= $unsigned(B));

			default:					Branch_Enable = 1'b0;
		endcase
	end
endmodule
