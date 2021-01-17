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



/*
 *	Description:
 *
 *		This module implements the control and status registers (CSRs).
 */

`include "../include/rv32i-defines.v"
`include "../include/sail-core-defines.v"

`define CSR_RDCYCLEL_ADDR 12'hc00 //TODO move these to defines file
`define CSR_RDCYCLEH_ADDR 12'hc80

module csr_file (clk, rst_n_i, CSR_wr_en_i, CSR_addr_i, CSR_src_i, CSP_op_i, CSR_rd_o);
	input clk;
	input rst_n_i;
	input CSR_wr_en_i;
	input [11:0] CSR_addr_i;
	input [31:0] CSR_src_i;
	input [ 3:0] CSP_op_i;
	output reg[31:0] CSR_rd_o;

	reg [63:0] rdcycle_csr_n;
	reg [63:0] rdcycle_csr_d;
	reg [63:0] rdcycle_csr_q;

	reg [31:0] CSR_tmp;

	always @(*) begin
		rdcycle_csr_n = rdcycle_csr_q; //increment

		case (CSR_addr_i) //CSR select
			`CSR_RDCYCLEL_ADDR: begin //NOTE: rdcycle is a read-only csr, this deviation from spec just acts as a placeholder for writable CSRs
				CSR_rd_o = rdcycle_csr_q[31:0];
				rdcycle_csr_n[31:0] = CSR_tmp;
			end
			`CSR_RDCYCLEH_ADDR: begin
				CSR_rd_o = rdcycle_csr_q[63:32];
				rdcycle_csr_n[63:32] = CSR_tmp;
			end
			default: begin
				//do nothing
				CSR_rd_o = 32'b0;
			end
		endcase

		case (CSP_op_i) //TODO: could decouple CSR op from ALUCTL and use less bits
			/*
			 *	CSRRW  only
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRW:	CSR_tmp = CSR_src_i;

			/*
			 *	CSRRS only
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRS:	CSR_tmp = CSR_src_i | CSR_rd_o;

			/*
			 *	CSRRC only
			 */
			`kSAIL_MICROARCHITECTURE_ALUCTL_3to0_CSRRC:	CSR_tmp = (~CSR_src_i) & CSR_rd_o;

			default: CSR_tmp = CSR_src_i; //TODO: CSRRW could just be the default
		endcase

		rdcycle_csr_d = CSR_wr_en_i ? rdcycle_csr_n : rdcycle_csr_q + 64'b1;
	end

	always @(posedge clk, negedge rst_n_i) begin
		if (!rst_n_i) begin
			rdcycle_csr_q <= 64'b0;
		end
		else begin
			rdcycle_csr_q <= rdcycle_csr_d;
		end
	end

endmodule
