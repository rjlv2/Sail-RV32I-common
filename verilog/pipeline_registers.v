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
 *	Pipeline registers
 */



/* IF/ID pipeline registers */ 
module if_id (clk, rst_n_i, data_mem_stall, data_in, data_out);
	input			clk;
	input           rst_n_i;
	input           data_mem_stall;
	input [63:0]		data_in;
	output reg[63:0]	data_out;

	always @(posedge clk, negedge rst_n_i) begin
		if(!rst_n_i) begin
			data_out <= 64'b0;
		end
		else begin
			data_out <= data_mem_stall ? data_out : data_in;
		end
	end
endmodule



/* ID/EX pipeline registers */ 
module id_ex (clk, rst_n_i, data_mem_stall, data_in, data_out);
	input			clk;
	input           rst_n_i;
	input           data_mem_stall;
	input [177:0]		data_in;
	output reg[177:0]	data_out;

	always @(posedge clk, negedge rst_n_i) begin
		if(!rst_n_i) begin
			data_out <= 178'b0;
		end
		else begin
			data_out <= data_mem_stall ? data_out : data_in;
		end
	end
endmodule



/* EX/MEM pipeline registers */ 
module ex_mem (clk, rst_n_i, data_mem_stall, data_in, data_out);
	input			clk;
	input           rst_n_i;
	input           data_mem_stall;
	input [154:0]		data_in;
	output reg[154:0]	data_out;

	always @(posedge clk, negedge rst_n_i) begin
		if(!rst_n_i) begin
			data_out <= 155'b0;
		end
		else begin
			data_out <= data_mem_stall ? data_out : data_in;
		end
	end
endmodule



/* MEM/WB pipeline registers */ 
module mem_wb (clk, rst_n_i, data_mem_stall, data_in, data_out);
	input			clk;
	input           rst_n_i;
	input           data_mem_stall;
	input [116:0]		data_in;
	output reg[116:0]	data_out;

	always @(posedge clk, negedge rst_n_i) begin
		if(!rst_n_i) begin
			data_out <= 117'b0;
		end
		else begin
			data_out <= data_mem_stall ? data_out : data_in;
		end
	end
endmodule
