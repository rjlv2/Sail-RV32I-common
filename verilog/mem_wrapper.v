`define MEM_SIZE 2048

module mem_wrapper(
		input clk_i,
		input rst_n_i,
		input[10:0] addr_i,
		input[31:0] wr_data_i,
		input[3:0] bytemask_i, //4 bits for 4 byte work
		input write_en_i,
		input read_en_i,
		output[31:0] rd_data_o
	);

wire[$clog2(512)-1:0] sbram_addr;
assign sbram_addr = addr_i[$clog2(512)-1 : $clog2(4)];

reg[31:0] rdata_out;

parameter addr_width = 9;
parameter data_width = 8;
generate
	genvar i;
	for(i=0; i<4; i=i+1) begin
		reg [data_width-1:0] mem [(1<<addr_width)-1:0];
		always @(posedge clk_i) begin
			if(write_en_i && bytemask_i[i]) begin
				mem[(sbram_addr)] <= wr_data_i[8*i +: 8];
			end
			rdata_out[8*i +: 8] <= mem[sbram_addr];
		end
	end
endgenerate

assign rd_data_o = rdata_out;

endmodule