

module mem_block #(parameter DEPTH=512, parameter WIDTH=4)(
	input clk,
	input rst_n_i,
	input[$clog2(DEPTH*WIDTH)-1:0] addr_i,
	input[31:0] wr_data_i,
	input[WIDTH-1:0] bytemask_i,
	input write_en_i,
	input read_en_i,
	output[31:0] rd_data_o
);

reg[7:0] mline_output[WIDTH-1:0];
reg[7:0] mem_column[DEPTH-1:0][WIDTH-1:0];

//wire[7:0] wr_data_col[WIDTH-1:0];


wire[$clog2(DEPTH)-1:0] mcol_idx;
assign mcol_idx = addr_i[$clog2(DEPTH*WIDTH)-1:$clog2(WIDTH)];

generate
	genvar i;
	for(i=0; i<WIDTH; i=i+1) begin //for each byte along the width
		always @(posedge clk, negedge rst_n_i) begin
			if(!rst_n_i) begin
				mline_output[i] <= 8'b0;
			end
			else begin
				mline_output[i] <= read_en_i ? mem_column[i][mcol_idx] : mline_output[i];
				if(write_en_i && bytemask_i[i]) begin
					mem_column[i][mcol_idx] <= wr_data_i[8*i +: 8];
				end
			end
		end

		always @(*) begin
			rd_data_o[8*i +: 8] = mline_output[i];
		end
	end
endgenerate

endmodule