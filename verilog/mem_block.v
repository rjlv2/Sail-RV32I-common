

module mem_block #(parameter DEPTH=512, parameter WIDTH=4)(
	input clk,
	input rst_n_i,
	input[$clog2(DEPTH*WIDTH)-1:0] addr_i,
	input[31:0] wr_data_i,
	input[WIDTH-1:0] bytemask_i,
	input write_en_i,
	input read_en_i,
	output reg[31:0] rd_data_o
);

reg[7:0] mline_output[WIDTH-1:0];
reg[7:0] mem_column[WIDTH-1:0][DEPTH-1:0];

wire[$clog2(DEPTH)-1:0] mcol_idx;
assign mcol_idx = addr_i[$clog2(DEPTH*WIDTH)-1:$clog2(WIDTH)];

generate
	genvar i;
	for(i=0; i<WIDTH; i=i+1) begin : mcolumn //for each byte along the width
		always @(posedge clk) begin
			if(write_en_i && bytemask_i[i]) begin
				mem_column[i][mcol_idx] <= wr_data_i[8*i +: 8];
			end
			mline_output[i] <= read_en_i ? mem_column[i][mcol_idx] : mline_output[i];
		end

		always @(*) begin
			rd_data_o[8*i +: 8] = mline_output[i];
		end
	end
endgenerate

endmodule

/*module ram_single(q, a, d, we, clk);
   output[7:0] q;
   input [7:0] d;
   input [6:0] a;
   input we, clk;
   reg [7:0] mem [127:0];
    always @(posedge clk) begin
        if (we)
            mem[a] <= d;
        q <= mem[a];
   end
endmodule*/