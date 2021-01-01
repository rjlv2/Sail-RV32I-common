module uart_buffer(
	input clk,
	input rst_n_i,
	input[31:0] addr_i,
	input[31:0] wdata_i,
	input buf_rnw_i,
	output[31:0] rdata_o,
	output rx_buffer_full_o,
	output rx_buffer_empty_o,
	output rx_buf_access_o
);

assign rx_buf_access_o = valid_rx_buf_read || valid_rx_buf_write;

//for now, set address 0x10000 as rx read pointer, 0x10004 as rx write pointer
//set address 0x20000 as tx read pointer, 0x10014 as tx write pointer
//set address 0x30000 as rx buffer start
//set address 0x40000 as rx buffer start
//and it's only byte accessible for now, no matter what type of load/store you do

reg[7:0] rx_read_ptr; //use 128 byte buffer, 1 extra bit is used for keeping track 
reg[7:0] rx_write_ptr; //actual index into buffer is [6:0]
//reg[7:0] tx_read_ptr;
//reg[7:0] tx_write_ptr;

//wire rx_ptr_access;
//wire tx_ptr_access;
wire rx_buf_access;
//wire tx_buf_access;
assign rx_buf_access = (addr_i[18:16] == 3'h1); //TODO use a more compact form
//assign tx_buf_access = (addr_i[18:16] == 3'h2);
//assign rx_ptr_access = (addr_i[18:16] == 3'h3);
//assign tx_ptr_access = (addr_i[18:16] == 3'h4);

assign rx_buffer_full_o  = (rx_read_ptr[6:0] == rx_write_ptr[6:0]) && (rx_read_ptr[7] ^ rx_write_ptr[7]);
assign rx_buffer_empty_o = (rx_read_ptr == rx_write_ptr);

wire valid_rx_buf_read;
wire valid_rx_buf_write;
assign valid_rx_buf_read  = (rx_buf_access && buf_rnw_i && !rx_buffer_empty_o);
assign valid_rx_buf_write = (rx_buf_access && !buf_rnw_i && !rx_buffer_full_o);

reg[31:0] rx_read_ptr_n;
reg[31:0] rx_write_ptr_n;
always @(*) begin
	rx_read_ptr_n  = valid_rx_buf_read  ? rx_read_ptr  + 10'b1 : rx_read_ptr;
	rx_write_ptr_n = valid_rx_buf_write ? rx_write_ptr + 10'b1 : rx_write_ptr;
end

always @(posedge clk, negedge rst_n_i) begin
	if(!rst_n_i) begin
		rx_read_ptr  <= 8'b0;
		rx_write_ptr <= 8'b0;
		//tx_read_ptr  <= 10'b0;
		//tx_write_ptr <= 10'b0;
	end
	else begin
		rx_read_ptr  <= rx_read_ptr_n;
		rx_write_ptr <= rx_write_ptr_n;
	end
end

reg[7:0] rx_buffer [127:0];

reg[31:0] rx_read_data;
always @(posedge clk, negedge rst_n_i) begin
	if (!rst_n_i) begin
		rx_read_data <= 32'b0;
	end
	else begin
		rx_read_data <= valid_rx_buf_read ? {24'b0, rx_buffer[rx_read_ptr[6:0]]} : 32'hdeadbeef;
	end
end

always @(posedge clk) begin
	//for memory, there's no rst, just latch on
	if (valid_rx_buf_write) begin
		rx_buffer[rx_write_ptr[6:0]] <= wdata_i[7:0];
	end
end

assign rdata_o = rx_read_data;

endmodule