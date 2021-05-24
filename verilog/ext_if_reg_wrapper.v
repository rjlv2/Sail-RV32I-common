

module ext_if_reg_wrapper(
	input clk,
	input rst_n_i,

	// Load-store from cpu
	input[31:0] addr_i,
	input[31:0] wdata_i,
	input mem_read_i,
	input mem_write_i,
	output reg[31:0] rdata_o,

	// UART RX insterface
	input rx_buf_write_i, //write to buffer by uart
	input[7:0] rx_buf_wr_data_i,
	output rx_buffer_full_o,
	output rx_buffer_empty_o,
	
	//external led
	output [7:0] led_o
);

//TODO: make changes so that cpu can perform buffer accesses larger than a byte 
/*
	For bottom 11 bits of address:
		Reads from 0x0 will return value in buffer pointed to by rx_read_ptr, writes do nothing, reads from empty buffer return 0xdeadbeef
		Reads from 0x4 will return status of rx_buffer, bits {0: rx buffer is empty, 1: rx buffer is full}, writes here do nothing
		Writes to 0x8 will write bits [7:0] of wdata_i to the tx buffer location pointed to by tx_wr_ptr, if it's full then the write will be discarded, reads from here will return deadbeef
		Reads from 0xc will return status of tx_buffer, bits {0: rx buffer is empty, 1: rx buffer is full}, writes here do nothing
		Writes to 0x10 will write bits [7:0] of wdata_i to the led register, which is connected to external output pins, reads from here will read the zero-padded 8-bit content of the register
*/
wire addr_is_rx_buf;
wire addr_is_rx_sts;
//wire addr_is_tx_buf; //TODO: implement tx for sending messages out
//wire addr_is_tx_sts; //TODO: implement tx for sending messages out
wire addr_is_led_o;
assign addr_is_rx_buf = (addr_i[10:0] == 11'h0);
assign addr_is_rx_sts = (addr_i[10:0] == 11'h4);
//assign addr_is_tx_buf = (addr_i[10:0] == 11'h8); //TODO: implement tx for sending messages out
//assign addr_is_tx_sts = (addr_i[10:0] == 11'hc); //TODO: implement tx for sending messages out
assign addr_is_led_o  = (addr_i[10:0] == 11'h10); //external led output, modify this section to create I/O


/***************** uart rx buffer *****************/

reg[7:0] rx_buffer [7:0]; //TODO replace magic numbers

//these pointers are hidden to the program, the value of the byte pointed to by the read pointer is exposed through a register
reg[3:0] rx_read_ptr; //use 8 byte buffer, 1 extra bit is used for keeping track of circular buffer
reg[3:0] rx_write_ptr; //actual index into buffer is [2:0]

assign rx_buffer_full_o  = (rx_read_ptr[2:0] == rx_write_ptr[2:0]) && (rx_read_ptr[3] ^ rx_write_ptr[3]); //top bit keeps track of circular buffer
assign rx_buffer_empty_o = (rx_read_ptr == rx_write_ptr);

wire valid_rx_buf_read;
wire valid_rx_buf_write;
assign valid_rx_buf_read  = addr_is_rx_buf && mem_read_i && !rx_buffer_empty_o;
assign valid_rx_buf_write = rx_buf_write_i && !rx_buffer_full_o; //data is dropped when it's full

reg[3:0] rx_read_ptr_n;
reg[3:0] rx_write_ptr_n;
always @(*) begin
	rx_read_ptr_n  = valid_rx_buf_read  ? rx_read_ptr  + 4'b1 : rx_read_ptr;
	rx_write_ptr_n = valid_rx_buf_write ? rx_write_ptr + 4'b1 : rx_write_ptr;
end

reg[31:0] rx_read_val;
always @(posedge clk, negedge rst_n_i) begin
	if (!rst_n_i) begin
		rx_read_ptr  <= 4'b0;
		rx_write_ptr <= 4'b0;
	end
	else begin
		rx_read_ptr  <= rx_read_ptr_n;
		rx_write_ptr <= rx_write_ptr_n;
		if(valid_rx_buf_read) begin
			rx_read_val <= {24'b0, rx_buffer[rx_read_ptr]}; //only read out value if valid read
		end
		if(valid_rx_buf_write) begin
			rx_buffer[rx_write_ptr] <= rx_buf_wr_data_i;
		end
	end
end


/***************** uart tx buffer *****************/

//TODO: implement tx for sending messages out

/********************** LEDs **********************/

wire valid_led_read;
wire valid_led_write;
assign valid_led_read  = addr_is_led_o && mem_read_i;
assign valid_led_write = addr_is_led_o && mem_write_i;

reg[7:0] led_reg;
reg[7:0] led_reg_n;
wire[7:0] led_reg_val;
always @(*) begin
	led_reg_n = led_reg;
	if (valid_led_write) begin
		led_reg_n = wdata_i[7:0];
	end
end

always @(posedge clk, negedge rst_n_i) begin
	if (!rst_n_i) begin
		led_reg <= 8'b0;
	end
	else begin
		led_reg <= led_reg_n;
	end
end

assign led_reg_val = led_reg;
assign led_o = led_reg;

/*************** select read output ***************/

reg valid_rx_buf_read_q;
reg valid_rx_sts_read_q;
reg valid_led_read_q;

always @(posedge clk or negedge rst_n_i) begin
	if (!rst_n_i) begin
		valid_rx_buf_read_q <= 1'b0;
		valid_rx_sts_read_q <= 1'b0;
		valid_led_read_q    <= 1'b0;
	end
	else begin
		valid_rx_buf_read_q <= valid_rx_buf_read;
		valid_rx_sts_read_q <= addr_is_rx_sts && mem_read_i;
		valid_led_read_q    <= valid_led_read;
	end
end

always @(*) begin
	case(1'b1)
		valid_rx_buf_read_q: rdata_o = rx_read_val;
		valid_rx_sts_read_q: rdata_o = {30'b0, rx_buffer_full_o, rx_buffer_empty_o};
		valid_led_read_q   : rdata_o = {24'b0, led_reg_val};
		default: rdata_o = 32'hdeadbeef; // Did you read the correct address? Was the buffer empty when you attempted to read it?
	endcase
end

endmodule