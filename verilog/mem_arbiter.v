`define NSECT 4//number of sectors, must be a power of 2 for this design
`define KiB 1024
`define SECT_SIZE 2*`KiB
`define SECT_WIDTH 4 //4 byte width memory block for all sectors

module mem_arbiter(
    input clk_i,
    input rst_n_i,
    

    //instruction fetch interface
    input[31:0] pc_i,
    output reg[31:0] if_rdata_o, // Instruction fetch data output

    //load-store interface
    input[31:0] lsu_addr_ex_i,
    input[31:0] lsu_wr_data_i,
    input signed_i,
    input[`SECT_WIDTH-1:0] bytemask_unshifted_i,
    input lsu_write_i,
    input lsu_read_i,
    output reg[31:0] load_rdata_o, // result from loads
    output reg lsu_stall_ex_o,

    //memory interface for each sector
    //input[31:0] rdata_raw_m0_i[`NSECT-1:0], //data returned from reading
    input[31:0] rdata_raw_m0_0_i,
    input[31:0] rdata_raw_m0_1_i,
    input[31:0] rdata_raw_m0_2_i,
    input[31:0] rdata_raw_m0_3_i,
    //output[`SECT_WIDTH-1:0] wr_bytemask_ex_o[`NSECT-1:0], //This is just bytemask_unshifted_i shifted to align with offset
    output[`SECT_WIDTH-1:0] wr_bytemask_ex_0_o,
    output[`SECT_WIDTH-1:0] wr_bytemask_ex_1_o,
    output[`SECT_WIDTH-1:0] wr_bytemask_ex_2_o,
    output[`SECT_WIDTH-1:0] wr_bytemask_ex_3_o,
    //output[31:0] wr_data_shifted_ex_o[`NSECT-1:0], // Data is shifted to align with addr offset
    output[31:0] wr_data_shifted_ex_0_o,
    output[31:0] wr_data_shifted_ex_1_o,
    output[31:0] wr_data_shifted_ex_2_o,
    output[31:0] wr_data_shifted_ex_3_o,
    //output[31:0] sect_addr_ex_o[`NSECT-1:0], //address of each sector that is to be accessed
    output[31:0] sect_addr_ex_0_o,
    output[31:0] sect_addr_ex_1_o,
    output[31:0] sect_addr_ex_2_o,
    output[31:0] sect_addr_ex_3_o,
    output reg[`NSECT-1:0] sect_wr_req_ex_o, // Which sector to write to
    output reg[`NSECT-1:0] sect_rd_req_ex_o, // Which sector to read from

    //When something goes wrong
    //output exception_cause, //load-access fault? misaliged exception?
    output illegal_access_o
);

//interface assignments
wire[31:0] rdata_raw_m0_i[`NSECT-1:0];
assign rdata_raw_m0_i[0] = rdata_raw_m0_0_i;
assign rdata_raw_m0_i[1] = rdata_raw_m0_1_i;
assign rdata_raw_m0_i[2] = rdata_raw_m0_2_i;
assign rdata_raw_m0_i[3] = rdata_raw_m0_3_i;

reg[`SECT_WIDTH-1:0] wr_bytemask_ex_o[`NSECT-1:0];
assign wr_bytemask_ex_0_o = wr_bytemask_ex_o[0];
assign wr_bytemask_ex_1_o = wr_bytemask_ex_o[1];
assign wr_bytemask_ex_2_o = wr_bytemask_ex_o[2];
assign wr_bytemask_ex_3_o = wr_bytemask_ex_o[3];

reg[31:0] wr_data_shifted_ex_o[`NSECT-1:0];
assign wr_data_shifted_ex_0_o = wr_data_shifted_ex_o[0];
assign wr_data_shifted_ex_1_o = wr_data_shifted_ex_o[1];
assign wr_data_shifted_ex_2_o = wr_data_shifted_ex_o[2];
assign wr_data_shifted_ex_3_o = wr_data_shifted_ex_o[3];

reg[31:0] sect_addr_ex_o[`NSECT-1:0];
assign sect_addr_ex_0_o = sect_addr_ex_o[0];
assign sect_addr_ex_1_o = sect_addr_ex_o[1];
assign sect_addr_ex_2_o = sect_addr_ex_o[2];
assign sect_addr_ex_3_o = sect_addr_ex_o[3];

localparam sect_top_bit_loc = $clog2(`NSECT) + $clog2(`SECT_SIZE) - 1;
localparam sect_bot_bit_loc = $clog2(`SECT_SIZE);

//memory requests by instr fetch and cpu instructions
reg[`NSECT-1:0] if_req_pc; //fetch can only read
reg[`NSECT-1:0] lsu_wr_req_ex;
reg[`NSECT-1:0] lsu_rd_req_ex;
reg[`NSECT-1:0] if_req_if;
//reg[`NSECT-1:0] lsu_rd_req_q;
//reg[`NSECT-1:0] lsu_wr_req_q;
reg[`SECT_WIDTH-1:0] lsu_wr_bytemask_ex;
reg[31:0] lsu_wr_data_shifted_ex;

/* This is in case instr fetch overlaps with load-stores where the latter has priority.
 * I'd be surprised if someone chooses to write code that does this, I hope you know what you're doing.
 * Read-write ordering is undefined if you do this.
 */
wire if_stalled_pc;
reg if_stalled_if;

wire[$clog2(`NSECT)-1:0] if_req_idx_pc;
reg[$clog2(`NSECT)-1:0] if_req_idx_if;
wire[$clog2(`NSECT)-1:0] lsu_req_idx_ex;
reg[$clog2(`NSECT)-1:0] lsu_req_idx_m0;

//arbitrate
integer i;
always @(*) begin
    for(i=0; i<`NSECT; i=i+1) begin
        wr_bytemask_ex_o[i] = `SECT_WIDTH'b0;
        wr_data_shifted_ex_o[i] = 32'b0;
        sect_addr_ex_o[i] = 32'b0;
    end
    //wr_bytemask_ex_o[if_req_idx_pc] = 0; //fetch doesn't write
    wr_bytemask_ex_o[lsu_req_idx_ex] = lsu_wr_bytemask_ex;

    wr_data_shifted_ex_o[lsu_req_idx_ex] = lsu_wr_data_shifted_ex;

    sect_addr_ex_o[if_req_idx_pc] = pc_i;
    if(lsu_rd_req_ex[lsu_req_idx_ex] || lsu_wr_req_ex[lsu_req_idx_ex]) begin
        sect_addr_ex_o[lsu_req_idx_ex] = lsu_addr_ex_i;
    end

    //sect_wr_req_ex_o = `NSECT'b0;
    sect_wr_req_ex_o = lsu_wr_req_ex;

    sect_rd_req_ex_o = (lsu_rd_req_ex | (if_req_pc & ~lsu_wr_req_ex)); //don't overlap with lsu write
end


assign if_req_idx_pc = if_stalled_if ? if_req_idx_if : pc_i[sect_top_bit_loc : sect_bot_bit_loc];
assign lsu_req_idx_ex = lsu_addr_ex_i[sect_top_bit_loc : sect_bot_bit_loc];
assign if_stalled_pc = |(if_req_pc & lsu_wr_req_ex & lsu_wr_req_ex); //actually if lsu and fetch are READING the same location then it can be allowed, but to resolve this special case more logic is needed so it's not implemented.

always @(*) begin
    if_req_pc = `NSECT'b0;
    lsu_wr_req_ex = `NSECT'b0;
    lsu_rd_req_ex = `NSECT'b0;
    //sect_wr_req_ex_o = `NSECT'b0;
    if_req_pc[if_req_idx_pc] = 1'b1; //if is always reading
    lsu_wr_req_ex[lsu_req_idx_ex] = lsu_write_i;
    lsu_rd_req_ex[lsu_req_idx_ex] = lsu_read_i;
    //sect_wr_req_ex_o[lsu_req_idx_ex] = lsu_write_i;
    if(if_stalled_if) begin
        if_req_pc = if_req_if;
    end
end

always @(posedge clk_i or negedge rst_n_i) begin
    if (!rst_n_i) begin
        if_req_if <= `NSECT'b0;
        //lsu_wr_req_q <= `NSECT'b0;
        //lsu_rd_req_q <= `NSECT'b0;
        if_stalled_if <= 1'b0;
        if_req_idx_if <= ($clog2(`NSECT))'('b0);
        lsu_req_idx_m0 <= ($clog2(`NSECT))'('b0);
        lsu_rdata_raw_m1 <= 32'b0;
        byteOffset_m0 <= ($clog2(`NSECT))'('b0);
        byteOffset_m1 <= ($clog2(`NSECT))'('b0);
        size_m0 <= 2'b0;
        size_m1 <= 2'b0;
        if_rdata_q <= 32'b0;
        lsu_stall_m0 <= 1'b0;
    end
    else begin
        if_req_if <= if_req_pc;
        //lsu_wr_req_q <= lsu_wr_req_ex;
        //lsu_rd_req_q <= lsu_wr_req_ex;
        if_stalled_if <= if_stalled_pc;
        if_req_idx_if <= if_req_idx_pc;
        lsu_req_idx_m0 <= lsu_req_idx_ex;
        lsu_rdata_raw_m1 <= lsu_rdata_raw_m0;
        byteOffset_m0 <= byteOffset_ex;
        byteOffset_m1 <= byteOffset_m0;
        size_m0 <= size_ex;
        size_m1 <= size_m0;
        if_rdata_q <= if_rdata_o;
        lsu_stall_m0 <= lsu_stall_ex_o;
    end
end

assign lsu_stall_ex_o = (lsu_write_i || lsu_read_i) && !lsu_stall_m0; //TODO only read needs stalling?

reg[31:0] lsu_rdata_raw_m0;
reg[31:0] lsu_rdata_raw_m1;
reg[31:0] if_rdata_q;
reg lsu_stall_m0;

always @(*) begin
    lsu_rdata_raw_m0 = rdata_raw_m0_i[lsu_req_idx_m0];
    if(if_stalled_if) begin
        if_rdata_o = 32'h00000013; // nop
    end
    else begin
        if_rdata_o = lsu_stall_m0 ? if_rdata_q : rdata_raw_m0_i[if_req_idx_if]; //always aligned to 32-bit boundary //TODO: case statement neater?
    end
end

//shifted lsu write data
wire[$clog2(`SECT_WIDTH)-1:0] byteOffset_ex;
assign byteOffset_ex = lsu_addr_ex_i[$clog2(`SECT_WIDTH)-1:0];

always @(*) begin
    lsu_wr_bytemask_ex = bytemask_unshifted_i << byteOffset_ex;
    case(byteOffset_ex)
        2'b00: lsu_wr_data_shifted_ex = lsu_wr_data_i;
        2'b01: lsu_wr_data_shifted_ex = {lsu_wr_data_i[23:0], 8'b0};
        2'b10: lsu_wr_data_shifted_ex = {lsu_wr_data_i[15:0], 16'b0};
        default: lsu_wr_data_shifted_ex = {lsu_wr_data_i[7:0], 24'b0};
    endcase
end

reg[1:0] size_ex;
reg[1:0] size_m0;
reg[1:0] size_m1;
always @(*) begin
    case(bytemask_unshifted_i)
        4'b0001: size_ex = 2'b00;
        4'b0011: size_ex = 2'b01;
        default: size_ex = 2'b11;
    endcase
end

reg[$clog2(`SECT_WIDTH)-1:0] byteOffset_m0;
reg[$clog2(`SECT_WIDTH)-1:0] byteOffset_m1;

reg[31:0] lsu_rdata_raw_shifted;
always @(*) begin
    case(byteOffset_m1)
        2'b00: lsu_rdata_raw_shifted = lsu_rdata_raw_m1;
        2'b01: lsu_rdata_raw_shifted = {7'b0, lsu_rdata_raw_m1[31:8]};
        2'b10: lsu_rdata_raw_shifted = {15'b0, lsu_rdata_raw_m1[31:16]};
        default: lsu_rdata_raw_shifted = {24'b0, lsu_rdata_raw_m1[31:24]};
    endcase
end

//sign_extend
wire[31:0] lsu_rdata_s8_m1;
wire[31:0] lsu_rdata_s16_m1;
wire[31:0] lsu_rdata_s32_m1;
assign lsu_rdata_s8_m1 = {{24{signed_i && lsu_rdata_raw_shifted[7]}}, lsu_rdata_raw_shifted[7:0]};
assign lsu_rdata_s16_m1 = {{16{signed_i && lsu_rdata_raw_shifted[15]}}, lsu_rdata_raw_shifted[15:0]};
assign lsu_rdata_s32_m1 = lsu_rdata_raw_shifted;

always @(*) begin
    case(size_m1)
        2'b00: load_rdata_o = lsu_rdata_s8_m1;
        2'b01: load_rdata_o = lsu_rdata_s16_m1;
        default: load_rdata_o = lsu_rdata_s32_m1;
    endcase
end

assign illegal_access_o = |lsu_addr_ex_i[31:sect_top_bit_loc+1] || |pc_i[31:sect_top_bit_loc+1] || |pc_i[$clog2(`SECT_WIDTH)-1:0];

endmodule