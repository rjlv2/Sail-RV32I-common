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
 *	cpu top-level
 */



module cpu(
			clk,
			rst_n_i,
			inst_mem_in,
			inst_mem_out,
			data_mem_out,
			data_mem_addr,
			data_mem_WrData,
			data_mem_memwrite,
			data_mem_memread,
			data_mem_sign_mask,
			data_mem_stall
		);
	/*
	 *	Input Clock
	 */
	input clk;
	input rst_n_i;

	/*
	 *	instruction memory input
	 */
	output [31:0]		inst_mem_in;
	input [31:0]		inst_mem_out;

	/*
	 *	Data Memory
	 */
	input [31:0]		data_mem_out;
	output [31:0]		data_mem_addr;
	output [31:0]		data_mem_WrData;
	output			data_mem_memwrite;
	output			data_mem_memread;
	output [3:0]		data_mem_sign_mask;
	input 			data_mem_stall;

	/*
	 *	Program Counter
	 */
	wire [31:0]		pc_mux0;
	wire [31:0]		pc_in;
	wire [31:0]		pc_out;
	wire			pcsrc;
	wire [31:0]		inst_mux_out;
	wire [31:0]		fence_mux_out;

	/*
	 *	Pipeline Registers
	 */
	//wire [63:0]		if_id_out;
	//wire [177:0]		id_ex_out;
	//wire [154:0]		ex_mem_out;
	//wire [116:0]		mem_wb_out;

	/*
	 *	Control signals
	 */
	wire			MemtoReg_id;
	wire			RegWrite_id;
	wire			MemWrite_id;
	wire			MemRead_id;
	wire			Branch_id;
	wire			Jump_id;
	wire			Jalr_id;
	wire			ALUSrc_id;
	wire			Lui_id;
	wire			Auipc_id;
	wire			Fence_signal;
	wire			CSRR_signal_id;
	wire			CSRRI_signal;

	/*
	 *	Decode stage
	 */
	wire [10:0]		cont_mux_out; //control signal mux
	wire [31:0]		regA_out;
	wire [31:0]		regB_out;
	wire [31:0]		imm_out;
	wire [31:0]		RegA_mux_out;
	wire [31:0]		RegB_mux_out;
	wire [ 4:0]		RegA_AddrFwdFlush_mux_out;
	wire [31:0]		RegB_AddrFwdFlush_mux_out;
	wire [31:0]		rdValOut_CSR;
	wire [3:0]		dataMem_sign_mask;
	wire 			aluCtl_sign_id;

	/*
	 *	Execute stage
	 */
	wire [31:0]		addr_adder_mux_out;
	wire [31:0]		alu_mux_out;
	wire [31:0]		addr_adder_sum_ex;
	wire [6:0]		alu_ctl;
	wire			alu_branch_enable;
	wire [31:0]		alu_result;
	wire [31:0]		alu_result_ex;
	reg 			aluCtl_sign_ex;

	/*
	 *	Memory access stage
	 */
	wire [31:0]		auipc_mux_out;
	wire [31:0]		mem_csrr_mux_out;

	/*
	 *	Writeback to registers stage
	 */
	wire [31:0]		wb_mux_out;
	wire [31:0]		reg_dat_mux_out;

	/*
	 *	Forwarding multiplexer wires
	 */
	wire [31:0]		dataMemOut_fwd_mux_out;
	wire [31:0]		mem_fwd1_mux_out;
	wire [31:0]		mem_fwd2_mux_out;
	wire [31:0]		wb_fwd1_mux_out;
	wire [31:0]		wb_fwd2_mux_out;
	wire			mfwd1;
	wire			mfwd2;
	wire			wfwd1;
	wire			wfwd2;

	/*
	 *	Branch Predictor
	 */
	wire [31:0]		pc_adder_out;
	wire [31:0]		branch_predictor_addr;
	wire			predict;
	wire [31:0]		branch_predictor_mux_out;
	wire			actual_branch_decision;
	wire			mistake_trigger;
	wire			decode_ctrl_mux_sel;
	wire			inst_mux_sel;

	/*
	 *	Instruction Fetch Stage
	 */

	assign pc_in = pcsrc ? addr_adder_sum_m0 : pc_mux0;


	adder pc_adder(
			.input1(32'b100),
			.input2(pc_out),
			.out(pc_adder_out)
		);

	program_counter PC(
			.inAddr(pc_in),
			.outAddr(pc_out),
			.clk(clk),
			.rst_n_i(rst_n_i),
			.data_mem_stall(data_mem_stall)
		);

	assign inst_mux_out = inst_mux_sel ? 32'b0 : inst_mem_out;
	assign fence_mux_out = Fence_signal ? pc_out : pc_adder_out;

	/*
	 *	IF/ID Pipeline Register
	 */

	/*if_id if_id_reg(
			.clk(clk),
			.rst_n_i(rst_n_i),
			.data_mem_stall(data_mem_stall),
			.data_in({inst_mux_out, pc_out}),
			.data_out(if_id_out)
		);*/

	reg [31:0] instr_id;
	reg [31:0] pc_id;
	always @(posedge clk, negedge rst_n_i) begin
		if(!rst_n_i) begin
			instr_id <= 32'b0;
			pc_id    <= 32'b0;
		end
		else begin
			instr_id <= data_mem_stall ? instr_id : inst_mux_out;
			pc_id    <= data_mem_stall ? pc_id    : pc_out;
		end
	end

	/*
	 *	Decode Stage
	 */
	control control_unit(
			.opcode({instr_id[6:0]}),
			.MemtoReg(MemtoReg_id),
			.RegWrite(RegWrite_id),
			.MemWrite(MemWrite_id),
			.MemRead(MemRead_id),
			.Branch(Branch_id),
			.ALUSrc(ALUSrc_id),
			.Jump(Jump_id),
			.Jalr(Jalr_id),
			.Lui(Lui_id),
			.Auipc(Auipc_id),
			.Fence(Fence_signal),
			.CSRR(CSRR_signal_id)
		);

	wire Jalr_gtd_id;
	wire ALUSrc_gtd_id;
	wire Lui_gtd_id;
	wire Auipc_gtd_id;
	wire Branch_gtd_id;
	wire MemRead_gtd_id;
	wire MemWrite_gtd_id;
	wire CSRR_signal_gtd_id;
	wire RegWrite_gtd_id;
	wire MemtoReg_gtd_id;
	wire Jump_gtd_id;
	assign cont_mux_out = decode_ctrl_mux_sel ? 11'b0 : {Jalr_id, ALUSrc_id, Lui_id, Auipc_id, Branch_id, MemRead_id, MemWrite_id, CSRR_signal_id, RegWrite_id, MemtoReg_id, Jump_id};
	assign Jalr_gtd_id = decode_ctrl_mux_sel ? 1'b0 : Jalr_id;
	assign ALUSrc_gtd_id = decode_ctrl_mux_sel ? 1'b0 : ALUSrc_id;
	assign Lui_gtd_id = decode_ctrl_mux_sel ? 1'b0 : Lui_id;
	assign Auipc_gtd_id = decode_ctrl_mux_sel ? 1'b0 : Auipc_id;
	assign Branch_gtd_id = decode_ctrl_mux_sel ? 1'b0 : Branch_id;
	assign MemRead_gtd_id = decode_ctrl_mux_sel ? 1'b0 : MemRead_id;
	assign MemWrite_gtd_id = decode_ctrl_mux_sel ? 1'b0 : MemWrite_id;
	assign CSRR_signal_gtd_id = decode_ctrl_mux_sel ? 1'b0 : CSRR_signal_id;
	assign RegWrite_gtd_id = decode_ctrl_mux_sel ? 1'b0 : RegWrite_id;
	assign MemtoReg_gtd_id = decode_ctrl_mux_sel ? 1'b0 : MemtoReg_id;
	assign Jump_gtd_id = decode_ctrl_mux_sel ? 1'b0 : Jump_id;

	regfile register_files(
			.clk(clk),
			.rst_n_i(rst_n_i),
			.data_mem_stall(data_mem_stall),
			.write(RegWrite_m0),
			.wrAddr(rd_addr_m0),
			.wrData(reg_dat_mux_out),
			.rdAddrA(inst_mux_out[19:15]),
			.rdDataA(regA_out),
			.rdAddrB(inst_mux_out[24:20]),
			.rdDataB(regB_out)
		);

	imm_gen immediate_generator(
			.inst(instr_id),
			.imm(imm_out)
		);

	ALUControl alu_control(
			.Opcode(instr_id[6:0]),
			.FuncCode({instr_id[30], instr_id[14:12]}),
			.ALUCtl(alu_ctl),
			.sign(aluCtl_sign_id)
		);

	sign_mask_gen sign_mask_gen_inst(
			.func3(instr_id[14:12]),
			.sign_mask(dataMem_sign_mask)
		);

	csr_file ControlAndStatus_registers(
			.clk(clk),
			.write(CSRR_signal_wb), //TODO
			.wrAddr_CSR(imm12_wb),
			.wrVal_CSR(alu_result_wb),
			.rdAddr_CSR(inst_mux_out[31:20]),
			.rdVal_CSR(rdValOut_CSR)
		);

	assign RegA_mux_out = CSRRI_signal ? {27'b0, instr_id[19:15]} : regA_out;
	assign RegB_mux_out = CSRR_signal_id ? rdValOut_CSR : regB_out;
	assign RegA_AddrFwdFlush_mux_out = CSRRI_signal ? 5'b0 : instr_id[19:15];
	assign RegB_AddrFwdFlush_mux_out = CSRR_signal_id ? 5'b0 : instr_id[24:20];

	assign CSRRI_signal = CSRR_signal_id & (instr_id[14]);

	//ID/EX Pipeline Register
	reg [11:0] imm12_ex;
	reg [ 4:0] regB_addr_ex;
	reg [ 4:0] regA_addr_ex;
	reg [ 4:0] rd_addr_ex;
	reg [ 3:0] dataMem_sign_mask_ex;
	reg [ 6:0] alu_ctl_ex;
	reg [31:0] imm_sext_ex;
	reg [31:0] RegB_val_ex;
	reg [31:0] RegA_val_ex;
	reg [31:0] pc_ex;
	reg        predict_ex;

	reg Jalr_ex;
	reg ALUSrc_ex;
	reg Lui_ex;
	reg Auipc_ex;
	reg Branch_ex;
	reg MemRead_ex;
	reg MemWrite_ex;
	reg CSRR_signal_ex;
	reg RegWrite_ex;
	reg MemtoReg_ex;
	reg Jump_ex;
	always @(posedge clk or negedge rst_n_i) begin
		if (!rst_n_i) begin
			imm12_ex <= 12'b0;
			regB_addr_ex <= 5'b0;
			regA_addr_ex <= 5'b0;
			rd_addr_ex <= 5'b0;
			dataMem_sign_mask_ex <= 4'b0;
			alu_ctl_ex <= 7'b0;
			imm_sext_ex <= 32'b0;
			RegB_val_ex <= 32'b0;
			RegA_val_ex <= 32'b0;
			pc_ex <= 32'b0;
			Jalr_ex <= 1'b0;
			ALUSrc_ex <= 1'b0;
			Lui_ex <= 1'b0;
			Auipc_ex <= 1'b0;
			Branch_ex <= 1'b0;
			MemRead_ex <= 1'b0;
			MemWrite_ex <= 1'b0;
			CSRR_signal_ex <= 1'b0;
			RegWrite_ex <= 1'b0;
			MemtoReg_ex <= 1'b0;
			Jump_ex <= 1'b0;
			predict_ex <= 1'b0;
			aluCtl_sign_ex <= 1'b0;
		end
		else begin
			imm12_ex     <= data_mem_stall ? imm12_ex : instr_id[31:20];
			regB_addr_ex <= data_mem_stall ? regB_addr_ex : RegB_AddrFwdFlush_mux_out;
			regA_addr_ex <= data_mem_stall ? regA_addr_ex : RegA_AddrFwdFlush_mux_out;
			rd_addr_ex   <= data_mem_stall ? rd_addr_ex : instr_id[11:7];
			dataMem_sign_mask_ex <= data_mem_stall ? dataMem_sign_mask_ex : dataMem_sign_mask;
			alu_ctl_ex   <= data_mem_stall ? alu_ctl_ex : alu_ctl;
			imm_sext_ex  <= data_mem_stall ? imm_sext_ex : imm_out;
			RegB_val_ex  <= data_mem_stall ? RegB_val_ex : RegB_mux_out;
			RegA_val_ex  <= data_mem_stall ? RegA_val_ex : RegA_mux_out;
			pc_ex        <= data_mem_stall ? pc_ex : pc_id;
			Jalr_ex      <= data_mem_stall ? Jalr_ex : Jalr_gtd_id;
			ALUSrc_ex    <= data_mem_stall ? ALUSrc_ex : ALUSrc_gtd_id;
			Lui_ex       <= data_mem_stall ? Lui_ex : Lui_gtd_id;
			Auipc_ex     <= data_mem_stall ? Auipc_ex : Auipc_gtd_id;
			Branch_ex    <= data_mem_stall ? Branch_ex : Branch_gtd_id;
			MemRead_ex   <= data_mem_stall ? MemRead_ex : MemRead_gtd_id;
			MemWrite_ex  <= data_mem_stall ? MemWrite_ex : MemWrite_gtd_id;
			CSRR_signal_ex <= data_mem_stall ? CSRR_signal_ex : CSRR_signal_gtd_id;
			RegWrite_ex    <= data_mem_stall ? RegWrite_ex : RegWrite_gtd_id;
			MemtoReg_ex    <= data_mem_stall ? MemtoReg_ex : MemtoReg_gtd_id;
			Jump_ex        <= data_mem_stall ? Jump_ex : Jump_gtd_id;
			predict_ex     <= data_mem_stall ? predict_ex : predict;
			aluCtl_sign_ex <= data_mem_stall ? aluCtl_sign_ex : aluCtl_sign_id;
		end
	end

	/*id_ex id_ex_reg(
			.clk(clk),
			.rst_n_i(rst_n_i),
			.data_mem_stall(data_mem_stall),
			.data_in({instr_id[31:20], RegB_AddrFwdFlush_mux_out, RegA_AddrFwdFlush_mux_out, instr_id[11:7], dataMem_sign_mask, alu_ctl, imm_out, RegB_mux_out, RegA_mux_out, pc_id, cont_mux_out[10:7], predict, cont_mux_out[6:0]}),
			.data_out(id_ex_out)
		);*/
	wire Auipc_gtd_ex;
	wire predict_gtd_ex;
	wire Branch_gtd_ex;
	wire MemRead_gtd_ex;
	wire MemWrite_gtd_ex;
	wire CSRR_signal_gtd_ex;
	wire RegWrite_gtd_ex;
	wire MemtoReg_gtd_ex;
	wire Jump_gtd_ex;
	assign Auipc_gtd_ex = pcsrc ? 1'b0 : Auipc_ex;
	assign predict_gtd_ex = pcsrc ? 1'b0 : predict_ex;
	assign Branch_gtd_ex = pcsrc ? 1'b0 : Branch_ex;
	assign MemRead_gtd_ex = pcsrc ? 1'b0 : MemRead_ex;
	assign MemWrite_gtd_ex = pcsrc ? 1'b0 : MemWrite_ex;
	assign CSRR_signal_gtd_ex = pcsrc ? 1'b0 : CSRR_signal_ex;
	assign RegWrite_gtd_ex = pcsrc ? 1'b0 : RegWrite_ex;
	assign MemtoReg_gtd_ex = pcsrc ? 1'b0 : MemtoReg_ex;
	assign Jump_gtd_ex = pcsrc ? 1'b0 : Jump_ex;

	assign addr_adder_mux_out = Jalr_ex ? wb_fwd1_mux_out : pc_ex;

	adder addr_adder(
			.input1(addr_adder_mux_out),
			.input2(imm_sext_ex),
			.out(addr_adder_sum_ex)
		);

	assign alu_mux_out = ALUSrc_ex ? imm_sext_ex : wb_fwd2_mux_out;

	alu alu_main(
			.ALUctl(alu_ctl_ex),
			.A(wb_fwd1_mux_out),
			.B(alu_mux_out),
			.sign(aluCtl_sign_ex),
			.ALUOut(alu_result),
			.Branch_Enable(alu_branch_enable)
		);

	assign alu_result_ex = Lui_ex ? imm_sext_ex : alu_result;

	//EX/MEM Pipeline Register
	/*ex_mem ex_mem_reg(
			.clk(clk),
			.rst_n_i(rst_n_i),
			.data_mem_stall(data_mem_stall),
			.data_in({imm12_ex, rd_addr_ex, wb_fwd2_mux_out, alu_result_ex, alu_branch_enable, addr_adder_sum_ex, pc_ex, ex_cont_mux_out}),
			.data_out(ex_mem_out)
		);*/

	reg[11:0] imm12_m0;
	reg[ 4:0] rd_addr_m0;
	reg[31:0] wrData_m0;
	reg[31:0] alu_result_m0;
	reg alu_branch_enable_m0;
	reg[31:0] addr_adder_sum_m0;
	reg[31:0] pc_m0;
	reg Auipc_m0;
	reg predict_m0;
	reg Branch_m0;
	reg MemRead_m0;
	reg MemWrite_m0;
	reg CSRR_signal_m0;
	reg RegWrite_m0;
	reg MemtoReg_m0;
	reg Jump_m0;
	always @(posedge clk or negedge rst_n_i) begin
		if (!rst_n_i) begin
			imm12_m0             <= 12'b0;
			rd_addr_m0           <= 5'b0;
			wrData_m0            <= 32'b0;
			alu_result_m0        <= 32'b0;
			alu_branch_enable_m0 <= 1'b0;
			addr_adder_sum_m0    <= 32'b0;
			pc_m0                <= 32'b0;
			Auipc_m0             <= 1'b0;
			predict_m0           <= 1'b0;
			Branch_m0            <= 1'b0;
			MemRead_m0           <= 1'b0;
			MemWrite_m0          <= 1'b0;
			CSRR_signal_m0       <= 1'b0;
			RegWrite_m0          <= 1'b0;
			MemtoReg_m0          <= 1'b0;
			Jump_m0              <= 1'b0;
		end
		else begin
			imm12_m0             <= data_mem_stall ? imm12_m0 : imm12_ex;
			rd_addr_m0           <= data_mem_stall ? rd_addr_m0 : rd_addr_ex;
			wrData_m0            <= data_mem_stall ? wrData_m0 : wb_fwd2_mux_out;
			alu_result_m0        <= data_mem_stall ? alu_result_m0 : alu_result_ex;
			alu_branch_enable_m0 <= data_mem_stall ? alu_branch_enable_m0 : alu_branch_enable;
			addr_adder_sum_m0    <= data_mem_stall ? addr_adder_sum_m0 : addr_adder_sum_ex;
			pc_m0                <= data_mem_stall ? pc_m0 : pc_ex;
			Auipc_m0             <= data_mem_stall ? Auipc_m0 : Auipc_gtd_ex;
			predict_m0           <= data_mem_stall ? predict_m0 : predict_gtd_ex;
			Branch_m0            <= data_mem_stall ? Branch_m0 : Branch_gtd_ex;
			MemRead_m0           <= data_mem_stall ? MemRead_m0 : MemRead_gtd_ex;
			MemWrite_m0          <= data_mem_stall ? MemWrite_m0 : MemWrite_gtd_ex;
			CSRR_signal_m0       <= data_mem_stall ? CSRR_signal_m0 : CSRR_signal_gtd_ex;
			RegWrite_m0          <= data_mem_stall ? RegWrite_m0 : RegWrite_gtd_ex;
			MemtoReg_m0          <= data_mem_stall ? MemtoReg_m0 : MemtoReg_gtd_ex;
			Jump_m0              <= data_mem_stall ? Jump_m0 : Jump_gtd_ex;
		end
	end

	//Memory Access Stage
	branch_decision branch_decide(
			.Branch(Branch_m0),
			.Predicted(predict_m0),
			.Branch_Enable(alu_branch_enable_m0),
			.Jump(Jump_m0),
			.Mispredict(mistake_trigger),
			.Decision(actual_branch_decision),
			.Branch_Jump_Trigger(pcsrc)
		);

	assign auipc_mux_out = Auipc_m0 ? addr_adder_sum_m0 : alu_result_m0;
	assign mem_csrr_mux_out = CSRR_signal_m0 ? wrData_m0 : auipc_mux_out;

	//MEM/WB Pipeline Register
	/*mem_wb mem_wb_reg(
			.clk(clk),
			.rst_n_i(rst_n_i),
			.data_mem_stall(data_mem_stall),
			.data_in({imm12_m0, rd_addr_m0, data_mem_out, mem_csrr_mux_out, alu_result_m0, CSRR_signal_m0, RegWrite_m0, MemtoReg_m0, Jump_m0}),
			.data_out(mem_wb_out)
		);*/

	reg[11:0] imm12_wb;
	reg[ 4:0] rd_addr_wb;
	reg[31:0] data_mem_out_wb;
	reg[31:0] auipc_result_wb;
	reg[31:0] alu_result_wb;
	reg       CSRR_signal_wb;
	reg       RegWrite_wb;
	reg       MemtoReg_wb;
	reg       Jump_wb;

	always @(posedge clk or negedge rst_n_i) begin
		if (!rst_n_i) begin
			imm12_wb        <= 12'b0;
			rd_addr_wb      <= 5'b0;
			data_mem_out_wb <= 32'b0;
			auipc_result_wb <= 32'b0;
			alu_result_wb   <= 32'b0;
			CSRR_signal_wb  <= 1'b0;
			RegWrite_wb     <= 1'b0;
			MemtoReg_wb     <= 1'b0;
			Jump_wb         <= 1'b0;
		end
		else begin //Allow writeback stage to proceed even with data mem stall
			imm12_wb        <= data_mem_stall ? imm12_wb : imm12_m0;
			rd_addr_wb      <= data_mem_stall ? rd_addr_wb : rd_addr_m0;
			data_mem_out_wb <= data_mem_stall ? data_mem_out_wb : data_mem_out;
			auipc_result_wb <= data_mem_stall ? auipc_result_wb : mem_csrr_mux_out;
			alu_result_wb   <= data_mem_stall ? alu_result_wb : alu_result_m0;
			CSRR_signal_wb  <= data_mem_stall ? CSRR_signal_wb : CSRR_signal_m0;
			RegWrite_wb     <= data_mem_stall ? RegWrite_wb : RegWrite_m0;
			MemtoReg_wb     <= data_mem_stall ? MemtoReg_wb : MemtoReg_m0;
			Jump_wb         <= data_mem_stall ? Jump_wb : Jump_m0;
		end
	end

	assign wb_mux_out = MemtoReg_wb ? data_mem_out_wb : auipc_result_wb;
	assign reg_dat_mux_out = Jump_m0 ? pc_ex : mem_regwb_mux_out;

	//Forwarding Unit
	ForwardingUnit forwarding_unit(
			.rs1(regA_addr_ex),
			.rs2(regB_addr_ex),
			.MEM_RegWriteAddr(rd_addr_m0),
			.WB_RegWriteAddr(rd_addr_wb),
			.MEM_RegWrite(RegWrite_m0),
			.WB_RegWrite(RegWrite_wb),
			.EX_CSRR_Addr(imm12_ex),
			.MEM_CSRR_Addr(imm12_m0),
			.WB_CSRR_Addr(imm12_wb),
			.MEM_CSRR(CSRR_signal_m0),
			.WB_CSRR(CSRR_signal_wb),
			.MEM_fwd1(mfwd1),
			.MEM_fwd2(mfwd2),
			.WB_fwd1(wfwd1),
			.WB_fwd2(wfwd2)
		);

	assign mem_fwd1_mux_out = mfwd1 ? dataMemOut_fwd_mux_out : RegA_val_ex;
	assign mem_fwd2_mux_out = mfwd2 ? dataMemOut_fwd_mux_out : RegB_val_ex;
	assign wb_fwd1_mux_out = wfwd1 ? wb_mux_out : mem_fwd1_mux_out;
	assign wb_fwd2_mux_out = wfwd2 ? wb_mux_out : mem_fwd2_mux_out;
	assign dataMemOut_fwd_mux_out = MemtoReg_m0 ? data_mem_out : alu_result_m0;

	//Branch Predictor
	branch_predictor branch_predictor_FSM(
			.clk(clk),
			.actual_branch_decision(actual_branch_decision),
			.branch_decode_sig(cont_mux_out[6]),
			.branch_mem_sig(Branch_m0),
			.in_addr(pc_id),
			.offset(imm_out),
			.branch_addr(branch_predictor_addr),
			.prediction(predict)
		);

	assign branch_predictor_mux_out = predict ? branch_predictor_addr : fence_mux_out;

	assign pc_mux0 = mistake_trigger ? pc_ex : branch_predictor_mux_out;

	wire[31:0] mem_regwb_mux_out; //TODO copy of wb_mux but in mem stage, move back and cleanup
	//A copy of the writeback mux, but in MEM stage //TODO move back and cleanup
	assign mem_regwb_mux_out = MemtoReg_m0 ? data_mem_out : mem_csrr_mux_out;

	//OR gate assignments, used for flushing
	assign decode_ctrl_mux_sel = pcsrc | mistake_trigger;
	assign inst_mux_sel = pcsrc | predict | mistake_trigger | Fence_signal;

	//Instruction Memory Connections
	assign inst_mem_in = pc_in;

	//Data Memory Connections
	assign data_mem_addr = alu_result_ex;
	assign data_mem_WrData = wb_fwd2_mux_out;
	assign data_mem_memwrite = MemWrite_gtd_ex;
	assign data_mem_memread = MemRead_gtd_ex;
	assign data_mem_sign_mask = dataMem_sign_mask_ex;
endmodule
