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
	wire [177:0]		id_ex_out;
	wire [154:0]		ex_mem_out;
	wire [116:0]		mem_wb_out;

	/*
	 *	Control signals
	 */
	wire			MemtoReg1;
	wire			RegWrite1;
	wire			MemWrite1;
	wire			MemRead1;
	wire			Branch1;
	wire			Jump1;
	wire			Jalr1;
	wire			ALUSrc1;
	wire			Lui1;
	wire			Auipc1;
	wire			Fence_signal;
	wire			CSRR_signal;
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
	wire [8:0]		ex_cont_mux_out;
	wire [31:0]		addr_adder_mux_out;
	wire [31:0]		alu_mux_out;
	wire [31:0]		addr_adder_sum;
	wire [6:0]		alu_ctl;
	wire			alu_branch_enable;
	wire [31:0]		alu_result;
	wire [31:0]		lui_result;
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

	assign pc_in = pcsrc ? ex_mem_out[72:41] : pc_mux0;


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
			.MemtoReg(MemtoReg1),
			.RegWrite(RegWrite1),
			.MemWrite(MemWrite1),
			.MemRead(MemRead1),
			.Branch(Branch1),
			.ALUSrc(ALUSrc1),
			.Jump(Jump1),
			.Jalr(Jalr1),
			.Lui(Lui1),
			.Auipc(Auipc1),
			.Fence(Fence_signal),
			.CSRR(CSRR_signal)
		);

	assign cont_mux_out = decode_ctrl_mux_sel ? 11'b0 : {Jalr1, ALUSrc1, Lui1, Auipc1, Branch1, MemRead1, MemWrite1, CSRR_signal, RegWrite1, MemtoReg1, Jump1};

	regfile register_files(
			.clk(clk),
			.rst_n_i(rst_n_i),
			.data_mem_stall(data_mem_stall),
			.write(ex_mem_out[2]),
			.wrAddr(ex_mem_out[142:138]),
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
			.write(mem_wb_out[3]), //TODO
			.wrAddr_CSR(mem_wb_out[116:105]),
			.wrVal_CSR(mem_wb_out[35:4]),
			.rdAddr_CSR(inst_mux_out[31:20]),
			.rdVal_CSR(rdValOut_CSR)
		);

	assign RegA_mux_out = CSRRI_signal ? {27'b0, instr_id[19:15]} : regA_out;
	assign RegB_mux_out = CSRR_signal ? rdValOut_CSR : regB_out;
	assign RegA_AddrFwdFlush_mux_out = CSRRI_signal ? 5'b0 : instr_id[19:15];
	assign RegB_AddrFwdFlush_mux_out = CSRR_signal ? 5'b0 : instr_id[24:20];

	assign CSRRI_signal = CSRR_signal & (instr_id[14]);

	//ID/EX Pipeline Register
	always @(posedge clk or negedge rst_n_i) begin
		if (!rst_n_i) begin
			aluCtl_sign_ex <= 1'b0;
		end
		else begin
			aluCtl_sign_ex <= aluCtl_sign_id;
		end
	end

	id_ex id_ex_reg(
			.clk(clk),
			.rst_n_i(rst_n_i),
			.data_mem_stall(data_mem_stall),
			.data_in({instr_id[31:20], RegB_AddrFwdFlush_mux_out, RegA_AddrFwdFlush_mux_out, instr_id[11:7], dataMem_sign_mask, alu_ctl, imm_out, RegB_mux_out, RegA_mux_out, pc_id, cont_mux_out[10:7], predict, cont_mux_out[6:0]}),
			.data_out(id_ex_out)
		);

	assign ex_cont_mux_out = pcsrc ? 9'b0 : id_ex_out[8:0];
	assign addr_adder_mux_out = id_ex_out[11] ? wb_fwd1_mux_out : id_ex_out[43:12];

	adder addr_adder(
			.input1(addr_adder_mux_out),
			.input2(id_ex_out[139:108]),
			.out(addr_adder_sum)
		);

	assign alu_mux_out = id_ex_out[10] ? id_ex_out[139:108] : wb_fwd2_mux_out;

	alu alu_main(
			.ALUctl(id_ex_out[146:140]),
			.A(wb_fwd1_mux_out),
			.B(alu_mux_out),
			.sign(aluCtl_sign_ex),
			.ALUOut(alu_result),
			.Branch_Enable(alu_branch_enable)
		);

	assign lui_result = id_ex_out[9] ? id_ex_out[139:108] : alu_result;

	//EX/MEM Pipeline Register
	ex_mem ex_mem_reg(
			.clk(clk),
			.rst_n_i(rst_n_i),
			.data_mem_stall(data_mem_stall),
			.data_in({id_ex_out[177:166], id_ex_out[155:151], wb_fwd2_mux_out, lui_result, alu_branch_enable, addr_adder_sum, id_ex_out[43:12], ex_cont_mux_out}),
			.data_out(ex_mem_out)
		);

	//Memory Access Stage
	branch_decision branch_decide(
			.Branch(ex_mem_out[6]),
			.Predicted(ex_mem_out[7]),
			.Branch_Enable(ex_mem_out[73]),
			.Jump(ex_mem_out[0]),
			.Mispredict(mistake_trigger),
			.Decision(actual_branch_decision),
			.Branch_Jump_Trigger(pcsrc)
		);

	assign auipc_mux_out = ex_mem_out[8] ? ex_mem_out[72:41] : ex_mem_out[105:74];
	assign mem_csrr_mux_out = ex_mem_out[3] ? ex_mem_out[137:106] : auipc_mux_out;

	//MEM/WB Pipeline Register
	mem_wb mem_wb_reg(
			.clk(clk),
			.rst_n_i(rst_n_i),
			.data_mem_stall(data_mem_stall),
			.data_in({ex_mem_out[154:143], ex_mem_out[142:138], data_mem_out, mem_csrr_mux_out, ex_mem_out[105:74], ex_mem_out[3:0]}),
			.data_out(mem_wb_out)
		);

	assign wb_mux_out = mem_wb_out[1] ? mem_wb_out[99:68] : mem_wb_out[67:36];
	assign reg_dat_mux_out = ex_mem_out[0] ? id_ex_out[43:12] : mem_regwb_mux_out;

	//Forwarding Unit
	ForwardingUnit forwarding_unit(
			.rs1(id_ex_out[160:156]),
			.rs2(id_ex_out[165:161]),
			.MEM_RegWriteAddr(ex_mem_out[142:138]),
			.WB_RegWriteAddr(mem_wb_out[104:100]),
			.MEM_RegWrite(ex_mem_out[2]),
			.WB_RegWrite(mem_wb_out[2]),
			.EX_CSRR_Addr(id_ex_out[177:166]),
			.MEM_CSRR_Addr(ex_mem_out[154:143]),
			.WB_CSRR_Addr(mem_wb_out[116:105]),
			.MEM_CSRR(ex_mem_out[3]),
			.WB_CSRR(mem_wb_out[3]),
			.MEM_fwd1(mfwd1),
			.MEM_fwd2(mfwd2),
			.WB_fwd1(wfwd1),
			.WB_fwd2(wfwd2)
		);

	assign mem_fwd1_mux_out = mfwd1 ? dataMemOut_fwd_mux_out : id_ex_out[75:44];
	assign mem_fwd2_mux_out = mfwd2 ? dataMemOut_fwd_mux_out : id_ex_out[107:76];
	assign wb_fwd1_mux_out = wfwd1 ? wb_mux_out : mem_fwd1_mux_out;
	assign wb_fwd2_mux_out = wfwd2 ? wb_mux_out : mem_fwd2_mux_out;
	assign dataMemOut_fwd_mux_out = ex_mem_out[1] ? data_mem_out : ex_mem_out[105:74];

	//Branch Predictor
	branch_predictor branch_predictor_FSM(
			.clk(clk),
			.actual_branch_decision(actual_branch_decision),
			.branch_decode_sig(cont_mux_out[6]),
			.branch_mem_sig(ex_mem_out[6]),
			.in_addr(pc_id),
			.offset(imm_out),
			.branch_addr(branch_predictor_addr),
			.prediction(predict)
		);

	assign branch_predictor_mux_out = predict ? branch_predictor_addr : fence_mux_out;

	assign pc_mux0 = mistake_trigger ? id_ex_out[43:12] : branch_predictor_mux_out;

	wire[31:0] mem_regwb_mux_out; //TODO copy of wb_mux but in mem stage, move back and cleanup
	//A copy of the writeback mux, but in MEM stage //TODO move back and cleanup
	assign mem_regwb_mux_out = ex_mem_out[1] ? data_mem_out : mem_csrr_mux_out;

	//OR gate assignments, used for flushing
	assign decode_ctrl_mux_sel = pcsrc | mistake_trigger;
	assign inst_mux_sel = pcsrc | predict | mistake_trigger | Fence_signal;

	//Instruction Memory Connections
	assign inst_mem_in = pc_out;

	//Data Memory Connections
	assign data_mem_addr = lui_result;
	assign data_mem_WrData = wb_fwd2_mux_out;
	assign data_mem_memwrite = ex_cont_mux_out[4];
	assign data_mem_memread = ex_cont_mux_out[5];
	assign data_mem_sign_mask = id_ex_out[150:147];
endmodule
