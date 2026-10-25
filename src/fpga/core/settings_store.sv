// Pocket loads this window before Reset Exit and reads it after Reset Enter.
// APF reset stops the CPU, but must not clear the published image or handshake.
module settings_store (
	input wire bridge_clk,
	input wire reset_n,
	input wire [31:0] bridge_addr,
	input wire bridge_rd,
	input wire bridge_wr,
	input wire [31:0] bridge_wr_data,
	output wire [31:0] bridge_rd_data,
	input wire slot_read,
	input wire [15:0] slot_read_id,
	input wire cpu_clk,
	input wire [4:0] cpu_addr,
	input wire [31:0] cpu_data,
	input wire cpu_wr,
	input wire cpu_commit,
	output reg [31:0] cpu_q,
	output wire [1:0] cpu_control,
	output reg [9:0] table_addr = 0,
	output reg table_wr = 0,
	output wire [31:0] table_data,
	input wire [31:0] table_q
);
	// Reads concurrent with writes to the same word are never consumed.
	(* ramstyle = "M10K, no_rw_check" *) reg [31:0] memory [0:63];
	reg request = 0, acknowledge = 0;
	(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *)
	reg request_meta = 0, request_sync = 0;
	(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *)
	reg ack_meta = 0, ack_sync = 0;
	reg read_bank = 0, read_locked = 0;
	reg [2:0] table_state = 0;
	wire selected = bridge_addr[31:7] == (32'h10000000 >> 7);
	wire [5:0] cpu_ram_addr = {cpu_wr ? ~ack_sync : ack_sync,cpu_addr};
	wire [5:0] bridge_ram_addr = {
		bridge_wr ? acknowledge : (read_locked ? read_bank : acknowledge),bridge_addr[6:2]};
	wire [31:0] incoming = {bridge_wr_data[7:0],bridge_wr_data[15:8],
		bridge_wr_data[23:16],bridge_wr_data[31:24]};
	reg [31:0] bridge_q;
	reg [31:0] bridge_response = 0;
	assign bridge_rd_data = bridge_response;
	assign cpu_control = {request != ack_sync,ack_sync};
	assign table_data = 32'd128;

	initial begin
		for (integer i = 0; i < 64; i = i + 1) memory[i] = 32'hffffffff;
	end

	always @(posedge cpu_clk) begin
		ack_meta <= acknowledge;
		ack_sync <= ack_meta;
		cpu_q <= memory[cpu_ram_addr];
		if (cpu_wr && request == ack_sync) memory[cpu_ram_addr] <= cpu_data;
		// Write a desired token, not "toggle on write": original MMIO strobes
		// can remain asserted for multiple CPU clocks.
		if (cpu_commit) request <= cpu_data[0];
	end

	always @(posedge bridge_clk) begin
		request_meta <= request;
		request_sync <= request_meta;
		bridge_q <= memory[bridge_ram_addr];
		// APF captures the PREVIOUS response before pulsing bridge_rd for
		// this address. Keep that response stable while the address changes.
		// In particular A+128 must flush the preceding in-range CRC word.
		if (bridge_rd)
			bridge_response <= selected ? {bridge_q[7:0],bridge_q[15:8],
				bridge_q[23:16],bridge_q[31:24]} : 32'd0;
		if (selected && bridge_wr) begin
			memory[bridge_ram_addr] <= incoming;
			read_locked <= 0;
		end
		if (reset_n) read_locked <= 0;
		else if (slot_read && slot_read_id == 16'h0400 && !read_locked) begin
			read_bank <= acknowledge;
			read_locked <= 1;
		end

		// APF's table contains 32 {ID,size} pairs. Publish the complete file
		// size even on first boot, when there was no settings.cfg on disk.
		table_wr <= 0;
		case (table_state)
			0: if (request_sync != acknowledge) begin
				acknowledge <= request_sync;
				table_addr <= 0;
				table_state <= 1;
			end
			1: table_state <= 2;
			2: table_state <= 3;
			3: if (table_q[15:0] == 16'h0400) begin
				table_addr <= table_addr + 1'b1;
				table_wr <= 1;
				table_state <= 4;
			end else if (table_addr == 62) table_state <= 0;
			else begin
				table_addr <= table_addr + 2'd2;
				table_state <= 1;
			end
			4: table_state <= 0;
			default: table_state <= 0;
		endcase
	end
endmodule
