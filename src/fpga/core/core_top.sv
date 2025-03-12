//
// User core top-level
//
// Instantiated by the real top-level: apf_top
//

`default_nettype none

module core_top (

//
// physical connections
//

///////////////////////////////////////////////////
// clock inputs 74.25mhz. not phase aligned, so treat these domains as asynchronous

input   wire            clk_74a, // mainclk1
input   wire            clk_74b, // mainclk1 

///////////////////////////////////////////////////
// cartridge interface
// switches between 3.3v and 5v mechanically
// output enable for multibit translators controlled by pic32

// GBA AD[15:8]
inout   wire    [7:0]   cart_tran_bank2,
output  wire            cart_tran_bank2_dir,

// GBA AD[7:0]
inout   wire    [7:0]   cart_tran_bank3,
output  wire            cart_tran_bank3_dir,

// GBA A[23:16]
inout   wire    [7:0]   cart_tran_bank1,
output  wire            cart_tran_bank1_dir,

// GBA [7] PHI#
// GBA [6] WR#
// GBA [5] RD#
// GBA [4] CS1#/CS#
//     [3:0] unwired
inout   wire    [7:4]   cart_tran_bank0,
output  wire            cart_tran_bank0_dir,

// GBA CS2#/RES#
inout   wire            cart_tran_pin30,
output  wire            cart_tran_pin30_dir,
// when GBC cart is inserted, this signal when low or weak will pull GBC /RES low with a special circuit
// the goal is that when unconfigured, the FPGA weak pullups won't interfere.
// thus, if GBC cart is inserted, FPGA must drive this high in order to let the level translators
// and general IO drive this pin.
output  wire            cart_pin30_pwroff_reset,

// GBA IRQ/DRQ
inout   wire            cart_tran_pin31,
output  wire            cart_tran_pin31_dir,

// infrared
input   wire            port_ir_rx,
output  wire            port_ir_tx,
output  wire            port_ir_rx_disable, 

// GBA link port
inout   wire            port_tran_si,
output  wire            port_tran_si_dir,
inout   wire            port_tran_so,
output  wire            port_tran_so_dir,
inout   wire            port_tran_sck,
output  wire            port_tran_sck_dir,
inout   wire            port_tran_sd,
output  wire            port_tran_sd_dir,
 
///////////////////////////////////////////////////
// cellular psram 0 and 1, two chips (64mbit x2 dual die per chip)

output  wire    [21:16] cram0_a,
inout   wire    [15:0]  cram0_dq,
input   wire            cram0_wait,
output  wire            cram0_clk,
output  wire            cram0_adv_n,
output  wire            cram0_cre,
output  wire            cram0_ce0_n,
output  wire            cram0_ce1_n,
output  wire            cram0_oe_n,
output  wire            cram0_we_n,
output  wire            cram0_ub_n,
output  wire            cram0_lb_n,

output  wire    [21:16] cram1_a,
inout   wire    [15:0]  cram1_dq,
input   wire            cram1_wait,
output  wire            cram1_clk,
output  wire            cram1_adv_n,
output  wire            cram1_cre,
output  wire            cram1_ce0_n,
output  wire            cram1_ce1_n,
output  wire            cram1_oe_n,
output  wire            cram1_we_n,
output  wire            cram1_ub_n,
output  wire            cram1_lb_n,

///////////////////////////////////////////////////
// sdram, 512mbit 16bit

output  wire    [12:0]  dram_a,
output  wire    [1:0]   dram_ba,
inout   wire    [15:0]  dram_dq,
output  wire    [1:0]   dram_dqm,
output  wire            dram_clk,
output  wire            dram_cke,
output  wire            dram_ras_n,
output  wire            dram_cas_n,
output  wire            dram_we_n,

///////////////////////////////////////////////////
// sram, 1mbit 16bit

output  wire    [16:0]  sram_a,
inout   wire    [15:0]  sram_dq,
output  wire            sram_oe_n,
output  wire            sram_we_n,
output  wire            sram_ub_n,
output  wire            sram_lb_n,

///////////////////////////////////////////////////
// vblank driven by dock for sync in a certain mode

input   wire            vblank,

///////////////////////////////////////////////////
// i/o to 6515D breakout usb uart

output  wire            dbg_tx,
input   wire            dbg_rx,

///////////////////////////////////////////////////
// i/o pads near jtag connector user can solder to

output  wire            user1,
input   wire            user2,

///////////////////////////////////////////////////
// RFU internal i2c bus 

inout   wire            aux_sda,
output  wire            aux_scl,

///////////////////////////////////////////////////
// RFU, do not use
output  wire            vpll_feed,


//
// logical connections
//

///////////////////////////////////////////////////
// video, audio output to scaler
output  wire    [23:0]  video_rgb,
output  wire            video_rgb_clock,
output  wire            video_rgb_clock_90,
output  wire            video_de,
output  wire            video_skip,
output  wire            video_vs,
output  wire            video_hs,
    
output  wire            audio_mclk,
input   wire            audio_adc,
output  wire            audio_dac,
output  wire            audio_lrck,

///////////////////////////////////////////////////
// bridge bus connection
// synchronous to clk_74a
output  wire            bridge_endian_little,
input   wire    [31:0]  bridge_addr,
input   wire            bridge_rd,
output  reg     [31:0]  bridge_rd_data,
input   wire            bridge_wr,
input   wire    [31:0]  bridge_wr_data,

///////////////////////////////////////////////////
// controller data
// 
// key bitmap:
//   [0]    dpad_up
//   [1]    dpad_down
//   [2]    dpad_left
//   [3]    dpad_right
//   [4]    face_a
//   [5]    face_b
//   [6]    face_x
//   [7]    face_y
//   [8]    trig_l1
//   [9]    trig_r1
//   [10]   trig_l2
//   [11]   trig_r2
//   [12]   trig_l3
//   [13]   trig_r3
//   [14]   face_select
//   [15]   face_start
//   [31:28] type
// joy values - unsigned
//   [ 7: 0] lstick_x
//   [15: 8] lstick_y
//   [23:16] rstick_x
//   [31:24] rstick_y
// trigger values - unsigned
//   [ 7: 0] ltrig
//   [15: 8] rtrig
//
input   wire    [31:0]  cont1_key,
input   wire    [31:0]  cont2_key,
input   wire    [31:0]  cont3_key,
input   wire    [31:0]  cont4_key,
input   wire    [31:0]  cont1_joy,
input   wire    [31:0]  cont2_joy,
input   wire    [31:0]  cont3_joy,
input   wire    [31:0]  cont4_joy,
input   wire    [15:0]  cont1_trig,
input   wire    [15:0]  cont2_trig,
input   wire    [15:0]  cont3_trig,
input   wire    [15:0]  cont4_trig
    
);

// not using the IR port, so turn off both the LED, and
// disable the receive circuit to save power
assign port_ir_tx = 0;
assign port_ir_rx_disable = 1;

// bridge endianness
assign bridge_endian_little = 0;

// cart is unused, so set all level translators accordingly
// directions are 0:IN, 1:OUT
assign cart_tran_bank3 = 8'hzz;//{4'b0000,debugclk,ce_7mp,pix_clk,pix_clk_90};//8'hzz;
assign cart_tran_bank3_dir = 1'b0;
assign cart_tran_bank2 = 8'hzz;
assign cart_tran_bank2_dir = 1'b0;
assign cart_tran_bank1 = 8'hzz;
assign cart_tran_bank1_dir = 1'b0;
assign cart_tran_bank0 = 4'hf;
assign cart_tran_bank0_dir = 1'b1;
assign cart_tran_pin30 = 1'b0;      // reset or cs2, we let the hw control it by itself
assign cart_tran_pin30_dir = 1'bz;
assign cart_pin30_pwroff_reset = 1'b0;  // hardware can control this
assign cart_tran_pin31 = 1'bz;      // input
assign cart_tran_pin31_dir = 1'b0;  // input

// link port is unused, set to input only to be safe
// each bit may be bidirectional in some applications
assign port_tran_so = 1'bz;
assign port_tran_so_dir = 1'b0;     // SO is output only
assign port_tran_si = 1'bz;
assign port_tran_si_dir = 1'b0;     // SI is input only
assign port_tran_sck = 1'bz;
assign port_tran_sck_dir = 1'b0;    // clock direction can change
assign port_tran_sd = 1'bz;
assign port_tran_sd_dir = 1'b0;     // SD is input and not used

// tie off the rest of the pins we are not using
/*assign cram0_a = 'h0;
assign cram0_dq = {16{1'bZ}};
assign cram0_clk = 0;
assign cram0_adv_n = 1;
assign cram0_cre = 0;
assign cram0_ce0_n = 1;
assign cram0_ce1_n = 1;
assign cram0_oe_n = 1;
assign cram0_we_n = 1;
assign cram0_ub_n = 1;
assign cram0_lb_n = 1;
*/
assign cram1_a = 'h0;
assign cram1_dq = {16{1'bZ}};
assign cram1_clk = 0;
assign cram1_adv_n = 1;
assign cram1_cre = 0;
assign cram1_ce0_n = 1;
assign cram1_ce1_n = 1;
assign cram1_oe_n = 1;
assign cram1_we_n = 1;
assign cram1_ub_n = 1;
assign cram1_lb_n = 1;

/*assign dram_a = 'h0;
assign dram_ba = 'h0;
assign dram_dq = {16{1'bZ}};
assign dram_dqm = 'h0;
assign dram_clk = 'h0;
assign dram_cke = 'h0;
assign dram_ras_n = 'h1;
assign dram_cas_n = 'h1;
assign dram_we_n = 'h1;
*/
assign sram_a = 'h0;
assign sram_dq = {16{1'bZ}};
assign sram_oe_n  = 1;
assign sram_we_n  = 1;
assign sram_ub_n  = 1;
assign sram_lb_n  = 1;

assign dbg_tx = 1'bZ;
assign user1 = 1'bZ;
assign aux_scl = 1'bZ;
assign vpll_feed = 1'bZ;



localparam ARCH_ZX48  = 5'b011_00; // ZX 48
localparam ARCH_ZX128 = 5'b000_01; // ZX 128/+2
localparam ARCH_ZX3   = 5'b100_01; // ZX 128 +3
localparam ARCH_P48   = 5'b011_10; // Pentagon 48
localparam ARCH_P128  = 5'b000_10; // Pentagon 128
localparam ARCH_P1024 = 5'b001_10; // Pentagon 1024

localparam CONF_BDI   = "(BDI)";
localparam CONF_PLUSD = "(+D) ";
localparam CONF_PLUS3 = "(+3) ";


////////////////////   CLOCKS   ///////////////////

wire    pll_core_locked;
wire    pll_core_locked_s;
wire	  locked=pll_core_locked;
wire	  clk_sys, clk_56;
wire	  clk_aud = clk_56;
synch_3 s01(pll_core_locked, pll_core_locked_s, clk_74a);

mf_pllbase mp1 (
    .refclk         ( clk_74a ),
    .rst            ( 0 ),
    
    .outclk_0       ( clk_sys ),
    .outclk_1       ( clk_56),
    
    .locked         ( pll_core_locked )
);


reg  ce_7mp;
reg  ce_7mn;
reg  pix_clk;
reg  pix_clk_90;

reg  pause;
reg  cpu_en = 1;
reg  ce_cpu_tp;
reg  ce_cpu_tn;
reg  ce_tape;
reg  ce_wd1793;
reg  ce_u765;
reg  ce_spi;

wire ce_cpu_p = cpu_en & cpu_p;
wire ce_cpu_n = cpu_en & cpu_n;

wire cpu_p = ~&turbo ? ce_cpu_tp : ce_cpu_sp;
wire cpu_n = ~&turbo ? ce_cpu_tn : ce_cpu_sn;

always @(posedge clk_sys) begin
	reg [5:0] counter = 0;

	counter   <=  counter + 1'd1;

	ce_7mp    <= !counter[3] & !counter[2:0];
	ce_7mn    <=  counter[3] & !counter[2:0];
	
	if (!counter[3] & !counter[2:0]) pix_clk<=1;
	if (counter[3] & !counter[2:0]) pix_clk<=0;
	
	if (!counter[3] & counter[2] & !counter[1:0]) pix_clk_90<=1;
	if (counter[3] & counter[2] & !counter[1:0]) pix_clk_90<=0;

	// split ce for relaxed fitting
	ce_cpu_tp <= !(counter & turbo);
	ce_tape   <= !(counter & turbo) & cpu_en;
	ce_wd1793 <= !(counter & turbo) & cpu_en;
	ce_u765   <= !(counter & turbo) & cpu_en;

	ce_cpu_tn <= !((counter & turbo) ^ turbo ^ turbo[4:1]);

	//ce_spi    <= vsd_sel | ((status[33] | !counter[1]) & !counter[0]);
	ce_spi    <= ((status[33] | !counter[1]) & !counter[0]);
end

wire [4:0] turbo_req;
always_comb begin
	casex({tape_active & ~status[6], status[24:22]})
		 'b1XXX: turbo_req = 5'b00001;
		 'b0001: turbo_req = 5'b01111;
		 'b0010: turbo_req = 5'b00111;
		 'b0011: turbo_req = 5'b00011;
		 'b0100: turbo_req = 5'b00001;
		default: turbo_req = 5'b11111;
	endcase
end

reg [2:0] speed_req;
reg       speed_set;
always @(posedge clk_sys) begin
	reg [9:4] old_Fn;
	old_Fn <= Fn[9:4];

	if(reset) pause <= 0;

	speed_set <= 0;
	if(!mod) begin
		if(~old_Fn[4] & Fn[4]) {speed_set,speed_req} <= 4'b1_000;
		if(~old_Fn[5] & Fn[5]) {speed_set,speed_req} <= 4'b1_001;
		if(~old_Fn[6] & Fn[6]) {speed_set,speed_req} <= 4'b1_010;
		if(~old_Fn[7] & Fn[7]) {speed_set,speed_req} <= 4'b1_011;
		if(~old_Fn[8] & Fn[8]) {speed_set,speed_req} <= 4'b1_100;
		if(~old_Fn[9] & Fn[9]) pause <= ~pause;
	end
end

reg [4:0] turbo = 5'b11111;
always @(posedge clk_sys) begin
	reg [1:0] timeout;

	if(cpu_n) begin
		if(timeout) timeout <= timeout + 1'd1;
		if(turbo != turbo_req) begin
			cpu_en  <= 0;
			timeout <= 1;
			turbo   <= turbo_req;
		end else if(!cpu_en & !timeout & ram_ready) begin
			cpu_en  <= ~pause;
		end else if(!turbo[4:2] & !ram_ready) begin // SDRAM wait for 28MHz/56MHz turbo
			cpu_en  <= 0;
		end else if(!turbo[4:3] & !ram_ready & tape_active) begin // SDRAM wait for TAPE load on 14MHz
			cpu_en  <= 0;
		end else if(cpu_en & pause) begin
			cpu_en  <= 0;
		end
	end
end


reg  [2:0] cur_mode = 0;
wire       need_apply = status[12:10] != cur_mode;

always @(posedge clk_sys) begin
	if(reset) cur_mode <= status[12:10];
end


///////////////////   CPU   ///////////////////
wire [15:0] addr;
wire  [7:0] cpu_dout;
wire        nM1;
wire        nMREQ;
wire        nIORQ;
wire        nRD;
wire        nWR;
wire        nRFSH;
wire        nBUSACK;
wire        nINT;
wire        nBUSRQ = ~ioctl_download;
wire        reset  = status[0] | cold_reset | warm_reset | shdw_reset | Fn[10] | mmc_reset;
//wire        reset  = status[0] | cold_reset | warm_reset | shdw_reset | Fn[10];// | ~rom_loaded;

wire        cold_reset =((mod[2:1] == 1) & Fn[11]) | init_reset | arch_reset | snap_reset | mmc_reset;
//wire        cold_reset =((mod[2:1] == 1) & Fn[11]) | init_reset | arch_reset | snap_reset ;// | ~rom_loaded;
wire        warm_reset = (mod[2:1] == 2) & Fn[11];
wire        shdw_reset = (mod[2:1] == 3) & Fn[11] & ~plus3;

wire        io_wr = ~nIORQ & ~nWR & nM1;
wire        io_rd = ~nIORQ & ~nRD & nM1;
wire        m1    = ~nM1 & ~nMREQ;

wire[211:0]	cpu_reg;  // IFF2, IFF1, IM, IY, HL', DE', BC', IX, HL, DE, BC, PC, SP, R, I, F', A', F, A
wire [15:0] reg_DE  = cpu_reg[111:96];
wire  [7:0] reg_A   = cpu_reg[7:0];

reg pause_z80_req;
reg pause_z80;
initial begin
	pause_z80<=0;
	pause_z80_req<=0;
end
always @(posedge ce_cpu_p) begin
	pause_z80<=pause_z80_req;	//set pause z80 when cpu clk is high and req is asserted
end

T80pa cpu
(
	.RESET_n(~reset),
	.CLK(clk_sys),
	.CEN_p(ce_cpu_p | pause_z80),
	.CEN_n(ce_cpu_n & ~pause_z80),
	.WAIT_n(mmc_ready),
	//.WAIT_n(1'b1),
	.INT_n(nINT),
	.NMI_n(~NMI),
	.BUSRQ_n(nBUSRQ),
	.M1_n(nM1),
	.MREQ_n(nMREQ),
	.IORQ_n(nIORQ),
	.RD_n(nRD),
	.WR_n(nWR),
	.RFSH_n(nRFSH),
	.HALT_n(1),
	.BUSAK_n(nBUSACK),
	.A(addr),
	.DO(cpu_dout),
	.DI(cpu_din),
	.REG(cpu_reg),
	.DIR(snap_REG),
	.DIRSet(snap_REGSet)
);

wire [7:0] cpu_din =  
		~nMREQ   ? (tape_dout_en ? tape_dout : ram_dout)      :
		~io_rd   ? port_ff                                    :
		fdc_sel  ? fdc_dout                                   :
		mf3_port ? (&addr[14:13] ? page_reg : page_reg_plus3) :
		mmc_sel  ? mmc_dout                                   :
		kemp_sel ? kemp_dout                                  :
		portBF   ? {page_scr_copy, 7'b1111111}                :
		gs_sel   ? gs_dout                                    :
		psg_rd   ? psg_dout                                   :
		ulap_sel ? ulap_dout                                  :
		~addr[0] ? {1'b1, ula_tape_in, 1'b1, kbd_dout}        :
					  port_ff;
					  
//assign status[0]=~reset_n;
reg init_reset = 1;
always @(posedge clk_sys) begin
	reg old_rst = 0;
	old_rst <= status[0];
	if(old_rst & ~status[0]) init_reset <= 0;	
end

reg NMI;
always @(posedge clk_sys) begin
	reg old_F11;

	old_F11 <= Fn[11];

	if(reset | ~Fn[11] | (m1 & (addr == 'h66))) NMI <= 0;
	else if(~old_F11 & Fn[11] & (mod[2:1] == 0)) NMI <= 1;
end


//////////////////   MEMORY   //////////////////
reg  [24:0] ram_addr;
reg   [7:0] ram_din;
reg         ram_we;
reg         ram_rd;
wire  [7:0] ram_dout;
wire        ram_ready;

reg [24:0] load_addr;
//always @(posedge clk_sys) load_addr <= ioctl_addr + (ioctl_index[4:0] ? 25'h400000 : ioctl_index[7:6] ? 25'h14C000 : 25'h150000);
//always @(posedge clk_sys) load_addr <= ioctl_addr + (ioctl_id[9] ? 25'h150000 : ioctl_id[10] ? 25'h14C000 : 25'h400000);
always @(posedge clk_sys) load_addr <= ioctl_addr + 25'h400000;

//14c000 = ESXDOS memory
//150000 = ROM load location (196,608/30000 bytes) 
//400000 = tape buffer


reg load;
always @(posedge clk_sys) load <= (reset | ~nBUSACK) & ~nBUSRQ;

//wire rom_load=(bridge_wr && bridge_addr[20:16] == 5'h15);
//wire rom_load=(bridge_addr[24:21]==0) & bridge_addr[20] & ~bridge_addr[19];
//wire rom_load=(ioctl_id==16'h200) & ~rom_loaded;
wire rom_load=ioctl_download && ioctl_id == 16'h200;

//reg [7:0] bridge_byte_in;

//reg [25:0] requested_addr;
		
always_comb begin

	/*case (bridge_addr[1:0])
		'b00: bridge_byte_in=bridge_wr_data[7:0];
		'b01: bridge_byte_in=bridge_wr_data[15:8];
		'b10: bridge_byte_in=bridge_wr_data[23:16];
		'b11: bridge_byte_in=bridge_wr_data[31:24];
	endcase*/
	
	/*case (bridge_addr[1:0])
		'b11: bridge_byte_in=bridge_wr_data[7:0];
		'b10: bridge_byte_in=bridge_wr_data[15:8];
		'b01: bridge_byte_in=bridge_wr_data[23:16];
		'b00: bridge_byte_in=bridge_wr_data[31:24];
	endcase*/

	casex({rom_load,snap_reset, load, tape_req, mmc_ram_en, page_special, addr[15:14]})			
		//'b1_XXXX_X_XX: ram_addr = (ioctl_size_req=='h34000)?25'h14C000+ioctl_addr[24:0]:25'h150000+ioctl_addr[24:0];
		'b1_XXXX_X_XX: ram_addr = 25'h14C000+ioctl_addr[24:0];
		'b0_1XXX_X_XX: ram_addr = snap_addr;
		'b0_01XX_X_XX: ram_addr = load_addr;
		'b0_001X_X_XX: ram_addr = tape_addr;
		'b0_0001_X_XX: ram_addr = { 4'b1000, mmc_ram_bank,                                     addr[12:0]};		
		'b0_0000_0_00: ram_addr = { 3'b101,  page_rom,                                         addr[13:0]}; //ROM
		'b0_0000_0_01: ram_addr = { 4'b0000, 3'd5,                                             addr[13:0]}; //Non-special page modes
		'b0_0000_0_10: ram_addr = { 4'b0000, 3'd2,                                             addr[13:0]};
		'b0_0000_0_11: ram_addr = { 1'b0,    page_ram,                                         addr[13:0]};
		'b0_0000_1_00: ram_addr = { 4'b0000, |page_reg_plus3[2:1],                      2'b00, addr[13:0]}; //Special page modes
		'b0_0000_1_01: ram_addr = { 4'b0000, |page_reg_plus3[2:1], &page_reg_plus3[2:1], 1'b1, addr[13:0]};
		'b0_0000_1_10: ram_addr = { 4'b0000, |page_reg_plus3[2:1],                      2'b10, addr[13:0]};
		'b0_0000_1_11: ram_addr = { 4'b0000, ~page_reg_plus3[2] & page_reg_plus3[1],    2'b11, addr[13:0]};
	endcase

	casex({rom_load,snap_reset, load, tape_req})
		'b1_XXX: ram_din = ioctl_dout;		
		'b0_1XX: ram_din = snap_data;
		'b0_01X: ram_din = ioctl_dout;
		'b0_001: ram_din = 0;
		'b0_000: ram_din = cpu_dout;
	endcase

	casex({rom_load,load, tape_req})
		'b1_XX: ram_rd = 0;
		'b0_1X: ram_rd = 0;
		'b0_01: ram_rd = ~nMREQ;
		'b0_00: ram_rd = ~nMREQ & ~nRD;		
	endcase

	casex({rom_load,snap_reset, load, tape_req})
		'b1_XXX: ram_we = ioctl_wr;
		'b0_1XX: ram_we = snap_wr;
		'b0_01X: ram_we = ioctl_wr;
		'b0_001: ram_we = 0;
		'b0_000: ram_we = (mmc_ram_en | page_special | addr[15] | addr[14] | ((plusd_mem | mf128_mem) & addr[13])) & ~nMREQ & ~nWR;				
	endcase
end

wire dram_cs;
sdram ram
(
	.init(~pll_core_locked),
	.clk(clk_sys),
	
   .SDRAM_nCS(dram_cs), 
	.SDRAM_A(dram_a),
	.SDRAM_BA(dram_ba),
	.SDRAM_DQ(dram_dq),
	.SDRAM_DQML(dram_dqm[0]),
	.SDRAM_DQMH(dram_dqm[1]),
	.SDRAM_CLK(dram_clk),
	.SDRAM_CKE(dram_cke),
	.SDRAM_nRAS(dram_ras_n),
	.SDRAM_nCAS(dram_cas_n),
	.SDRAM_nWE(dram_we_n),

	
	
	
	.dout(ram_dout),
	.din (ram_din),
	.addr(ram_addr),
	.we(ram_we),
	.rd(ram_rd),
	.ready(ram_ready)
);

wire vram_we = (ram_addr[24:16] == 1) & ram_addr[14];
dpram #(.ADDRWIDTH(15)) vram
(
	.clock(clk_sys),

	.address_a({ram_addr[15], ram_addr[13:0]}),
	.data_a(ram_din),
	.wren_a(ram_we & vram_we),

	.address_b(vram_addr),
	.q_b(vram_dout)
);

reg        zx48;
reg        p1024;
reg        pf1024;
reg        plus3;
reg        page_scr_copy;
reg        shadow_rom;
reg  [7:0] page_reg;
reg  [7:0] page_reg_plus3;
reg  [7:0] page_reg_p1024;
wire       page_disable = zx48 | (~p1024 & page_reg[5]) | (p1024 & page_reg_p1024[2] & page_reg[5]);
wire       page_scr     = page_reg[3];
wire [5:0] page_ram     = {page_128k, page_reg[2:0]};
wire       page_write   = ~addr[15] & ~addr[1] & (addr[14] | ~plus3) & ~page_disable; //7ffd
wire       page_write_plus3 = ~addr[1] & addr[12] & ~addr[13] & ~addr[14] & ~addr[15] & plus3 & ~page_disable; //1ffd
wire       page_special = page_reg_plus3[0];
wire       motor_plus3 = page_reg_plus3[3];
wire       page_p1024 = addr[15] & addr[14] & addr[13] & ~addr[12] & ~addr[3]; //eff7
reg  [2:0] page_128k;

reg  [3:0] page_rom;
wire       active_48_rom = zx48 | (page_reg[4] & ~plus3) | (plus3 & page_reg[4] & page_reg_plus3[2] & ~page_special);

always_comb begin
	casex({mmc_rom_en, shadow_rom, trdos_en, plusd_mem, mf128_mem, plus3})	
		'b1XXXXX: page_rom <=   4'b0011; //esxdos
		'b01XXXX: page_rom <=   4'b0100; //shadow
		'b001XXX: page_rom <=   4'b0101; //trdos
		'b0001XX: page_rom <=   4'b1100; //plusd
		'b00001X: page_rom <= { 2'b11, plus3, ~plus3 }; //MF128/+3
		'b000001: page_rom <= { 2'b10, page_reg_plus3[2], page_reg[4] }; //+3
		'b000000: page_rom <= { zx48, 2'b11, zx48 | page_reg[4] }; //up to +2
	endcase
end

always @(posedge clk_sys) begin
	reg old_wr, old_m1, old_reset;
	reg [2:0] rmod;

	old_wr <= io_wr;
	old_m1 <= m1;
	
	old_reset <= reset;
	if(~old_reset & reset) rmod <= mod;

	if(reset) begin
		page_scr_copy <= 0;
		page_reg    <= 0;
		page_reg_plus3 <= 0; 
		page_reg_p1024 <= 0;
		page_128k   <= 0;
		page_reg[4] <= Fn[10];
		page_reg_plus3[2] <= Fn[10];
		shadow_rom <= shdw_reset & ~plusd_en;		
		if(Fn[10] && (rmod == 1)) begin
			p1024  <= 0;
			pf1024 <= 0;
			zx48   <= ~plus3;
		end else begin
			p1024 <= (status[12:10] == 1);
			pf1024<= (status[12:10] == 2);
			zx48  <= (status[12:10] == 3);
			plus3 <= (status[12:10] == 4);
		end
	end else begin
		if(snap_REGSet) begin
			if((snap_hw == ARCH_ZX128) || (snap_hw == ARCH_P128) || (snap_hw == ARCH_ZX3)) page_reg <= snap_7ffd;
			if(snap_hw == ARCH_ZX3) page_reg_plus3 <= snap_1ffd;
		end
		else begin
			if(m1 && ~old_m1 && addr[15:14]) shadow_rom <= 0;
			if(m1 && ~old_m1 && ~plusd_en && ~mod[0] && (addr == 'h66) && ~plus3) shadow_rom <= 1; 			
			

			if(io_wr & ~old_wr) begin
				if(page_write) begin
					page_reg  <= cpu_dout;
					if(p1024 & ~page_reg_p1024[2]) page_128k[2:0] <= { cpu_dout[5], cpu_dout[7:6] };
					if(~plusd_mem) page_scr_copy <= cpu_dout[3];
				end else if (page_write_plus3) begin
					page_reg_plus3 <= cpu_dout; 
				end
				if(pf1024 & (addr == 'hDFFD)) page_128k <= cpu_dout[2:0];
				if(p1024 & page_p1024) page_reg_p1024 <= cpu_dout;
			end
		end
	end
end


////////////////////  ULA PORT  ///////////////////
reg [2:0] border_color;
reg       ear_out;
reg       mic_out;

always @(posedge clk_sys) begin
	if(reset) {ear_out, mic_out} <= 2'b00;
	else if(io_wr & ~addr[0]) begin
		border_color <= cpu_dout[2:0];
		ear_out <= cpu_dout[4]; 
		mic_out <= cpu_dout[3];
	end
	
	if(snap_REGSet) border_color <= snap_border;
end


////////////////////   AUDIO   ///////////////////
wire  [7:0] psg_dout;
wire [11:0] ts_l, ts_r;
wire        psg_sel = /*addr[0] &*/ addr[15] & ~addr[1];
wire        psg_we  = psg_sel & ~nIORQ & ~nWR & nM1;
wire        psg_rd  = psg_sel & addr[14];
reg         psg_reset;
reg         psg_active;

wire        aud_reset = reset | psg_reset;

reg  ce_ym;  //3.5MHz
always @(posedge clk_aud) begin
	reg [3:0] counter = 0;
	reg p1,p2,p3;
	
	p1 <= pause;
	p2 <= p1;
	p3 <= p2;

	counter <=  counter + 1'd1;
	ce_ym   <= !counter & ~p3;
end

// Turbosound FM: dual YM2203 chips
turbosound turbosound
(
	.RESET(aud_reset),
	.CLK(clk_aud),
	.CE(ce_ym),
	.BDIR(psg_we),
	.BC(addr[14]),
	.DI(cpu_dout),
	.DO(psg_dout),

	.ENABLE(~status[39]),
	.PSG_MIX(status[40]),
	.PSG_TYPE(status[41]),

	.CHANNEL_L(ts_l),
	.CHANNEL_R(ts_r)
);

reg  ce_saa;  //8MHz
always @(posedge clk_aud) begin
	reg [3:0] counter = 0;

	counter <=  counter + 1'd1;
	if(counter == 6) counter <= 0;
	
	ce_saa <= !counter;
end

wire [7:0] saa_l;
wire [7:0] saa_r;

saa1099 saa1099
(
	.clk_sys(clk_aud),
	.ce(ce_saa),
	.rst_n(~aud_reset),
	.cs_n((addr[7:0] != 255) | nIORQ | tmx_avail),
	.a0(addr[8]),
	.wr_n(nWR),
	.din(cpu_dout),
	.out_l(saa_l),
	.out_r(saa_r)
);

wire [7:0] gs_dout;
wire [14:0] gs_l, gs_r;

reg ce_gs = 0;
always @(posedge clk_aud) ce_gs <= ~ce_gs;

gs #("core/gs105b.mif") gs
(
	.RESET(aud_reset),
	.CLK(clk_aud),
	.CE(ce_gs),

	.A(addr[3]),
	.DI(cpu_dout),
	.DO(gs_dout),
	.CS_n(~nM1 | nIORQ | ~gs_sel),
	.WR_n(nWR),
	.RD_n(nRD),

	.MEM_ADDR(gs_mem_addr),
	.MEM_DI(gs_mem_din),
	.MEM_DO(gs_mem_dout | gs_mem_mask),
	.MEM_RD(gs_mem_rd),
	.MEM_WR(gs_mem_wr),
	.MEM_WAIT(~gs_mem_ready),

	.OUTL(gs_l),
	.OUTR(gs_r)
);

wire [20:0] gs_mem_addr;
wire  [7:0] gs_mem_dout;
wire  [7:0] gs_mem_din;
wire        gs_mem_rd;
wire        gs_mem_wr;
wire        gs_mem_busy;
wire        gs_mem_ready=~gs_mem_busy;
reg   [7:0] gs_mem_mask;

always_comb begin
	gs_mem_mask = 0;
	case(status[21:20])
		0: if(gs_mem_addr[20:19]) gs_mem_mask = 8'hFF;
		1: if(gs_mem_addr[20])    gs_mem_mask = 8'hFF;
	 2,3:                        gs_mem_mask = 0;
	endcase
end

/*assign DDRAM_CLK = clk_aud;
ddram ddram
(
	.*,

	.addr(gs_mem_addr),
	.dout(gs_mem_dout),
	.din(gs_mem_din),
	.we(gs_mem_wr),
	.rd(gs_mem_rd),
	.ready(gs_mem_ready)
);*/


/*assign	cram0_a[21:16]={1'b0,gs_mem_addr[20:16]};
assign	gs_mem_dout=cram0_dq[7:0];
assign	cram0_dq=gs_mem_wr?{gs_mem_din,gs_mem_din}:gs_mem_rd?{16{1'bZ}}:gs_mem_addr[15:0];
assign	cram0_ub_n=0;
assign	cram0_lb_n=0;
assign	cram0_we_n=~gs_mem_wr;
assign	cram0_oe_n=~gs_mem_rd;
assign	cram0_ce0_n=0;
assign	cram0_ce1_n=0;
assign	cram0_cre=0;
assign	cram0_clk = 0;
assign	cram0_adv_n = 1;
*/
  // Generate a clock enable signal in the 32MHz domain that is aligned with
  // the corresponding clk_8mhz_1mhz_ph[12]_en signals in the 8MHz domain.
  /*reg [3:0] clk_32mhz_cntr;
  wire clk_32mhz_1mhz_ph12_en;
  assign clk_32mhz_1mhz_ph12_en = clk_32mhz_cntr[3:0] == 0;
  always @(posedge clk_32mhz) begin
    if (rst) clk_32mhz_cntr <= 10; // XXX: This is a bit arbitrary but BAD: 13,0,15 GOOD: 10
    else clk_32mhz_cntr <= clk_32mhz_cntr + 1;
  end*/

  // Then use it to synchronize transactions of the PSRAM controller to the two
  // phases of the C64 core. Since clk_8mhz and clk_32mhz are 'related
  // clocks' the tools should take care of the rest (I hope).
  /*wire psram_wen;
  assign psram_wen = clk_32mhz_1mhz_ph12_en & (ext_rom_cart_we | c64_cart_we);
  wire psram_ren;
  assign psram_ren = clk_32mhz_1mhz_ph12_en & ~(ext_rom_cart_we | c64_cart_we);*/

  psram #(
    .CLOCK_SPEED(56)
  ) u_psram (
    .clk(clk_aud),
    .bank_sel(1'b0),
    .addr({1'b0,gs_mem_addr}),
    .write_en(gs_mem_wr),
    .data_in({gs_mem_din,gs_mem_din}),
    .write_high_byte(1'b0),
    .write_low_byte(1'b1),

    .read_en(gs_mem_rd),
    .read_avail(),
    .data_out(gs_mem_dout[7:0]),

    .busy(gs_mem_busy),

    // PSRAM signals
    .cram_a(cram0_a),
    .cram_dq(cram0_dq),
    .cram_wait(cram0_wait),
    .cram_clk(cram0_clk),
    .cram_adv_n(cram0_adv_n),
    .cram_cre(cram0_cre),
    .cram_ce0_n(cram0_ce0_n),
    .cram_ce1_n(cram0_ce1_n),
    .cram_oe_n(cram0_oe_n),
    .cram_we_n(cram0_we_n),
    .cram_ub_n(cram0_ub_n),
    .cram_lb_n(cram0_lb_n)
  );


//output  wire    [21:16] cram0_a,
//inout   wire    [15:0]  cram0_dq,
//input   wire            cram0_wait,
//output  wire            cram0_clk,
//output  wire            cram0_adv_n,
//output  wire            cram0_cre,
//output  wire            cram0_ce0_n,
//output  wire            cram0_ce1_n,
//output  wire            cram0_oe_n,
//output  wire            cram0_we_n,
//output  wire            cram0_ub_n,
//output  wire            cram0_lb_n,


wire gs_sel = (addr[7:0] ==? 'b1011?011) & ~&status[21:20];

localparam [3:0] comp_f = 4;
localparam [3:0] comp_a = 2;
localparam       comp_x = ((32767 * (comp_f - 1)) / ((comp_f * comp_a) - 1)) + 1; // +1 to make sure it won't overflow
localparam       comp_b = comp_x * comp_a;

function [15:0] compr; input [15:0] inp;
	reg [15:0] v, v2;
	begin
		v  = inp[15] ? (~inp) + 1'd1 : inp;
		v2 = (v < comp_x[15:0]) ? (v * comp_a) : (((v - comp_x[15:0])/comp_f) + comp_b[15:0]);
		compr = inp[15] ? ~(v2-1'd1) : v2;
	end
endfunction

reg [15:0] audio_l, audio_r;
always @(posedge clk_aud) begin
	reg [15:0] pre_l, pre_r;
	pre_l <= {ts_l,4'd0} + {{3{gs_l[14]}}, gs_l[13:1]} + {2'b00, saa_l, 6'd0} + {3'b000, ear_out, mic_out, tape_aud, 10'd0};
	pre_r <= {ts_r,4'd0} + {{3{gs_r[14]}}, gs_r[13:1]} + {2'b00, saa_r, 6'd0} + {3'b000, ear_out, mic_out, tape_aud, 10'd0};

	audio_l <= compr(pre_l);
	audio_r <= compr(pre_r);
end
/*
assign AUDIO_L = audio_l;
assign AUDIO_R = audio_r;
*/

////////////////////   VIDEO   ///////////////////
/*
	/////////// ULA Ports ///////////////////
	input         reset,

	input         clk_sys,	// master clock
	input         ce_7mp,
	input         ce_7mn,
	output        ce_cpu_sp,
	output        ce_cpu_sn,

	// CPU interfacing
	input  [15:0] addr,
	input   [7:0] din,
	input         nMREQ,
	input         nIORQ,
	input         nRFSH,
	input         nRD,
	input         nWR,
	output        nINT,

	// VRAM interfacing
	output [14:0] vram_addr,
	input   [7:0] vram_dout,
	output  [7:0] port_ff,
	
	// ULA+
	input         ulap_avail,
	output        ulap_sel,
	output  [7:0] ulap_dout,
	output reg    ulap_ena,
	output reg    ulap_mono,
	output  [7:0] ulap_color,

	// Timex mode
	input         tmx_avail,
	output reg    mode512,
vv
	// Misc. signals
	input         snow_ena,
	input         mZX,
	input         m128,
	input         page_scr,
	input   [2:0] page_ram,
	input   [2:0] border_color,
	input   [1:0] wide,

	// Video outputs
	output reg    HSync,
	output reg    VSync,
	output reg    HBlank,
	output reg    VBlank,
	output        I,R,G,B
	*/
	
wire        ce_cpu_sn;
wire        ce_cpu_sp;
wire [14:0] vram_addr;
wire  [7:0] vram_dout;
wire  [7:0] port_ff;
wire        ulap_sel;
wire  [7:0] ulap_dout;

reg mZX, m128;
always @(posedge clk_sys) begin
	case(status[9:8])
		      0: {mZX, m128} <= 2'b10;
		      1: {mZX, m128} <= 2'b11;
		default: {mZX, m128} <= 2'b00;
	endcase
end

wire [1:0] scale = status[16:15];
wire       HSync, VSync, HBlank, VBlank, PHBlank, PVBlank;
wire       ulap_ena, ulap_mono, mode512;
wire       ulap_avail = ~status[14] & ~trdos_en;
wire       tmx_avail = ~status[13] & ~trdos_en;
wire       snow_ena = status[25] & &turbo & ~plus3;
wire       I,R,G,B;
wire [7:0] ulap_color;
	
//ULA ULA(.*, .din(cpu_dout), .page_ram(page_ram[2:0]));
ULA ULA(.*, .din(cpu_dout), .page_ram(page_ram[2:0]),	.HOut(ULA_HC),.VOut(ULA_VC),.borderon(~status[4]));

wire ce_sys = ce_7mp | (mode512 & ce_7mn);
reg ce_sys1;
always @(posedge clk_sys) begin
	reg [1:0] ce_sys2;
	ce_sys2 <= {ce_sys2[0],ce_sys};
	ce_sys1 <= |ce_sys2;
end

wire CLK_VIDEO = clk_56;

reg ce_pix;
always @(posedge CLK_VIDEO) begin
	reg ce1;
	ce1 <= ce_sys1;
	ce_pix <= ce1;
end

reg [7:0] Rx, Gx, Bx;
reg       hs,vs,hbl,vbl;
always @(posedge CLK_VIDEO) if (ce_pix) begin
	hs <= HSync;
	if(~hs & HSync) vs <= VSync;

	//hbl <= HBlank;
	//vbl <= VBlank;
	hbl <= PHBlank;
	vbl <= PVBlank;

	casex({ulap_ena, ulap_mono})
		'b0X: {Gx,Rx,Bx} <= {G, G, {6{I & G}}, R, R, {6{I & R}}, B, B, {6{I & B}}};
		'b10: {Gx,Rx,Bx} <= {{2{ulap_color[7:5]}}, ulap_color[7:6], {2{ulap_color[4:2]}}, ulap_color[4:3], {4{ulap_color[1:0]}}};
		'b11: {Gx,Rx,Bx} <= {3{ulap_color}};
	endcase
end

reg [9:0] vcrop;
reg [1:0] wide;
always @(posedge CLK_VIDEO) begin
	vcrop <= 0;
	wide[0] <= 0;
	wide[1] <= status[38] & ~vcrop_en;
	/*if(HDMI_WIDTH >= (HDMI_HEIGHT + HDMI_HEIGHT[11:1]) && !forced_scandoubler && !scale) begin
		if(HDMI_HEIGHT == 480)  vcrop <= 240;
		if(HDMI_HEIGHT == 600)  begin vcrop <= 200; wide <= vcrop_en; end
		if(HDMI_HEIGHT == 720)  vcrop <= 240;
		if(HDMI_HEIGHT == 768)  vcrop <= 256;
		if(HDMI_HEIGHT == 800)  begin vcrop <= 200; wide <= vcrop_en; end
		if(HDMI_HEIGHT == 1080) vcrop <= status[29] ? 10'd216 : 10'd270;
		if(HDMI_HEIGHT == 1200) vcrop <= 240;
	end*/
end

wire en1080p=0;
wire [1:0] ar = status[5:4];
wire vcrop_en = en1080p ? |status[29:28] : status[28];

//wire de=~(HBlank | VBlank);
wire de=~(hbl | vbl);
assign video_rgb_clock=pix_clk;
assign video_rgb_clock_90=pix_clk_90;
assign video_skip=1'b0;

/*assign video_vs=vs;
assign video_hs=hs;
assign video_de=de;*/

  reg video_de_reg;
  reg video_hs_reg;
  reg video_vs_reg;
  reg [23:0] video_rgb_reg;

assign video_vs=video_vs_reg;
assign video_hs=video_hs_reg;
assign video_de=video_de_reg;

reg menu_state;
reg [1:0] vkb_state;
reg debug_state;
wire osd_active=vkb_state[0] | menu_state | debug_state;

//On Screen Display paramters
wire [8:0]OSDX=9'd13;
wire [8:0]OSDY=9'd32;
wire [8:0]OSDW=9'd256;
wire [8:0]OSDH=9'd128;

wire [8:0] ULA_HC;
wire [8:0] ULA_VC;

wire osd_out;
wire [14:0] osd_address=((ULA_VC-OSDY)*256) + (ULA_HC-OSDX) ;

reg fdc_led;
always @(posedge CLK_VIDEO) begin
 if (ULA_VC==240) fdc_led<=fdc_sel;
end

//Translation of 4 bit colour (Bright bit + 3 bit GBR)
//wire [23:0] video_translate=I?{R,R,R,R,R,R,R,R,G,G,G,G,G,G,G,G,B,B,B,B,B,B,B,B}:{R,R,1'b0,R,R,3'b000,G,G,1'b0,G,G,3'b000,B,B,1'b0,B,B,3'b000};
wire [23:0] video_translate={Rx,Gx,Bx};
wire in_OSD_Area=((ULA_HC >= OSDX) && (ULA_HC < OSDX+OSDW) && (ULA_VC >= OSDY) && (ULA_VC < OSDY+OSDH));

wire in_LED_Area=((ULA_HC >= 280) && (ULA_HC < 296) && (ULA_VC >= 200) && (ULA_VC < 208) && (!status[42]));
wire [23:0] LED_State=fdc_led?24'hff0000:24'h400000;

wire [23:0] video_inc_OSD_mask=in_OSD_Area?video_translate & 24'b010111110101111101011111:video_translate;
wire [23:0] video_inc_OSD=(in_OSD_Area & osd_out)?24'b111111111111111111111111:video_inc_OSD_mask;
//wire [23:0] video_final=osd_active?video_inc_OSD:video_translate;
wire [23:0] video_pre_led=osd_active?video_inc_OSD:video_translate;
wire [23:0] video_final=in_LED_Area?LED_State:video_pre_led;
//wire vblank_frame=new_border_mode?{10'd0,status[4],10'd0,3'd0}:24'd0;		//change scaler slot if needed
assign video_rgb=video_rgb_reg;//de?video_final:vblank_frame;


//need to inform pocket to change scaler
 reg hs_prev;
  reg [2:0] hs_delay;
  reg vs_prev;
  reg de_prev;
wire [23:0] video_slot_rgb = {10'b0, status[4], 10'b0, 3'b0};

  always @(posedge ce_7mp) begin//pix_clk) begin
    video_hs_reg  <= 0;
    video_de_reg  <= 0;
    video_rgb_reg <= 24'h0;

    if (de) begin
      video_de_reg  <= 1;

      video_rgb_reg <= video_final;
    end else if (de_prev && ~de) begin
      video_rgb_reg <= video_slot_rgb;
    end

    if (hs_delay > 0) begin
      hs_delay <= hs_delay - 1;
    end

    if (hs_delay == 1) begin
      video_hs_reg <= 1;
    end

    if (~hs_prev && hs) begin
      // HSync went high. Delay by 3 cycles to prevent overlapping with VSync
      hs_delay <= 7;
    end

    // Set VSync to be high for a single cycle on the rising edge of the VSync coming out of the core
    video_vs_reg <= ~vs_prev && vs;
    hs_prev <= hs;
    vs_prev <= vs;
    de_prev <= de;
  end

/*reg new_vmode = 0;
always @(posedge clk_sys) begin
	reg [1:0] vmode;
	
	vmode<=status[9:8];
	if(vmode != status[9:8]) new_vmode <= ~new_vmode;		//8:9 = video timing
end*/



//////////////////////////////////////////////////////////
//  SOFT CPU FOR PROCESSING FILES/OSD ETC               //
//////////////////////////////////////////////////////////

reg [63:0] status;
reg  status_set;
reg  status_ack;
wire [11:1] Fn;
wire  [2:0] mod;


reg [31:0] pico_mem_rd;
wire [31:0] pico_mem_wr;
wire [31:0] pico_address;
wire [3:0] pico_mem_wstrb;
wire pico_mem_valid;
wire pico_mem_instr;
wire pico_mem_we;
wire pico_mem_oe;
wire pico_reset_n=reset_n;

reg pico_mem_ready;

//wire resetp=1'b0;
wire clk_pico=ce_7mp;
  always @(posedge clk_pico) begin
    if (~pico_reset_n) pico_mem_ready <= 0;
    else begin
      casex (pico_address)
        32'h0xxx_xxxx: pico_mem_ready <= ~pico_mem_ready & pico_mem_valid;
        32'h1xxx_xxxx: pico_mem_ready <= ~pico_mem_ready & pico_mem_valid;
        32'h2xxx_xxxx: pico_mem_ready <= ~pico_mem_ready & pico_mem_valid;
		  32'h3xxx_xxxx: pico_mem_ready <= ~pico_mem_ready & pico_mem_valid;
		  32'h4xxx_xxxx: pico_mem_ready <= ~pico_mem_ready & pico_mem_valid;
        default: pico_mem_ready <= 0;
      endcase
    end
  end

  picorv32 #(
      .COMPRESSED_ISA(1),
      .ENABLE_IRQ(1),
      .ENABLE_MUL(1),
      .ENABLE_DIV(1)
  ) u_cpu (
      .clk(clk_pico),
      .resetn(pico_reset_n),
      .mem_valid(pico_mem_valid),	//output
      .mem_instr(pico_mem_instr),	//output
      .mem_ready(pico_mem_ready),	//input
      .mem_addr(pico_address),		//output
      .mem_wdata(pico_mem_wr),		//output
      .mem_wstrb(pico_mem_wstrb),	//output
      .mem_rdata(pico_mem_rd)		//input
  );

/*address ranges
0x00000000 - PICOVR32 ROM
0x10000000 - PICOVR32 ROM
0x20000000 - OSD RAM	4k
0x30000000 - Core IO
*/

// ROM - CPU code.
wire [31:0] pico_rom_dout;
sprom #(
    .aw(12),
    .dw(32),
    .MEM_INIT_FILE("../../firmware/firmware.hex")
) pico_rom (
    .clk (clk_sys),
    .rst (~pico_reset_n),
    .ce  (pico_mem_valid && pico_address[31:28] == 4'h0),
    .oe  (1'b1),
    .addr(pico_address[31:2]),
    .dout  (pico_rom_dout)
);

wire [31:0] pico_workram_dout;
// RAM - CPU code.
  genvar gi;
  generate
    for (gi = 0; gi < 4; gi = gi + 1) begin : picoram
      spram #(
          .aw(12),
          .dw(8)
      ) pico_ram (
          .clk (clk_sys),
          .rst (~pico_reset_n),
          .ce  (pico_mem_valid && pico_address[31:28] == 4'h1),
          .oe  (1'b1),
          .addr(pico_address[31:2]),
          .dout  (pico_workram_dout[(gi+1)*8-1:gi*8]),
          .di  (pico_mem_wr[(gi+1)*8-1:gi*8]),
          .we  (pico_mem_wstrb[gi])
      );
    end
  endgenerate




wire  osd_ram_we=(pico_address[31:24]==8'h20) && pico_mem_valid && (pico_mem_wstrb != 0);
reg [1:0] osd_write_state;
reg [31:0] pico_address_byte;
reg [7:0] pico_data_byte;

always_comb begin    
	case (pico_mem_wstrb)
		4'b1000:	begin
			pico_address_byte<={pico_address[31:2],2'b11};			
			pico_data_byte<=pico_mem_wr[31:24];			
		end
		4'b0100:	begin
			pico_address_byte<={pico_address[31:2],2'b10};			
			pico_data_byte<=pico_mem_wr[23:16];						
		end
		4'b0010:	begin
			pico_address_byte<={pico_address[31:2],2'b01};			
			pico_data_byte<=pico_mem_wr[15:8];			
		end
		4'b0001:	begin
			pico_address_byte<={pico_address[31:2],2'b00};			
			pico_data_byte<=pico_mem_wr[7:0];
		end
		default:  begin
			pico_address_byte<=32'd0;
			pico_data_byte<=8'd0;
			end
	endcase
end

wire [7:0] osd_ram_dout;

//4k ram to hold osd display (max 256 x 128 @ 1 bit)
osdram osdram (
	.clock_a(clk_sys),
	//Port A - Read/Write 8 bit data bus (10 bit address bus)
	.address_a(pico_address_byte[11:0]),
	.data_a(pico_data_byte),
	.q_a(osd_ram_dout),
	.wren_a(osd_ram_we),
	
	//Port B - Video read 1 bit data bus (13 bit address bus)
	.clock_b(pix_clk),
	.address_b(osd_address),
	.wren_b(1'b0),
	.q_b(osd_out)
);


always_comb
begin		
	casex (pico_address)
		32'h0xxx_xxxx:	pico_mem_rd<=pico_rom_dout;
		32'h1xxx_xxxx:	pico_mem_rd<=pico_workram_dout;
		32'h2xxx_xxxx:	pico_mem_rd<=osd_ram_dout;

		//IO
		//32'h3000_00D0:	pico_mem_rd<={31'h0,ram_ready};
		
		32'h3000_00_f0:	pico_mem_rd<={16'h0,cont1_key_s};	//read joypad
		
		32'h3000_02_00: pico_mem_rd<={status[31:25], speed_set ? speed_req : 3'b000, status[21:13], arch_set ? arch : snap_hwset ? snap_hw : status[12:8], status[7:0]};
		32'h3000_02_04: pico_mem_rd<=status[63:32];
		32'h3000_02_08: pico_mem_rd<={31'h0,status_set};
		
		//debug outputs to soft cpu
		/*32'h3000_0800:	pico_mem_rd<={15'h0,status_running,1'b0,target_dataslot_err_s,ioctl_state,1'h0,target_dataslot_ack_s,ioctl_wait,rom_loaded,3'h0,ram_ready};	//debug ioctl_state
		32'h3000_0804:	pico_mem_rd<=ram_addr;	//debug ioctl_state
		32'h3000_0808:	pico_mem_rd<={31'h0,target_dataslot_ack_s};
		32'h3000_080c:	pico_mem_rd<={24'h0,ram_dout};
		32'h3000_0810:	pico_mem_rd<=ioctl_addr;*/
		
		//32'h3000_0800:	pico_mem_rd<={8'b10000000,8'h0,6'h0,debug_image_scan_state,debug_buff[0]};	//debug ioctl_state
		//32'h3000_0800:	pico_mem_rd<={ioctl_id,4'b0000,ioctl_state,7'b0000000,vhd_en};
		
		//32'h3000_0804:	pico_mem_rd<={ioctl_id,8'h0,7'b0000000,fdd_ready};//sd_ack_addr;//ioctl_addr;	//debug ioctl_addr
		//32'h3000_0808:	pico_mem_rd<={31'h0,target_dataslot_ack_s};
		//32'h3000_080c:	pico_mem_rd<=ioctl_addr;//{24'h0,ioctl_dout};
		//32'h3000_0810:	pico_mem_rd<=ioctl_size;
				
		32'h4000_00_04: pico_mem_rd<={16'h0,pico_dataslot_size_u};
		32'h4000_00_08: pico_mem_rd<=pico_dataslot_size_l;
		32'h4000_00_0c: pico_mem_rd<={16'h0,pico_dataslot_id};
		32'h4000_00_10: pico_mem_rd<={31'h0,ioctl_download};
		32'h4000_00_1c: pico_mem_rd<={31'h0,pico_dataslot_update};
		32'h4000_00_20: pico_mem_rd<={31'h0,ioctl_complete};
		//32'h4000_00_24: pico_mem_rd<={16'h0,debug_image_track_offsets_in};
		
		//32'h4000_01_00: pico_mem_rd<={sd_lba_plus3[23:0],8'h0};
		32'h4000_01_00: pico_mem_rd<=sd_lba[0];//sd_lba_plus3;
		32'h4000_01_04: pico_mem_rd<={16'h0,8'h0,6'h0,sd_sector_request};
		
		32'h4000_01_10: pico_mem_rd<='h200;
		32'h4000_01_1c: pico_mem_rd<=sd_lba[1];
		//32'h4000_02_xx: pico_mem_rd<={24'h0,debug_buff[pico_address[4:2]]};
		
		
		32'h4000_02_04: pico_mem_rd<=filename_dout;
		
		32'h4000_04_00: pico_mem_rd<={24'h0,sd_buff_din_plus3};
		32'h4000_04_04: pico_mem_rd<={24'h0,sd_buff_din_wd};
		

		default: pico_mem_rd<=32'h0;
				
	endcase
	
end

/*
always_comb
begin		
	casex ({pico_mem_wstrb,pico_mem_valid,pico_address})
		
			//Virtual keyboard reads
			37'b0000_1_00011110_00000001_00000000: vkb_keyrowFE<=pico_mem_wr[4:0];	//0x30000100
			37'b0000_1_00011110_00000001_00000100: vkb_keyrowFD<=pico_mem_wr[4:0];	//0x30000104
			37'b0000_1_00011110_00000001_00001000: vkb_keyrowFB<=pico_mem_wr[4:0];	//0x30000108
			37'b0000_1_00011110_00000001_00001100: vkb_keyrowF7<=pico_mem_wr[4:0];	//0x3000010c
			37'b0000_1_00011110_00000010_00000000: vkb_keyrowEF<=pico_mem_wr[4:0];	//0x30000200	- change to 110 etc
			37'b0000_1_00011110_00000010_00000100: vkb_keyrowDF<=pico_mem_wr[4:0];	//0x30000104
			37'b0000_1_00011110_00000010_00001000: vkb_keyrowBF<=pico_mem_wr[4:0];	//0x30000108
			37'b0000_1_00011110_00000010_00001100: vkb_keyrow7F<=pico_mem_wr[4:0];	//0x3000010c
			37'b1_xx_x_xxxxxxxx_xxxxxxxx_xxxxxxxx:
				
	endcase	
end
*/

reg old_dataslot_update;
reg pico_dataslot_update;
reg [15:0] pico_dataslot_id;
reg [31:0] pico_dataslot_size_l;
reg [15:0] pico_dataslot_size_u;
reg dataslot_ack;
reg pico_ioctl_complete_ack;



always @(posedge clk_74a or negedge reset_n) begin
	if (!reset_n) begin
		pico_dataslot_update<=0;
		old_dataslot_update<=0;
	end
	else
	begin
		if (dataslot_update & ~old_dataslot_update) begin
			pico_dataslot_update<=dataslot_update;
			pico_dataslot_size_l<=dataslot_update_size;
			pico_dataslot_size_u<=dataslot_update_size_u;
			pico_dataslot_id<=dataslot_update_id;
		end else if (dataslot_ack) begin
			pico_dataslot_update<=0;
		end
		old_dataslot_update<=dataslot_update;
	end
end

initial begin
	ioctl_download_req<=0;
	ioctl_upload_req<=0;
end

always @(posedge clk_pico or negedge reset_n) //_comb
begin	
	if (!reset_n) begin
		ioctl_download_req<=0;
		ioctl_upload_req<=0;
		rom_loaded<=0;
		dataslot_ack<=0;
		status_ack<=0;
		pico_ioctl_complete_ack<=0;
//		u765_debug_assert<=0;
	end else if (pico_mem_wstrb != 0 && pico_mem_valid) begin
		case (pico_address)
		
//			32'h3000_03_00: requested_addr<=pico_mem_wr[24:0];

			//OSD control
			32'h3000_00_E0: vkb_state<=pico_mem_wr[1:0];
			32'h3000_00_E4: menu_state<=pico_mem_wr[0];
			32'h3000_00_E8: pause_z80_req<=pico_mem_wr[0];
			32'h3000_00_EC: debug_state<=pico_mem_wr[0];
		
			//Virtual keyboard reads
			32'h3000_01_00: vkb_keyrowFE<=pico_mem_wr[4:0];
			32'h3000_01_04: vkb_keyrowFD<=pico_mem_wr[4:0];
			32'h3000_01_08: vkb_keyrowFB<=pico_mem_wr[4:0];
			32'h3000_01_0c: vkb_keyrowF7<=pico_mem_wr[4:0];
			32'h3000_01_10: vkb_keyrowEF<=pico_mem_wr[4:0];
			32'h3000_01_14: vkb_keyrowDF<=pico_mem_wr[4:0];
			32'h3000_01_18: vkb_keyrowBF<=pico_mem_wr[4:0];
			32'h3000_01_1c: vkb_keyrow7F<=pico_mem_wr[4:0];
			
			32'h3000_02_00: status[31:0]<=pico_mem_wr;
			32'h3000_02_04: status[63:32]<=pico_mem_wr;
			32'h3000_02_08: status_ack<=pico_mem_wr[0];
			
			//joystick keyboard reads
			32'h3000_04_00: joykb_keyrowFE[0]<=pico_mem_wr[4:0];
			32'h3000_04_04: joykb_keyrowFD[0]<=pico_mem_wr[4:0];
			32'h3000_04_08: joykb_keyrowFB[0]<=pico_mem_wr[4:0];
			32'h3000_04_0c: joykb_keyrowF7[0]<=pico_mem_wr[4:0];
			32'h3000_04_10: joykb_keyrowEF[0]<=pico_mem_wr[4:0];
			32'h3000_04_14: joykb_keyrowDF[0]<=pico_mem_wr[4:0];
			32'h3000_04_18: joykb_keyrowBF[0]<=pico_mem_wr[4:0];
			32'h3000_04_1c: joykb_keyrow7F[0]<=pico_mem_wr[4:0];
			
			32'h3000_04_20: joykb_keyrowFE[1]<=pico_mem_wr[4:0];
			32'h3000_04_24: joykb_keyrowFD[1]<=pico_mem_wr[4:0];
			32'h3000_04_28: joykb_keyrowFB[1]<=pico_mem_wr[4:0];
			32'h3000_04_2c: joykb_keyrowF7[1]<=pico_mem_wr[4:0];
			32'h3000_04_30: joykb_keyrowEF[1]<=pico_mem_wr[4:0];
			32'h3000_04_34: joykb_keyrowDF[1]<=pico_mem_wr[4:0];
			32'h3000_04_38: joykb_keyrowBF[1]<=pico_mem_wr[4:0];
			32'h3000_04_3c: joykb_keyrow7F[1]<=pico_mem_wr[4:0];
			
			32'h3000_04_40: joykb_keyrowFE[2]<=pico_mem_wr[4:0];
			32'h3000_04_44: joykb_keyrowFD[2]<=pico_mem_wr[4:0];
			32'h3000_04_48: joykb_keyrowFB[2]<=pico_mem_wr[4:0];
			32'h3000_04_4c: joykb_keyrowF7[2]<=pico_mem_wr[4:0];
			32'h3000_04_50: joykb_keyrowEF[2]<=pico_mem_wr[4:0];
			32'h3000_04_54: joykb_keyrowDF[2]<=pico_mem_wr[4:0];
			32'h3000_04_58: joykb_keyrowBF[2]<=pico_mem_wr[4:0];
			32'h3000_04_5c: joykb_keyrow7F[2]<=pico_mem_wr[4:0];
			
			32'h3000_04_60: joykb_keyrowFE[3]<=pico_mem_wr[4:0];
			32'h3000_04_64: joykb_keyrowFD[3]<=pico_mem_wr[4:0];
			32'h3000_04_68: joykb_keyrowFB[3]<=pico_mem_wr[4:0];
			32'h3000_04_6c: joykb_keyrowF7[3]<=pico_mem_wr[4:0];
			32'h3000_04_70: joykb_keyrowEF[3]<=pico_mem_wr[4:0];
			32'h3000_04_74: joykb_keyrowDF[3]<=pico_mem_wr[4:0];
			32'h3000_04_78: joykb_keyrowBF[3]<=pico_mem_wr[4:0];
			32'h3000_04_7c: joykb_keyrow7F[3]<=pico_mem_wr[4:0];
			
			32'h3000_04_80: joykb_keyrowFE[4]<=pico_mem_wr[4:0];
			32'h3000_04_84: joykb_keyrowFD[4]<=pico_mem_wr[4:0];
			32'h3000_04_88: joykb_keyrowFB[4]<=pico_mem_wr[4:0];
			32'h3000_04_8c: joykb_keyrowF7[4]<=pico_mem_wr[4:0];
			32'h3000_04_90: joykb_keyrowEF[4]<=pico_mem_wr[4:0];
			32'h3000_04_94: joykb_keyrowDF[4]<=pico_mem_wr[4:0];
			32'h3000_04_98: joykb_keyrowBF[4]<=pico_mem_wr[4:0];
			32'h3000_04_9c: joykb_keyrow7F[4]<=pico_mem_wr[4:0];
			
			32'h3000_04_a0: joykb_keyrowFE[5]<=pico_mem_wr[4:0];
			32'h3000_04_a4: joykb_keyrowFD[5]<=pico_mem_wr[4:0];
			32'h3000_04_a8: joykb_keyrowFB[5]<=pico_mem_wr[4:0];
			32'h3000_04_ac: joykb_keyrowF7[5]<=pico_mem_wr[4:0];
			32'h3000_04_b0: joykb_keyrowEF[5]<=pico_mem_wr[4:0];
			32'h3000_04_b4: joykb_keyrowDF[5]<=pico_mem_wr[4:0];
			32'h3000_04_b8: joykb_keyrowBF[5]<=pico_mem_wr[4:0];
			32'h3000_04_bc: joykb_keyrow7F[5]<=pico_mem_wr[4:0];
			
			32'h3000_04_c0: joykb_keyrowFE[6]<=pico_mem_wr[4:0];
			32'h3000_04_c4: joykb_keyrowFD[6]<=pico_mem_wr[4:0];
			32'h3000_04_c8: joykb_keyrowFB[6]<=pico_mem_wr[4:0];
			32'h3000_04_cc: joykb_keyrowF7[6]<=pico_mem_wr[4:0];
			32'h3000_04_d0: joykb_keyrowEF[6]<=pico_mem_wr[4:0];
			32'h3000_04_d4: joykb_keyrowDF[6]<=pico_mem_wr[4:0];
			32'h3000_04_d8: joykb_keyrowBF[6]<=pico_mem_wr[4:0];
			32'h3000_04_dc: joykb_keyrow7F[6]<=pico_mem_wr[4:0];
			
			32'h3000_04_e0: joykb_keyrowFE[7]<=pico_mem_wr[4:0];
			32'h3000_04_e4: joykb_keyrowFD[7]<=pico_mem_wr[4:0];
			32'h3000_04_e8: joykb_keyrowFB[7]<=pico_mem_wr[4:0];
			32'h3000_04_ec: joykb_keyrowF7[7]<=pico_mem_wr[4:0];
			32'h3000_04_f0: joykb_keyrowEF[7]<=pico_mem_wr[4:0];
			32'h3000_04_f4: joykb_keyrowDF[7]<=pico_mem_wr[4:0];
			32'h3000_04_f8: joykb_keyrowBF[7]<=pico_mem_wr[4:0];
			32'h3000_04_fc: joykb_keyrow7F[7]<=pico_mem_wr[4:0];
			
			32'h3000_05_00: joykb_keyrowFE[8]<=pico_mem_wr[4:0];
			32'h3000_05_04: joykb_keyrowFD[8]<=pico_mem_wr[4:0];
			32'h3000_05_08: joykb_keyrowFB[8]<=pico_mem_wr[4:0];
			32'h3000_05_0c: joykb_keyrowF7[8]<=pico_mem_wr[4:0];
			32'h3000_05_10: joykb_keyrowEF[8]<=pico_mem_wr[4:0];
			32'h3000_05_14: joykb_keyrowDF[8]<=pico_mem_wr[4:0];
			32'h3000_05_18: joykb_keyrowBF[8]<=pico_mem_wr[4:0];
			32'h3000_05_1c: joykb_keyrow7F[8]<=pico_mem_wr[4:0];
			
			32'h3000_05_20: joykb_keyrowFE[9]<=pico_mem_wr[4:0];
			32'h3000_05_24: joykb_keyrowFD[9]<=pico_mem_wr[4:0];
			32'h3000_05_28: joykb_keyrowFB[9]<=pico_mem_wr[4:0];
			32'h3000_05_2c: joykb_keyrowF7[9]<=pico_mem_wr[4:0];
			32'h3000_05_30: joykb_keyrowEF[9]<=pico_mem_wr[4:0];
			32'h3000_05_34: joykb_keyrowDF[9]<=pico_mem_wr[4:0];
			32'h3000_05_38: joykb_keyrowBF[9]<=pico_mem_wr[4:0];
			32'h3000_05_3c: joykb_keyrow7F[9]<=pico_mem_wr[4:0];
			
			32'h3000_06_00: begin
				Fn<=pico_mem_wr[10:0];
				mod<=pico_mem_wr[18:16];
			end
			
			32'h4000_00_00: ioctl_size_req<=pico_mem_wr;
			32'h4000_00_04: ioctl_addr_h_req<=pico_mem_wr[15:0];
			32'h4000_00_08: ioctl_addr_l_req<=pico_mem_wr;
			32'h4000_00_0c: ioctl_id_req<=pico_mem_wr[15:0];
			32'h4000_00_10: ioctl_download_req<=pico_mem_wr[0];
			32'h4000_00_14: ioctl_upload_req<=pico_mem_wr[0];
			32'h4000_00_18: rom_loaded<=pico_mem_wr[0];
			32'h4000_00_1c: dataslot_ack<=pico_mem_wr[0];
			32'h4000_00_20: pico_ioctl_complete_ack<=pico_mem_wr[0];
			32'h4000_00_24: ioctl_index<=pico_mem_wr[7:0];
			
			//32'h4000_01_00: sd_buff_addr<=pico_mem_wr[8:0];
			32'h4000_01_0c: {img_readonly,img_mounted[0]}<=pico_mem_wr[1:0];
			32'h4000_01_10: img_size[31:0]<=pico_mem_wr;
			32'h4000_01_14: img_size[63:32]<=pico_mem_wr;
			32'h4000_01_18: img_mounted[1]<=pico_mem_wr[0];
			
			
			32'h4000_02_00: filename_addr<=pico_mem_wr;
			32'h4000_02_04: filename_din<=pico_mem_wr;
			32'h4000_02_08: filename_wr<=pico_mem_wr[0];
			
//			32'h4000_04_00: u765_debug_addr<=pico_mem_wr[8:0];
//			32'h4000_04_04: u765_debug_assert<=pico_mem_wr[0];
				
			
			

				
		endcase
	end
	else begin
		status_ack<=0;
		dataslot_ack<=0;
	end
end


reg  status_set_req;
reg  old_status_set_req;
always @(posedge clk_sys) begin
	if (status_set_req & ~old_status_set_req) begin //new request to set status
		status_set<=1;
	end
	else begin
		if (status_ack) status_set<=0;	//turn off once ackowledged
	end
	status_set_req<=(speed_set|arch_set|snap_hwset);
	old_status_set_req<=status_set_req;
end


////////////////////   HID   ////////////////////

wire [15:0] cont1_key_s;

  synch_3 #(
      .WIDTH(16)
  ) cont1_s (
      cont1_key,
      cont1_key_s,
      ce_7mp
  );

reg [4:0] key_data;
initial begin
  vkb_keyrowFE<=5'b11111;  
  vkb_keyrowFD<=5'b11111;
  vkb_keyrowFB<=5'b11111;
  vkb_keyrowF7<=5'b11111;
  vkb_keyrowEF<=5'b11111;
  vkb_keyrowDF<=5'b11111;
  vkb_keyrowBF<=5'b11111;
  vkb_keyrow7F<=5'b11111;
end

reg  [4:0] vkb_keyrowFE;  
reg  [4:0] vkb_keyrowFD;
reg  [4:0] vkb_keyrowFB;
reg  [4:0] vkb_keyrowF7;
reg  [4:0] vkb_keyrowEF;
reg  [4:0] vkb_keyrowDF;
reg  [4:0] vkb_keyrowBF;
reg  [4:0] vkb_keyrow7F;
  
always @* begin

	if (vkb_state>0) begin
		key_data[4] = ~((~vkb_keyrowFE[4] & ~addr[8]) | (~vkb_keyrowFD[4] & ~addr[9]) | (~vkb_keyrowFB[4] & ~addr[10]) | (~vkb_keyrowF7[4] & ~addr[11]) | (~vkb_keyrowEF[4] & ~addr[12]) | (~vkb_keyrowDF[4] & ~addr[13]) | (~vkb_keyrowBF[4] & ~addr[14]) | (~vkb_keyrow7F[4] & ~addr[15]));
		key_data[3] = ~((~vkb_keyrowFE[3] & ~addr[8]) | (~vkb_keyrowFD[3] & ~addr[9]) | (~vkb_keyrowFB[3] & ~addr[10]) | (~vkb_keyrowF7[3] & ~addr[11]) | (~vkb_keyrowEF[3] & ~addr[12]) | (~vkb_keyrowDF[3] & ~addr[13]) | (~vkb_keyrowBF[3] & ~addr[14]) | (~vkb_keyrow7F[3] & ~addr[15]));
		key_data[2] = ~((~vkb_keyrowFE[2] & ~addr[8]) | (~vkb_keyrowFD[2] & ~addr[9]) | (~vkb_keyrowFB[2] & ~addr[10]) | (~vkb_keyrowF7[2] & ~addr[11]) | (~vkb_keyrowEF[2] & ~addr[12]) | (~vkb_keyrowDF[2] & ~addr[13]) | (~vkb_keyrowBF[2] & ~addr[14]) | (~vkb_keyrow7F[2] & ~addr[15]));
		key_data[1] = ~((~vkb_keyrowFE[1] & ~addr[8]) | (~vkb_keyrowFD[1] & ~addr[9]) | (~vkb_keyrowFB[1] & ~addr[10]) | (~vkb_keyrowF7[1] & ~addr[11]) | (~vkb_keyrowEF[1] & ~addr[12]) | (~vkb_keyrowDF[1] & ~addr[13]) | (~vkb_keyrowBF[1] & ~addr[14]) | (~vkb_keyrow7F[1] & ~addr[15]));
		key_data[0] = ~((~vkb_keyrowFE[0] & ~addr[8]) | (~vkb_keyrowFD[0] & ~addr[9]) | (~vkb_keyrowFB[0] & ~addr[10]) | (~vkb_keyrowF7[0] & ~addr[11]) | (~vkb_keyrowEF[0] & ~addr[12]) | (~vkb_keyrowDF[0] & ~addr[13]) | (~vkb_keyrowBF[0] & ~addr[14]) | (~vkb_keyrow7F[0] & ~addr[15]));
	end
	else begin
		key_data = 5'b11111;	
	end
end

//Can't think of a better way to map the Pocket buttons
//to individual keys than to create 10 virtual keyboards
//and compare as to whether a button is pressed...
//There must be a more efficient way.....
reg  [4:0] joykb_keyrowFE [0:9];  
reg  [4:0] joykb_keyrowFD [0:9];
reg  [4:0] joykb_keyrowFB [0:9];
reg  [4:0] joykb_keyrowF7 [0:9];
reg  [4:0] joykb_keyrowEF [0:9];
reg  [4:0] joykb_keyrowDF [0:9];
reg  [4:0] joykb_keyrowBF [0:9];
reg  [4:0] joykb_keyrow7F [0:9];
reg  [4:0] joykey_data [0:9];
reg  [4:0] joykey_final;
reg  [4:0] joykey_ext;


//   [0]    dpad_up
//   [1]    dpad_down
//   [2]    dpad_left
//   [3]    dpad_right
//   [4]    face_a
//   [5]    face_b
//   [6]    face_x
//   [7]    face_y
//   [8]    trig_l1
//   [9]    trig_r1
//   [10]   trig_l2
//   [11]   trig_r2
//   [12]   trig_l3
//   [13]   trig_r3
//   [14]   face_select
//   [15]   face_start

always @* begin

	if (cont1_key_s[0]) begin	//UP
		joykey_data[0] [4] = ~((~joykb_keyrowFE[0] [4] & ~addr[8]) | (~joykb_keyrowFD[0] [4] & ~addr[9]) | (~joykb_keyrowFB[0] [4] & ~addr[10]) | (~joykb_keyrowF7[0] [4] & ~addr[11]) | (~joykb_keyrowEF[0] [4] & ~addr[12]) | (~joykb_keyrowDF[0] [4] & ~addr[13]) | (~joykb_keyrowBF[0] [4] & ~addr[14]) | (~joykb_keyrow7F[0] [4] & ~addr[15]));
		joykey_data[0] [3] = ~((~joykb_keyrowFE[0] [3] & ~addr[8]) | (~joykb_keyrowFD[0] [3] & ~addr[9]) | (~joykb_keyrowFB[0] [3] & ~addr[10]) | (~joykb_keyrowF7[0] [3] & ~addr[11]) | (~joykb_keyrowEF[0] [3] & ~addr[12]) | (~joykb_keyrowDF[0] [3] & ~addr[13]) | (~joykb_keyrowBF[0] [3] & ~addr[14]) | (~joykb_keyrow7F[0] [3] & ~addr[15]));
		joykey_data[0] [2] = ~((~joykb_keyrowFE[0] [2] & ~addr[8]) | (~joykb_keyrowFD[0] [2] & ~addr[9]) | (~joykb_keyrowFB[0] [2] & ~addr[10]) | (~joykb_keyrowF7[0] [2] & ~addr[11]) | (~joykb_keyrowEF[0] [2] & ~addr[12]) | (~joykb_keyrowDF[0] [2] & ~addr[13]) | (~joykb_keyrowBF[0] [2] & ~addr[14]) | (~joykb_keyrow7F[0] [2] & ~addr[15]));
		joykey_data[0] [1] = ~((~joykb_keyrowFE[0] [1] & ~addr[8]) | (~joykb_keyrowFD[0] [1] & ~addr[9]) | (~joykb_keyrowFB[0] [1] & ~addr[10]) | (~joykb_keyrowF7[0] [1] & ~addr[11]) | (~joykb_keyrowEF[0] [1] & ~addr[12]) | (~joykb_keyrowDF[0] [1] & ~addr[13]) | (~joykb_keyrowBF[0] [1] & ~addr[14]) | (~joykb_keyrow7F[0] [1] & ~addr[15]));
		joykey_data[0] [0] = ~((~joykb_keyrowFE[0] [0] & ~addr[8]) | (~joykb_keyrowFD[0] [0] & ~addr[9]) | (~joykb_keyrowFB[0] [0] & ~addr[10]) | (~joykb_keyrowF7[0] [0] & ~addr[11]) | (~joykb_keyrowEF[0] [0] & ~addr[12]) | (~joykb_keyrowDF[0] [0] & ~addr[13]) | (~joykb_keyrowBF[0] [0] & ~addr[14]) | (~joykb_keyrow7F[0] [0] & ~addr[15]));
	end else begin
		joykey_data[0]=5'b11111;
	end

	if (cont1_key_s[1]) begin //DOWN
		joykey_data[1] [4] = ~((~joykb_keyrowFE[1] [4] & ~addr[8]) | (~joykb_keyrowFD[1] [4] & ~addr[9]) | (~joykb_keyrowFB[1] [4] & ~addr[10]) | (~joykb_keyrowF7[1] [4] & ~addr[11]) | (~joykb_keyrowEF[1] [4] & ~addr[12]) | (~joykb_keyrowDF[1] [4] & ~addr[13]) | (~joykb_keyrowBF[1] [4] & ~addr[14]) | (~joykb_keyrow7F[1] [4] & ~addr[15]));
		joykey_data[1] [3] = ~((~joykb_keyrowFE[1] [3] & ~addr[8]) | (~joykb_keyrowFD[1] [3] & ~addr[9]) | (~joykb_keyrowFB[1] [3] & ~addr[10]) | (~joykb_keyrowF7[1] [3] & ~addr[11]) | (~joykb_keyrowEF[1] [3] & ~addr[12]) | (~joykb_keyrowDF[1] [3] & ~addr[13]) | (~joykb_keyrowBF[1] [3] & ~addr[14]) | (~joykb_keyrow7F[1] [3] & ~addr[15]));
		joykey_data[1] [2] = ~((~joykb_keyrowFE[1] [2] & ~addr[8]) | (~joykb_keyrowFD[1] [2] & ~addr[9]) | (~joykb_keyrowFB[1] [2] & ~addr[10]) | (~joykb_keyrowF7[1] [2] & ~addr[11]) | (~joykb_keyrowEF[1] [2] & ~addr[12]) | (~joykb_keyrowDF[1] [2] & ~addr[13]) | (~joykb_keyrowBF[1] [2] & ~addr[14]) | (~joykb_keyrow7F[1] [2] & ~addr[15]));
		joykey_data[1] [1] = ~((~joykb_keyrowFE[1] [1] & ~addr[8]) | (~joykb_keyrowFD[1] [1] & ~addr[9]) | (~joykb_keyrowFB[1] [1] & ~addr[10]) | (~joykb_keyrowF7[1] [1] & ~addr[11]) | (~joykb_keyrowEF[1] [1] & ~addr[12]) | (~joykb_keyrowDF[1] [1] & ~addr[13]) | (~joykb_keyrowBF[1] [1] & ~addr[14]) | (~joykb_keyrow7F[1] [1] & ~addr[15]));
		joykey_data[1] [0] = ~((~joykb_keyrowFE[1] [0] & ~addr[8]) | (~joykb_keyrowFD[1] [0] & ~addr[9]) | (~joykb_keyrowFB[1] [0] & ~addr[10]) | (~joykb_keyrowF7[1] [0] & ~addr[11]) | (~joykb_keyrowEF[1] [0] & ~addr[12]) | (~joykb_keyrowDF[1] [0] & ~addr[13]) | (~joykb_keyrowBF[1] [0] & ~addr[14]) | (~joykb_keyrow7F[1] [0] & ~addr[15]));
	end else begin
		joykey_data[1]=5'b11111;
	end

	if (cont1_key_s[2]) begin //LEFT
		joykey_data[2] [4] = ~((~joykb_keyrowFE[2] [4] & ~addr[8]) | (~joykb_keyrowFD[2] [4] & ~addr[9]) | (~joykb_keyrowFB[2] [4] & ~addr[10]) | (~joykb_keyrowF7[2] [4] & ~addr[11]) | (~joykb_keyrowEF[2] [4] & ~addr[12]) | (~joykb_keyrowDF[2] [4] & ~addr[13]) | (~joykb_keyrowBF[2] [4] & ~addr[14]) | (~joykb_keyrow7F[2] [4] & ~addr[15]));
		joykey_data[2] [3] = ~((~joykb_keyrowFE[2] [3] & ~addr[8]) | (~joykb_keyrowFD[2] [3] & ~addr[9]) | (~joykb_keyrowFB[2] [3] & ~addr[10]) | (~joykb_keyrowF7[2] [3] & ~addr[11]) | (~joykb_keyrowEF[2] [3] & ~addr[12]) | (~joykb_keyrowDF[2] [3] & ~addr[13]) | (~joykb_keyrowBF[2] [3] & ~addr[14]) | (~joykb_keyrow7F[2] [3] & ~addr[15]));
		joykey_data[2] [2] = ~((~joykb_keyrowFE[2] [2] & ~addr[8]) | (~joykb_keyrowFD[2] [2] & ~addr[9]) | (~joykb_keyrowFB[2] [2] & ~addr[10]) | (~joykb_keyrowF7[2] [2] & ~addr[11]) | (~joykb_keyrowEF[2] [2] & ~addr[12]) | (~joykb_keyrowDF[2] [2] & ~addr[13]) | (~joykb_keyrowBF[2] [2] & ~addr[14]) | (~joykb_keyrow7F[2] [2] & ~addr[15]));
		joykey_data[2] [1] = ~((~joykb_keyrowFE[2] [1] & ~addr[8]) | (~joykb_keyrowFD[2] [1] & ~addr[9]) | (~joykb_keyrowFB[2] [1] & ~addr[10]) | (~joykb_keyrowF7[2] [1] & ~addr[11]) | (~joykb_keyrowEF[2] [1] & ~addr[12]) | (~joykb_keyrowDF[2] [1] & ~addr[13]) | (~joykb_keyrowBF[2] [1] & ~addr[14]) | (~joykb_keyrow7F[2] [1] & ~addr[15]));
		joykey_data[2] [0] = ~((~joykb_keyrowFE[2] [0] & ~addr[8]) | (~joykb_keyrowFD[2] [0] & ~addr[9]) | (~joykb_keyrowFB[2] [0] & ~addr[10]) | (~joykb_keyrowF7[2] [0] & ~addr[11]) | (~joykb_keyrowEF[2] [0] & ~addr[12]) | (~joykb_keyrowDF[2] [0] & ~addr[13]) | (~joykb_keyrowBF[2] [0] & ~addr[14]) | (~joykb_keyrow7F[2] [0] & ~addr[15]));
	end else begin
		joykey_data[2]=5'b11111;
	end

	if (cont1_key_s[3]) begin //RIGHT
		joykey_data[3] [4] = ~((~joykb_keyrowFE[3] [4] & ~addr[8]) | (~joykb_keyrowFD[3] [4] & ~addr[9]) | (~joykb_keyrowFB[3] [4] & ~addr[10]) | (~joykb_keyrowF7[3] [4] & ~addr[11]) | (~joykb_keyrowEF[3] [4] & ~addr[12]) | (~joykb_keyrowDF[3] [4] & ~addr[13]) | (~joykb_keyrowBF[3] [4] & ~addr[14]) | (~joykb_keyrow7F[3] [4] & ~addr[15]));
		joykey_data[3] [3] = ~((~joykb_keyrowFE[3] [3] & ~addr[8]) | (~joykb_keyrowFD[3] [3] & ~addr[9]) | (~joykb_keyrowFB[3] [3] & ~addr[10]) | (~joykb_keyrowF7[3] [3] & ~addr[11]) | (~joykb_keyrowEF[3] [3] & ~addr[12]) | (~joykb_keyrowDF[3] [3] & ~addr[13]) | (~joykb_keyrowBF[3] [3] & ~addr[14]) | (~joykb_keyrow7F[3] [3] & ~addr[15]));
		joykey_data[3] [2] = ~((~joykb_keyrowFE[3] [2] & ~addr[8]) | (~joykb_keyrowFD[3] [2] & ~addr[9]) | (~joykb_keyrowFB[3] [2] & ~addr[10]) | (~joykb_keyrowF7[3] [2] & ~addr[11]) | (~joykb_keyrowEF[3] [2] & ~addr[12]) | (~joykb_keyrowDF[3] [2] & ~addr[13]) | (~joykb_keyrowBF[3] [2] & ~addr[14]) | (~joykb_keyrow7F[3] [2] & ~addr[15]));
		joykey_data[3] [1] = ~((~joykb_keyrowFE[3] [1] & ~addr[8]) | (~joykb_keyrowFD[3] [1] & ~addr[9]) | (~joykb_keyrowFB[3] [1] & ~addr[10]) | (~joykb_keyrowF7[3] [1] & ~addr[11]) | (~joykb_keyrowEF[3] [1] & ~addr[12]) | (~joykb_keyrowDF[3] [1] & ~addr[13]) | (~joykb_keyrowBF[3] [1] & ~addr[14]) | (~joykb_keyrow7F[3] [1] & ~addr[15]));
		joykey_data[3] [0] = ~((~joykb_keyrowFE[3] [0] & ~addr[8]) | (~joykb_keyrowFD[3] [0] & ~addr[9]) | (~joykb_keyrowFB[3] [0] & ~addr[10]) | (~joykb_keyrowF7[3] [0] & ~addr[11]) | (~joykb_keyrowEF[3] [0] & ~addr[12]) | (~joykb_keyrowDF[3] [0] & ~addr[13]) | (~joykb_keyrowBF[3] [0] & ~addr[14]) | (~joykb_keyrow7F[3] [0] & ~addr[15]));
	end else begin
		joykey_data[3]=5'b11111;
	end

	if (cont1_key_s[4]) begin //A
		joykey_data[4] [4] = ~((~joykb_keyrowFE[4] [4] & ~addr[8]) | (~joykb_keyrowFD[4] [4] & ~addr[9]) | (~joykb_keyrowFB[4] [4] & ~addr[10]) | (~joykb_keyrowF7[4] [4] & ~addr[11]) | (~joykb_keyrowEF[4] [4] & ~addr[12]) | (~joykb_keyrowDF[4] [4] & ~addr[13]) | (~joykb_keyrowBF[4] [4] & ~addr[14]) | (~joykb_keyrow7F[4] [4] & ~addr[15]));
		joykey_data[4] [3] = ~((~joykb_keyrowFE[4] [3] & ~addr[8]) | (~joykb_keyrowFD[4] [3] & ~addr[9]) | (~joykb_keyrowFB[4] [3] & ~addr[10]) | (~joykb_keyrowF7[4] [3] & ~addr[11]) | (~joykb_keyrowEF[4] [3] & ~addr[12]) | (~joykb_keyrowDF[4] [3] & ~addr[13]) | (~joykb_keyrowBF[4] [3] & ~addr[14]) | (~joykb_keyrow7F[4] [3] & ~addr[15]));
		joykey_data[4] [2] = ~((~joykb_keyrowFE[4] [2] & ~addr[8]) | (~joykb_keyrowFD[4] [2] & ~addr[9]) | (~joykb_keyrowFB[4] [2] & ~addr[10]) | (~joykb_keyrowF7[4] [2] & ~addr[11]) | (~joykb_keyrowEF[4] [2] & ~addr[12]) | (~joykb_keyrowDF[4] [2] & ~addr[13]) | (~joykb_keyrowBF[4] [2] & ~addr[14]) | (~joykb_keyrow7F[4] [2] & ~addr[15]));
		joykey_data[4] [1] = ~((~joykb_keyrowFE[4] [1] & ~addr[8]) | (~joykb_keyrowFD[4] [1] & ~addr[9]) | (~joykb_keyrowFB[4] [1] & ~addr[10]) | (~joykb_keyrowF7[4] [1] & ~addr[11]) | (~joykb_keyrowEF[4] [1] & ~addr[12]) | (~joykb_keyrowDF[4] [1] & ~addr[13]) | (~joykb_keyrowBF[4] [1] & ~addr[14]) | (~joykb_keyrow7F[4] [1] & ~addr[15]));
		joykey_data[4] [0] = ~((~joykb_keyrowFE[4] [0] & ~addr[8]) | (~joykb_keyrowFD[4] [0] & ~addr[9]) | (~joykb_keyrowFB[4] [0] & ~addr[10]) | (~joykb_keyrowF7[4] [0] & ~addr[11]) | (~joykb_keyrowEF[4] [0] & ~addr[12]) | (~joykb_keyrowDF[4] [0] & ~addr[13]) | (~joykb_keyrowBF[4] [0] & ~addr[14]) | (~joykb_keyrow7F[4] [0] & ~addr[15]));
	end else begin
		joykey_data[4]=5'b11111;
	end
	
	if (cont1_key_s[5]) begin	//B
		joykey_data[5] [4] = ~((~joykb_keyrowFE[5] [4] & ~addr[8]) | (~joykb_keyrowFD[5] [4] & ~addr[9]) | (~joykb_keyrowFB[5] [4] & ~addr[10]) | (~joykb_keyrowF7[5] [4] & ~addr[11]) | (~joykb_keyrowEF[5] [4] & ~addr[12]) | (~joykb_keyrowDF[5] [4] & ~addr[13]) | (~joykb_keyrowBF[5] [4] & ~addr[14]) | (~joykb_keyrow7F[5] [4] & ~addr[15]));
		joykey_data[5] [3] = ~((~joykb_keyrowFE[5] [3] & ~addr[8]) | (~joykb_keyrowFD[5] [3] & ~addr[9]) | (~joykb_keyrowFB[5] [3] & ~addr[10]) | (~joykb_keyrowF7[5] [3] & ~addr[11]) | (~joykb_keyrowEF[5] [3] & ~addr[12]) | (~joykb_keyrowDF[5] [3] & ~addr[13]) | (~joykb_keyrowBF[5] [3] & ~addr[14]) | (~joykb_keyrow7F[5] [3] & ~addr[15]));
		joykey_data[5] [2] = ~((~joykb_keyrowFE[5] [2] & ~addr[8]) | (~joykb_keyrowFD[5] [2] & ~addr[9]) | (~joykb_keyrowFB[5] [2] & ~addr[10]) | (~joykb_keyrowF7[5] [2] & ~addr[11]) | (~joykb_keyrowEF[5] [2] & ~addr[12]) | (~joykb_keyrowDF[5] [2] & ~addr[13]) | (~joykb_keyrowBF[5] [2] & ~addr[14]) | (~joykb_keyrow7F[5] [2] & ~addr[15]));
		joykey_data[5] [1] = ~((~joykb_keyrowFE[5] [1] & ~addr[8]) | (~joykb_keyrowFD[5] [1] & ~addr[9]) | (~joykb_keyrowFB[5] [1] & ~addr[10]) | (~joykb_keyrowF7[5] [1] & ~addr[11]) | (~joykb_keyrowEF[5] [1] & ~addr[12]) | (~joykb_keyrowDF[5] [1] & ~addr[13]) | (~joykb_keyrowBF[5] [1] & ~addr[14]) | (~joykb_keyrow7F[5] [1] & ~addr[15]));
		joykey_data[5] [0] = ~((~joykb_keyrowFE[5] [0] & ~addr[8]) | (~joykb_keyrowFD[5] [0] & ~addr[9]) | (~joykb_keyrowFB[5] [0] & ~addr[10]) | (~joykb_keyrowF7[5] [0] & ~addr[11]) | (~joykb_keyrowEF[5] [0] & ~addr[12]) | (~joykb_keyrowDF[5] [0] & ~addr[13]) | (~joykb_keyrowBF[5] [0] & ~addr[14]) | (~joykb_keyrow7F[5] [0] & ~addr[15]));
	end else begin
		joykey_data[5]=5'b11111;
	end

	if (cont1_key_s[6]) begin	//X
		joykey_data[6] [4] = ~((~joykb_keyrowFE[6] [4] & ~addr[8]) | (~joykb_keyrowFD[6] [4] & ~addr[9]) | (~joykb_keyrowFB[6] [4] & ~addr[10]) | (~joykb_keyrowF7[6] [4] & ~addr[11]) | (~joykb_keyrowEF[6] [4] & ~addr[12]) | (~joykb_keyrowDF[6] [4] & ~addr[13]) | (~joykb_keyrowBF[6] [4] & ~addr[14]) | (~joykb_keyrow7F[6] [4] & ~addr[15]));
		joykey_data[6] [3] = ~((~joykb_keyrowFE[6] [3] & ~addr[8]) | (~joykb_keyrowFD[6] [3] & ~addr[9]) | (~joykb_keyrowFB[6] [3] & ~addr[10]) | (~joykb_keyrowF7[6] [3] & ~addr[11]) | (~joykb_keyrowEF[6] [3] & ~addr[12]) | (~joykb_keyrowDF[6] [3] & ~addr[13]) | (~joykb_keyrowBF[6] [3] & ~addr[14]) | (~joykb_keyrow7F[6] [3] & ~addr[15]));
		joykey_data[6] [2] = ~((~joykb_keyrowFE[6] [2] & ~addr[8]) | (~joykb_keyrowFD[6] [2] & ~addr[9]) | (~joykb_keyrowFB[6] [2] & ~addr[10]) | (~joykb_keyrowF7[6] [2] & ~addr[11]) | (~joykb_keyrowEF[6] [2] & ~addr[12]) | (~joykb_keyrowDF[6] [2] & ~addr[13]) | (~joykb_keyrowBF[6] [2] & ~addr[14]) | (~joykb_keyrow7F[6] [2] & ~addr[15]));
		joykey_data[6] [1] = ~((~joykb_keyrowFE[6] [1] & ~addr[8]) | (~joykb_keyrowFD[6] [1] & ~addr[9]) | (~joykb_keyrowFB[6] [1] & ~addr[10]) | (~joykb_keyrowF7[6] [1] & ~addr[11]) | (~joykb_keyrowEF[6] [1] & ~addr[12]) | (~joykb_keyrowDF[6] [1] & ~addr[13]) | (~joykb_keyrowBF[6] [1] & ~addr[14]) | (~joykb_keyrow7F[6] [1] & ~addr[15]));
		joykey_data[6] [0] = ~((~joykb_keyrowFE[6] [0] & ~addr[8]) | (~joykb_keyrowFD[6] [0] & ~addr[9]) | (~joykb_keyrowFB[6] [0] & ~addr[10]) | (~joykb_keyrowF7[6] [0] & ~addr[11]) | (~joykb_keyrowEF[6] [0] & ~addr[12]) | (~joykb_keyrowDF[6] [0] & ~addr[13]) | (~joykb_keyrowBF[6] [0] & ~addr[14]) | (~joykb_keyrow7F[6] [0] & ~addr[15]));
	end else begin
		joykey_data[6]=5'b11111;
	end

	if (cont1_key_s[7]) begin  //Y
		joykey_data[7] [4] = ~((~joykb_keyrowFE[7] [4] & ~addr[8]) | (~joykb_keyrowFD[7] [4] & ~addr[9]) | (~joykb_keyrowFB[7] [4] & ~addr[10]) | (~joykb_keyrowF7[7] [4] & ~addr[11]) | (~joykb_keyrowEF[7] [4] & ~addr[12]) | (~joykb_keyrowDF[7] [4] & ~addr[13]) | (~joykb_keyrowBF[7] [4] & ~addr[14]) | (~joykb_keyrow7F[7] [4] & ~addr[15]));
		joykey_data[7] [3] = ~((~joykb_keyrowFE[7] [3] & ~addr[8]) | (~joykb_keyrowFD[7] [3] & ~addr[9]) | (~joykb_keyrowFB[7] [3] & ~addr[10]) | (~joykb_keyrowF7[7] [3] & ~addr[11]) | (~joykb_keyrowEF[7] [3] & ~addr[12]) | (~joykb_keyrowDF[7] [3] & ~addr[13]) | (~joykb_keyrowBF[7] [3] & ~addr[14]) | (~joykb_keyrow7F[7] [3] & ~addr[15]));
		joykey_data[7] [2] = ~((~joykb_keyrowFE[7] [2] & ~addr[8]) | (~joykb_keyrowFD[7] [2] & ~addr[9]) | (~joykb_keyrowFB[7] [2] & ~addr[10]) | (~joykb_keyrowF7[7] [2] & ~addr[11]) | (~joykb_keyrowEF[7] [2] & ~addr[12]) | (~joykb_keyrowDF[7] [2] & ~addr[13]) | (~joykb_keyrowBF[7] [2] & ~addr[14]) | (~joykb_keyrow7F[7] [2] & ~addr[15]));
		joykey_data[7] [1] = ~((~joykb_keyrowFE[7] [1] & ~addr[8]) | (~joykb_keyrowFD[7] [1] & ~addr[9]) | (~joykb_keyrowFB[7] [1] & ~addr[10]) | (~joykb_keyrowF7[7] [1] & ~addr[11]) | (~joykb_keyrowEF[7] [1] & ~addr[12]) | (~joykb_keyrowDF[7] [1] & ~addr[13]) | (~joykb_keyrowBF[7] [1] & ~addr[14]) | (~joykb_keyrow7F[7] [1] & ~addr[15]));
		joykey_data[7] [0] = ~((~joykb_keyrowFE[7] [0] & ~addr[8]) | (~joykb_keyrowFD[7] [0] & ~addr[9]) | (~joykb_keyrowFB[7] [0] & ~addr[10]) | (~joykb_keyrowF7[7] [0] & ~addr[11]) | (~joykb_keyrowEF[7] [0] & ~addr[12]) | (~joykb_keyrowDF[7] [0] & ~addr[13]) | (~joykb_keyrowBF[7] [0] & ~addr[14]) | (~joykb_keyrow7F[7] [0] & ~addr[15]));
	end else begin
		joykey_data[7]=5'b11111;
	end

	if (cont1_key_s[8] & ~cont1_key_s[14]) begin  //L	-- need to disable when usign L + Select Combo to issue NMI!
		joykey_data[8] [4] = ~((~joykb_keyrowFE[8] [4] & ~addr[8]) | (~joykb_keyrowFD[8] [4] & ~addr[9]) | (~joykb_keyrowFB[8] [4] & ~addr[10]) | (~joykb_keyrowF7[8] [4] & ~addr[11]) | (~joykb_keyrowEF[8] [4] & ~addr[12]) | (~joykb_keyrowDF[8] [4] & ~addr[13]) | (~joykb_keyrowBF[8] [4] & ~addr[14]) | (~joykb_keyrow7F[8] [4] & ~addr[15]));
		joykey_data[8] [3] = ~((~joykb_keyrowFE[8] [3] & ~addr[8]) | (~joykb_keyrowFD[8] [3] & ~addr[9]) | (~joykb_keyrowFB[8] [3] & ~addr[10]) | (~joykb_keyrowF7[8] [3] & ~addr[11]) | (~joykb_keyrowEF[8] [3] & ~addr[12]) | (~joykb_keyrowDF[8] [3] & ~addr[13]) | (~joykb_keyrowBF[8] [3] & ~addr[14]) | (~joykb_keyrow7F[8] [3] & ~addr[15]));
		joykey_data[8] [2] = ~((~joykb_keyrowFE[8] [2] & ~addr[8]) | (~joykb_keyrowFD[8] [2] & ~addr[9]) | (~joykb_keyrowFB[8] [2] & ~addr[10]) | (~joykb_keyrowF7[8] [2] & ~addr[11]) | (~joykb_keyrowEF[8] [2] & ~addr[12]) | (~joykb_keyrowDF[8] [2] & ~addr[13]) | (~joykb_keyrowBF[8] [2] & ~addr[14]) | (~joykb_keyrow7F[8] [2] & ~addr[15]));
		joykey_data[8] [1] = ~((~joykb_keyrowFE[8] [1] & ~addr[8]) | (~joykb_keyrowFD[8] [1] & ~addr[9]) | (~joykb_keyrowFB[8] [1] & ~addr[10]) | (~joykb_keyrowF7[8] [1] & ~addr[11]) | (~joykb_keyrowEF[8] [1] & ~addr[12]) | (~joykb_keyrowDF[8] [1] & ~addr[13]) | (~joykb_keyrowBF[8] [1] & ~addr[14]) | (~joykb_keyrow7F[8] [1] & ~addr[15]));
		joykey_data[8] [0] = ~((~joykb_keyrowFE[8] [0] & ~addr[8]) | (~joykb_keyrowFD[8] [0] & ~addr[9]) | (~joykb_keyrowFB[8] [0] & ~addr[10]) | (~joykb_keyrowF7[8] [0] & ~addr[11]) | (~joykb_keyrowEF[8] [0] & ~addr[12]) | (~joykb_keyrowDF[8] [0] & ~addr[13]) | (~joykb_keyrowBF[8] [0] & ~addr[14]) | (~joykb_keyrow7F[8] [0] & ~addr[15]));
	end else begin
		joykey_data[8]=5'b11111;
	end

	if (cont1_key_s[9]) begin //R
		joykey_data[9] [4] = ~((~joykb_keyrowFE[9] [4] & ~addr[8]) | (~joykb_keyrowFD[9] [4] & ~addr[9]) | (~joykb_keyrowFB[9] [4] & ~addr[10]) | (~joykb_keyrowF7[9] [4] & ~addr[11]) | (~joykb_keyrowEF[9] [4] & ~addr[12]) | (~joykb_keyrowDF[9] [4] & ~addr[13]) | (~joykb_keyrowBF[9] [4] & ~addr[14]) | (~joykb_keyrow7F[9] [4] & ~addr[15]));
		joykey_data[9] [3] = ~((~joykb_keyrowFE[9] [3] & ~addr[8]) | (~joykb_keyrowFD[9] [3] & ~addr[9]) | (~joykb_keyrowFB[9] [3] & ~addr[10]) | (~joykb_keyrowF7[9] [3] & ~addr[11]) | (~joykb_keyrowEF[9] [3] & ~addr[12]) | (~joykb_keyrowDF[9] [3] & ~addr[13]) | (~joykb_keyrowBF[9] [3] & ~addr[14]) | (~joykb_keyrow7F[9] [3] & ~addr[15]));
		joykey_data[9] [2] = ~((~joykb_keyrowFE[9] [2] & ~addr[8]) | (~joykb_keyrowFD[9] [2] & ~addr[9]) | (~joykb_keyrowFB[9] [2] & ~addr[10]) | (~joykb_keyrowF7[9] [2] & ~addr[11]) | (~joykb_keyrowEF[9] [2] & ~addr[12]) | (~joykb_keyrowDF[9] [2] & ~addr[13]) | (~joykb_keyrowBF[9] [2] & ~addr[14]) | (~joykb_keyrow7F[9] [2] & ~addr[15]));
		joykey_data[9] [1] = ~((~joykb_keyrowFE[9] [1] & ~addr[8]) | (~joykb_keyrowFD[9] [1] & ~addr[9]) | (~joykb_keyrowFB[9] [1] & ~addr[10]) | (~joykb_keyrowF7[9] [1] & ~addr[11]) | (~joykb_keyrowEF[9] [1] & ~addr[12]) | (~joykb_keyrowDF[9] [1] & ~addr[13]) | (~joykb_keyrowBF[9] [1] & ~addr[14]) | (~joykb_keyrow7F[9] [1] & ~addr[15]));
		joykey_data[9] [0] = ~((~joykb_keyrowFE[9] [0] & ~addr[8]) | (~joykb_keyrowFD[9] [0] & ~addr[9]) | (~joykb_keyrowFB[9] [0] & ~addr[10]) | (~joykb_keyrowF7[9] [0] & ~addr[11]) | (~joykb_keyrowEF[9] [0] & ~addr[12]) | (~joykb_keyrowDF[9] [0] & ~addr[13]) | (~joykb_keyrowBF[9] [0] & ~addr[14]) | (~joykb_keyrow7F[9] [0] & ~addr[15]));
	end else begin
		joykey_data[9]=5'b11111;
	end
	
	
		
	//now we have all the inputs from the buttons as if they were pressed - AND them with the the button status
	joykey_final=joykey_data[0] & joykey_data[1] & joykey_data[2] & joykey_data[3] & joykey_data[4] & joykey_data[5];// & joykey_data[6] & joykey_data[7] & joykey_data[8] & joykey_data[9];
	//joykey_ext only maps buttons x,y, l & r as additional keys to kempston input
	joykey_ext=joykey_data[6] & joykey_data[7] & joykey_data[8] & joykey_data[9];
end



//reg  [4:0] kbd_dout;
//always @(posedge clk_sys) kbd_dout <= key_data;

/*
wire [11:1] Fn;
wire  [2:0] mod;
wire  [4:0] key_data;
wire recreated_zx = status[37];
wire ghosting     = status[36];
keyboard kbd( .* );

wire  [7:0] mouse_data=8'h0;;

mouse mouse( .*, .reset(cold_reset), .addr(addr[10:8]), .sel(), .dout(mouse_data), .btn_swap(status[35]));
*/
wire       kemp_sel = addr[5:0] == 6'h1F;
//reg  [7:0] kemp_dout;

wire [7:0] kemp_dout =  vkb_state?8'h00:{2'b00, joyk}; //disable kempton input when virtual keyboard is active

/*reg        kemp_mode = 0;
always @(posedge clk_sys) begin
	reg old_status = 0;

	if(reset || joyk || !status[35:34]) kemp_mode <= 0;

	old_status <= ps2_mouse[24];
	if(old_status != ps2_mouse[24] && status[35:34]) kemp_mode <= 1;

	kemp_dout <= kemp_mode ? mouse_data : {2'b00, joyk};
end
*/
wire [2:0] jsel  = status[19:17];

//kempston port 1F
//wire [5:0] joyk  = !jsel ? (joy0[5:0] | joy1[5:0]) : 6'd0;
wire [5:0] joyk  = !jsel ? {cont1_key_s[5],cont1_key_s[4],cont1_key_s[0],cont1_key_s[1],cont1_key_s[2],cont1_key_s[3]} : 6'd0;

//sinclair 1 67890
wire [4:0] joys1 = ({5{jsel[2:0]==1}} & {cont1_key_s[2], cont1_key_s[3], cont1_key_s[1], cont1_key_s[0], cont1_key_s[4]});// | ({5{jsel[1:0]==1}} & {cont2_key_s[2], cont2_key_s[3], cont2_key_s[1], cont2_key_s[0], cont2_key_s[4]});

//sinclair 2 12345
wire [4:0] joys2 = ({5{jsel[2:0]==2}} & {cont1_key_s[4], cont1_key_s[0], cont1_key_s[1], cont1_key_s[3], cont1_key_s[2]});// |({5{jsel[1]}} & {cont2_key_s[4], cont2_key_s[0], cont2_key_s[1], cont2_key_s[3], cont2_key_s[2]});

//cursor 56780
wire [4:0] joyc1 = {5{jsel[2:0]==3}} & ({cont1_key_s[1], cont1_key_s[0], cont1_key_s[3],1'b0, cont1_key_s[4]});// | {cont2_key_s[1], cont2_key_s[0], cont2_key_s[3], 1'b0, cont2_key_s[4]});
//wire [4:0] joyc2 = {5{jsel[2]}} & {cont1_key_s[2] | cont2_key_s[2], 4'b0000};
wire [4:0] joyc2 = {5{jsel[2:0]==3}} & {cont1_key_s[2] , 4'b0000};

wire [4:0] joykb1 = {5{jsel[2:0]!=4}} | (joykey_final);// | {cont2_key_s[1], cont2_key_s[0], cont2_key_s[3], 1'b0, cont2_key_s[4]});




//map to keyboard
wire [4:0] joy_kbd = osd_active?5'b11111:({5{addr[12]}} | ~(joys1 | joyc1)) & ({5{addr[11]}} | ~(joys2 | joyc2)) & joykb1 & joykey_ext;

reg  [4:0] kbd_dout;

always @(posedge clk_sys) kbd_dout <= key_data & joy_kbd;

//////////////////   MF128   ///////////////////
reg         mf128_mem;
reg         mf128_en; // enable MF128 page-in from NMI till reset (or soft off)
wire        mf128_port = ~addr[6] & addr[5] & addr[4] & addr[1];
// read paging registers saved in MF3 (7f3f, 1f3f)
wire        mf3_port = mf128_port & ~addr[7] & (addr[12:8] == 'h1f) & plus3 & mf128_en;

always @(posedge clk_sys) begin
	reg old_m1, old_rd, old_wr;

	old_rd <= io_rd;
	old_wr <= io_wr;
	if(reset) {mf128_mem, mf128_en} <= 0;
	else if(~old_rd & io_rd) begin
		//page in/out for port IN
		if(mf128_port) mf128_mem <= (addr[7] ^ plus3) & mf128_en;
	end else if(~old_wr & io_wr) begin
		//Soft hide
		if(mf128_port) mf128_en <= addr[7] & mf128_en;
	end

	old_m1 <= m1;
	if(~old_m1 & m1 & mod[0] & (addr == 'h66) & ~&mmc_mode) {mf128_mem, mf128_en} <= 2'b11;
	//if(~old_m1 & m1 & mod[0] & (addr == 'h66)) {mf128_mem, mf128_en} <= 2'b11;
end


//////////////////   MMC   //////////////////
reg [1:0] mmc_mode;
reg       vsd_sel = 1;
reg vhd_en = 0;
always @(posedge clk_sys) begin
	//reg vhd_en = 0;

	if(img_mounted[1]) vhd_en <= |img_size;
	if(!reset_n) vhd_en <= 0;
	
	if(reset) begin
		vsd_sel  <= vhd_en;// && !status[33:32]);
		//mmc_mode <= (vhd_en || status[33:32]) ? (status[31:30] ? status[31:30] : 2'b11) : 2'b00;
		mmc_mode <= vhd_en ? (status[31:30] ? status[31:30] : 2'b11) : 2'b00;
	end
end

//wire       mmc_reset = (img_mounted[1] & !status[33:32]);
wire       mmc_reset = img_mounted[1];

wire       mmc_sel;
wire [7:0] mmc_dout;
wire       mmc_mem_en;
wire       mmc_rom_en;
wire       mmc_ram_en;
wire [3:0] mmc_ram_bank;
wire       mmc_ready;

divmmc divmmc
(
	.*,
	.disable_pagein(tape_loaded),
	.mode(mmc_mode), //00-off, 01-divmmc, 10-zxmmc, 11-divmmc+esxdos
	.din(cpu_dout),
	.dout(mmc_dout),
	.active_io(mmc_sel),
	.ready(mmc_ready),

	.rom_active(mmc_rom_en),
	.ram_active(mmc_ram_en),
	.ram_bank(mmc_ram_bank),
	
	.spi_ce(ce_spi),
	.spi_ss(sdss),
	.spi_clk(sdclk),
	.spi_di(sdmiso),
	.spi_do(sdmosi)
);

wire sdss;
wire sdclk;
wire vsdmiso;
wire sdmosi;
//wire sdmiso = vsd_sel ? vsdmiso : SD_MISO; 
wire sdmiso = vsdmiso; 

sd_card sd_card
(
		
	.clk_sys(clk_sys),
	.reset(reset),	
	
	.img_size(img_size),
	.sdhc(1),
	.img_mounted(img_mounted[1]),
	
	.sd_rd(sd_rd_mmc),
	.sd_wr(sd_wr_mmc),
	.sd_ack(sd_ack[1]),
	.sd_lba(sd_lba_mmc),
	
	
	.sd_buff_addr(ioctl_addr[8:0]),
	//.sd_buff_addr(u765_addr),
	.sd_buff_dout(ioctl_dout),
	.sd_buff_din(sd_buff_din_mmc),	
	.sd_buff_wr((ioctl_wr) && (ioctl_index[4:0]==3)),

	.clk_spi(clk_sys),
	.ss(sdss | ~vsd_sel),
	.sck(sdclk),
	.mosi(sdmosi),
	.miso(vsdmiso)
);

//assign SD_CS   = sdss   |  vsd_sel;
//assign SD_SCK  = sdclk  & ~vsd_sel;
//assign SD_MOSI = sdmosi & ~vsd_sel;

reg sd_act;
always @(posedge clk_sys) begin
	reg old_mosi, old_miso;
	integer timeout = 0;

	old_mosi <= sdmosi;
	old_miso <= sdmiso;

	sd_act <= 0;
	if(timeout < 1000000) begin
		timeout <= timeout + 1;
		sd_act <= 1;
	end

	if((old_mosi ^ sdmosi) || (old_miso ^ sdmiso)) timeout <= 0;
end


///////////////////   FDC   ///////////////////
reg         plusd_en;
reg         plusd_mem;
wire        plusd_ena = plusd_stealth ? plusd_mem : plusd_en;
wire        fdd_sel2 = plusd_ena & &addr[7:5] & ~addr[2] & &addr[1:0];

reg         trdos_en;
wire  [7:0] wd_dout;
wire        fdd_rd;
reg         fdd_ready;
reg         fdd_drive1;
reg         fdd_side;
reg         fdd_reset;
wire        fdd_intrq;
wire        fdd_drq;
wire        fdd_sel  = trdos_en & addr[2] & addr[1];
reg         fdd_ro;
wire  [7:0] wdc_dout = (addr[7] & ~plusd_en) ? {fdd_intrq, fdd_drq, 6'h3F} : wd_dout;

reg         plus3_fdd_ready;
wire        plus3_fdd = ~addr[1] & addr[13] & ~addr[14] & ~addr[15] & plus3 & ~page_disable;
wire [7:0]  u765_dout;

wire  [7:0] fdc_dout = plus3_fdd ? u765_dout : wdc_dout; 
wire        fdc_sel  = fdd_sel | fdd_sel2 | plus3_fdd;

//
// current +D implementation notes:
// 1) all +D ports (except page out port) are disabled if +D memory isn't paged in.
// 2) only possible way to page in is through hooks at h08, h3A, h66 addresses.
//
// This may break compatibility with some apps written specifically for +D using 
// direct port access (badly written apps), but won't introduce
// incompatibilities with +D unaware apps.
//
wire        plusd_stealth = 1;

// read video page.
// available for MF128 and PlusD(patched).
wire        portBF = mf128_port & addr[7] & (mf128_mem | plusd_mem);

always @(posedge clk_sys) begin
	reg old_wr, old_rd;
	reg old_mounted;
	reg old_m1;

	if(cold_reset) {plus3_fdd_ready, fdd_ready, plusd_en} <= 0;
	if(reset)      {plusd_mem, trdos_en} <= 0;

	old_mounted <= img_mounted[0];
	if(~old_mounted & img_mounted[0]) begin
		fdd_ro    <= img_readonly;

	   //Only TRDs on +3
		//fdd_ready <= (((ioctl_id == 'h101) & plus3) | ~plus3) && (img_size != 0);
		//plusd_en  <= ((ioctl_id != 'h101) & ~plus3) && (img_size != 0);
		fdd_ready <= ((!ioctl_index[7:6] & plus3) | ~plus3) && (img_size != 0);
		plusd_en  <= (|ioctl_index[7:6] & ~plus3) && (img_size != 0);
		//DSK only for +3
		plus3_fdd_ready <= (plus3 & (ioctl_index[7:6] == 2)) && (img_size != 0);
		//plus3_fdd_ready <= (plus3 & (ioctl_id == 'h100)) && (img_size != 0);
	end

	old_rd <= io_rd;
	old_wr <= io_wr;
	old_m1 <= m1;

	psg_reset <= m1 && (addr == 'h66); // reset AY upon NMI.

	if(plusd_en) begin
		trdos_en <= 0;
		if(~old_wr & io_wr  & (addr[7:0] == 'hEF) & plusd_ena) {fdd_side, fdd_drive1} <= {cpu_dout[7], cpu_dout[1:0] != 2};
		if(~old_wr & io_wr  & (addr[7:0] == 'hE7)) plusd_mem <= 0;
		if(~old_rd & io_rd  & (addr[7:0] == 'hE7) & ~plusd_stealth) plusd_mem <= 1;
		if(~old_m1 & m1 & ((addr == 'h08) | (addr == 'h3A) | (~mod[0] & ~&mmc_mode & (addr == 'h66)))) plusd_mem <= 1;
	end else begin
		plusd_mem <= 0;
		if(~old_wr & io_wr & fdd_sel & addr[7]) {fdd_side, fdd_reset, fdd_drive1} <= {~cpu_dout[4], ~cpu_dout[2], !cpu_dout[1:0]};
		if(m1 && ~old_m1) begin
			if(addr[15:14]) trdos_en <= 0;
				else if((addr[13:8] == 'h3D) & active_48_rom & ~&mmc_mode) trdos_en <= 1;				
				//else if(~mod[0] & (addr == 'h66)) trdos_en <= 1;
		end
	end
end


wire        sd_rd_mmc;
wire        sd_wr_mmc;
wire [31:0] sd_lba_mmc;
wire [7:0]  sd_buff_din_mmc;

wire 			sd_rd_plus3;
wire 			sd_wr_plus3;
wire [31:0] sd_lba_plus3;
wire [7:0]  sd_buff_din_plus3;

wire 			sd_rd_wd;
wire 			sd_wr_wd;
wire [31:0] sd_lba_wd;
wire [7:0]  sd_buff_din_wd;

wire [31:0] sd_lba[2] = '{plus3_fdd_ready ? sd_lba_plus3 : sd_lba_wd, sd_lba_mmc};
wire  [1:0] sd_rd = {sd_rd_mmc, plus3_fdd_ready ? sd_rd_plus3 : sd_rd_wd};
wire  [1:0] sd_wr = {sd_wr_mmc, plus3_fdd_ready ? sd_wr_plus3 : sd_wr_wd};
wire  [1:0] sd_ack;
reg  [8:0] sd_buff_addr;
wire  [7:0] sd_buff_dout;
wire  [7:0] sd_buff_din[2] = '{plus3_fdd_ready ? sd_buff_din_plus3 : sd_buff_din_wd, sd_buff_din_mmc};
wire        sd_buff_wr;
reg  [1:0] img_mounted;
reg [63:0] img_size;
reg        img_readonly;

wd1793 #(1) wd1793
(
	.clk_sys(clk_sys),
	.ce(ce_wd1793),
	.reset((fdd_reset & ~plusd_en) | reset),
	.io_en((fdd_sel2 | (fdd_sel & ~addr[7])) & ~nIORQ & nM1),
	.rd(~nRD),
	.wr(~nWR),
	.addr(plusd_en ? addr[4:3] : addr[6:5]),
	.din(cpu_dout),
	.dout(wd_dout),
	.drq(fdd_drq),
	.intrq(fdd_intrq),

	.img_mounted(img_mounted[0]),
	.img_size(img_size[19:0]),
	.sd_lba(sd_lba_wd),
	.sd_rd(sd_rd_wd),
	.sd_wr(sd_wr_wd), 
	.sd_ack(sd_ack[0]),
	.sd_buff_addr(ioctl_addr[8:0]),
	//.sd_buff_addr(u765_addr),
	.sd_buff_dout(ioctl_dout),
	.sd_buff_din(sd_buff_din_wd), 
	.sd_buff_wr((ioctl_wr) && (ioctl_index[4:0]==1) && (ioctl_index[7:6]!=2)),

	.wp(fdd_ro),

	.size_code(plusd_en ? 3'd4 : 3'd1),
	//.layout(ioctl_id == 'h103),   // 0 = Track-Side-Sector, 1 - Side-Track-Sector
	.layout(ioctl_index[7:6] == 1),
	.side(fdd_side),
	.ready(fdd_drive1 & fdd_ready),

	.input_active(0),
	.input_addr(0),
	.input_data(0),
	.input_wr(0),
	.buff_din(0)
);


/*wire [1:0] debug_image_scan_state;
wire debug_buff_wait;
wire debug_sd_busy;
wire debug_image_edsk;
wire [7:0] debug_buff[8];
wire [11:0] debug_buff_addr;
wire [31:0] debug_i_seek_pos;
wire [15:0] debug_image_track_offsets_in;
wire [15:0] debug_i_sector_size;
*/

//reg u765_debug_assert;
//reg [8:0] u765_debug_addr;
//wire [8:0]u765_addr=u765_debug_assert?u765_debug_addr:ioctl_addr[8:0];

reg [31:0] sd_ack_addr;

u765 #(20'd1800,1) u765
(
	.clk_sys(clk_sys),
	.ce(ce_u765),
	.reset(reset),
	.a0(addr[12]),
	.ready(plus3_fdd_ready),
	.motor(motor_plus3),
	.available(2'b01),
	.fast(1),
	.nRD(~plus3_fdd | nIORQ | ~nM1 | nRD),
	.nWR(~plus3_fdd | nIORQ | ~nM1 | nWR),
	.din(cpu_dout),
	.dout(u765_dout),

	.img_mounted(img_mounted[0]),
	.img_size(img_size[19:0]),
	.img_wp(fdd_ro),
	.sd_lba(sd_lba_plus3),
	.sd_rd(sd_rd_plus3),
	.sd_wr(sd_wr_plus3),
	.sd_ack(sd_ack[0]),	
	.sd_buff_addr(ioctl_addr[8:0]),
	//.sd_buff_addr(u765_addr),
	.sd_buff_dout(ioctl_dout),
	.sd_buff_din(sd_buff_din_plus3),
	//.sd_buff_wr(sd_buff_wr)
	.sd_buff_wr((ioctl_wr) && (ioctl_index==8'h81))
	/*.debug_image_scan_state(debug_image_scan_state),
	.debug_buff_wait(debug_buff_wait),
	.debug_sd_busy(debug_sd_busy),
	.debug_image_edsk(debug_image_edsk),
	.debug_buff(debug_buff),
	.debug_buff_addr(debug_buff_addr),
	.debug_i_seek_pos(debug_i_seek_pos),
	.debug_image_track_offsets_in(debug_image_track_offsets_in),
	.debug_i_sector_size(debug_i_sector_size)*/
);

reg [1:0] sd_sector_request;
reg [1:0] old_sd_rd;
//reg [8:0] sd_plus3_transfer_size;
reg [1:0] old_pico_ioctl_complete_ack;
//reg [7:0] debug_data[32];
always @(posedge clk_sys) begin	
	if (~old_sd_rd[0] & sd_rd[0]) begin		//rising edge of sd_rd		
		sd_sector_request[0]<=1;
	end
	if (~old_sd_rd[1] & sd_rd[1]) begin		//rising edge of sd_rd		
		sd_sector_request[1]<=1;
	end
	if (sd_sector_request[0] & ioctl_download_req) begin
		sd_sector_request[0]<=0;	//turn off request once download starts
		sd_ack[0]<=1;
	end
	if (sd_sector_request[1] & ioctl_download_req) begin
		sd_sector_request[1]<=0;	//turn off request once download starts
		sd_ack[1]<=1;
	end
	if ((~old_pico_ioctl_complete_ack[0] & pico_ioctl_complete_ack) && (ioctl_index[1:0]==1)) begin	
		sd_ack[0]<=0;
		//sd_ack_addr<=ioctl_addr;
	end
	if ((~old_pico_ioctl_complete_ack[1] & pico_ioctl_complete_ack) && (ioctl_index[1:0]==3)) begin	
		sd_ack[1]<=0;
		//sd_ack_addr<=ioctl_addr;
	end
	
	old_sd_rd<=sd_rd;
	//if ((ioctl_index[1:0]==1)) old_pico_ioctl_complete_ack[0]<=pico_ioctl_complete_ack;
	//if ((ioctl_index[1:0]==3)) old_pico_ioctl_complete_ack[1]<=pico_ioctl_complete_ack;
	old_pico_ioctl_complete_ack<={pico_ioctl_complete_ack,pico_ioctl_complete_ack};
	//if (bridge_addr<'d32) debug_data[bridge_addr[4:0]]<=sd_buff_dout;
end

/*always_comb begin
	case (bridge_addr[1:0])
	'b00:	sd_buff_dout=bridge_wr_data[7:0];
	'b01:	sd_buff_dout=bridge_wr_data[15:8];
	'b10:	sd_buff_dout=bridge_wr_data[23:16];
	'b11:	sd_buff_dout=bridge_wr_data[31:24];		
	endcase
end

wire [7:0] diskram_dout_to_u765;
wire [7:0] diskram_din_from_u765;
wire [31:0] diskram_dout_to_bridge;
wire diskram_cpu_write=1'b0;
//wire [8:0] disk_addr=sd_buff_addr;
*/

/*diskram diskram (
	.clock_a(clk_74a),
	.wren_a(bridge_wr && bridge_addr[31:28] == 4'h7),
	.address_a(bridge_addr[31:2]),
	.data_a({bridge_wr_data[7:0], bridge_wr_data[15:8], bridge_wr_data[23:16], bridge_wr_data[31:24]}),
	.q_a(diskram_dout_to_bridge),
	
	.clock_b(clk_sys),
	.wren_b(diskram_cpu_write),
	//.address_b(disk_addr),
	.address_b(sd_buff_addr),
	.data_b(diskram_din_from_765),
	.q_b(diskram_dout_to_765)
);
*/

///////////////////   TAPE   ///////////////////
wire [24:0] tape_addr = 25'h400000 + tape_addr_raw;
wire [24:0] tape_addr_raw;
wire        tape_req;
wire        tape_dout_en;
wire        tape_turbo;
wire  [7:0] tape_dout;
wire        tape_led;
wire        tape_active;
wire        tape_loaded;
wire        tape_vin;

//wire			tmode=(ioctl_id == 2)?2'd0:(ioctl_id == 3)?2'd2:2'd1;
smart_tape tape
(
	.*,
	.reset(reset & ~Fn[10]),
	.ce(ce_tape),

	.turbo(tape_turbo),
	.mode48k(page_disable),
	.pause(Fn[1] & !mod),
	.prev(Fn[2] & !mod),
	.next(Fn[3] & !mod),
	.audio_out(tape_vin),
	.led(tape_led),
	.active(tape_active),
	.available(tape_loaded),
	.req_hdr((reg_DE == 'h11) & !reg_A),

	.buff_rd_en(~nRFSH),
	.buff_rd(tape_req),
	.buff_addr(tape_addr_raw),
	.buff_din(ram_dout),

	.ioctl_download(ioctl_download && (ioctl_index[4:0] == 2)),
	.tape_size(ioctl_addr),
	.tape_mode(ioctl_index[7:6]),

	.m1(~nM1 & ~nMREQ),
	.rom_en(active_48_rom),
	.dout_en(tape_dout_en),
	.dout(tape_dout)
);

reg tape_act = 0;
always @(posedge clk_sys) begin
	int timeout = 0;
	reg old_vin;
	
	old_vin <= tape_vin;

	tape_act <= 1;
	if(old_vin ^ tape_vin) timeout <= 50000000;
	else if(timeout) timeout <= timeout - 1;
	else tape_act <= 0;
end

//wire tape_aud    = ~status[1] & (tape_act ? ~tape_vin : tape_adc_act & ~tape_adc);
wire tape_aud    = ~status[1] & (tape_act ? ~tape_vin : 1'b0);
wire ula_tape_in = tape_act ? ~tape_vin : 1'b0;
//wire ula_tape_in = tape_act ? ~tape_vin : tape_adc_act ? ~tape_adc : (ear_out | (~status[7] & mic_out));
/*
wire tape_adc, tape_adc_act;
ltc2308_tape ltc2308_tape
(
	.clk(CLK_50M),
	.ADC_BUS(ADC_BUS),
	.dout(tape_adc),
	.active(tape_adc_act)
);*/

//////////////////  ARCH SET  //////////////////

reg       arch_set = 0;
reg [4:0] arch;
reg       arch_reset = 0;

initial begin
	status<=64'h1;	//start off with init_reset held
	status_set<=0;
	status_ack<=0;
	arch_set <= 0;
end

always @(posedge clk_sys) begin
	reg [7:0] timeout = 0;
	reg [6:1] old_Fn;
	reg       setwait = 0;

	//arch_set <= 0;
	old_Fn <= Fn[6:1];
	if(mod == 2 && (old_Fn != Fn[6:1])) begin
		if(~old_Fn[1] & Fn[1]) {setwait,arch_set,arch} <= {2'b11,ARCH_ZX48 }; // Alt+F1 - ZX 48
		if(~old_Fn[2] & Fn[2]) {setwait,arch_set,arch} <= {2'b11,ARCH_ZX128}; // Alt+F2 - ZX 128/+2
		if(~old_Fn[3] & Fn[3]) {setwait,arch_set,arch} <= {2'b11,ARCH_ZX3  }; // Alt+F3 - ZX 128 +3
		if(~old_Fn[4] & Fn[4]) {setwait,arch_set,arch} <= {2'b11,ARCH_P48  }; // Alt+F4 - Pentagon 48
		if(~old_Fn[5] & Fn[5]) {setwait,arch_set,arch} <= {2'b11,ARCH_P128 }; // Alt+F5 - Pentagon 128
		if(~old_Fn[6] & Fn[6]) {setwait,arch_set,arch} <= {2'b11,ARCH_P1024}; // Alt+F6 - Pentagon 1024
	end

	if(timeout) timeout <= timeout - 1'd1;
	else arch_reset <= 0;

	if(setwait && (status[12:8] == arch)) begin
		timeout <= '1;
		arch_reset <= 1;
		setwait <= 0;
		arch_set <= 0;
	end
end


//////////////////  SNAPSHOT  //////////////////

wire [211:0] snap_REG;
wire         snap_REGSet;
wire  [24:0] snap_addr;
wire   [7:0] snap_data;
wire         snap_wr;
wire         snap_reset;
wire         snap_hwset;
wire   [4:0] snap_hw;
wire  [31:0] snap_status;
wire   [2:0] snap_border;
wire   [7:0] snap_1ffd;
wire   [7:0] snap_7ffd;

snap_loader #(ARCH_ZX48, ARCH_ZX128, ARCH_ZX3, ARCH_P128) snap_loader
(
	.clk_sys(clk_sys),

	.ioctl_download(ioctl_download && ioctl_index[4:0] == 4),
	.ioctl_addr(ioctl_addr),
	.ioctl_data(ioctl_dout),
	.ioctl_wr(ioctl_wr2),
	.ioctl_wait(sna_wait),
	.snap_sna(ioctl_index[7:6]),

	.ram_ready(ram_ready),

	.REG(snap_REG),
	.REGSet(snap_REGSet),

	.addr(snap_addr),
	.dout(snap_data),
	.wr(snap_wr),

   .reset(snap_reset),
   .hwset(snap_hwset),
   .hw(snap_hw),
   .hw_ack(status[12:8]),

   .border(snap_border),
   .reg_1ffd(snap_1ffd),
   .reg_7ffd(snap_7ffd)
);


// for bridge write data, we just broadcast it to all bus devices
// for bridge read data, we have to mux it
// add your own devices here
always @(*) begin
    casex(bridge_addr)
    default: begin
        bridge_rd_data <= 0;
    end
    32'h10xxxxxx: begin
        // example
        // bridge_rd_data <= example_device_data;
        bridge_rd_data <= 0;
    end
    32'hF8xxxxxx: begin
        bridge_rd_data <= cmd_bridge_rd_data;
    end
    endcase
end


//
// host/target command handler
//
    wire            reset_n;                // driven by host commands, can be used as core-wide reset
    wire    [31:0]  cmd_bridge_rd_data;
    
// bridge host commands
// synchronous to clk_74a
    wire            status_boot_done = pll_core_locked_s; 
    wire            status_setup_done = pll_core_locked_s; // rising edge triggers a target command
    wire            status_running = reset_n; // we are running as soon as reset_n goes high

    wire            dataslot_requestread;
    wire    [15:0]  dataslot_requestread_id;
    wire            dataslot_requestread_ack = 1;
    wire            dataslot_requestread_ok = 1;

    wire            dataslot_requestwrite;
    wire    [15:0]  dataslot_requestwrite_id;
    wire    [31:0]  dataslot_requestwrite_size;
    wire            dataslot_requestwrite_ack = 1;
    wire            dataslot_requestwrite_ok = 1;

    wire            dataslot_update;
    wire    [15:0]  dataslot_update_id;
    wire    [31:0]  dataslot_update_size;
	 wire    [15:0]  dataslot_update_size_u;
	 
	 wire            dataslot_update_s;
    wire    [15:0]  dataslot_update_id_s;
    wire    [31:0]  dataslot_update_size_s;
    
    wire            dataslot_allcomplete;

    wire     [31:0] rtc_epoch_seconds;
    wire     [31:0] rtc_date_bcd;
    wire     [31:0] rtc_time_bcd;
    wire            rtc_valid;

    wire            savestate_supported;
    wire    [31:0]  savestate_addr;
    wire    [31:0]  savestate_size;
    wire    [31:0]  savestate_maxloadsize;

    wire            savestate_start;
    wire            savestate_start_ack;
    wire            savestate_start_busy;
    wire            savestate_start_ok;
    wire            savestate_start_err;

    wire            savestate_load;
    wire            savestate_load_ack;
    wire            savestate_load_busy;
    wire            savestate_load_ok;
    wire            savestate_load_err;
    
    wire            osnotify_inmenu;

// bridge target commands
// synchronous to clk_74a

    reg             target_dataslot_read;       
    reg             target_dataslot_write;
	 reg             target_dataslot_read_48;       
    reg             target_dataslot_write_48;
    reg             target_dataslot_getfile;    // require additional param/resp structs to be mapped
    reg             target_dataslot_openfile;   // require additional param/resp structs to be mapped
    
    wire            target_dataslot_ack;
	 wire            target_dataslot_ack_s;         
    wire            target_dataslot_done;
	 wire            target_dataslot_done_s;
    wire    [2:0]   target_dataslot_err;
	 wire    [2:0]   target_dataslot_err_s;
	

    reg     [15:0]  target_dataslot_id;
    reg     [31:0]  target_dataslot_slotoffset;
	 reg     [15:0]  target_dataslot_slotoffset_48;
    reg     [31:0]  target_dataslot_bridgeaddr;
    reg     [31:0]  target_dataslot_length;
    
    wire    [31:0]  target_buffer_param_struct; // to be mapped/implemented when using some Target commands
    wire    [31:0]  target_buffer_resp_struct;  // to be mapped/implemented when using some Target commands
    
// bridge data slot access
// synchronous to clk_74a

    wire    [9:0]   datatable_addr;
    wire            datatable_wren;
    wire    [31:0]  datatable_data;
    wire    [31:0]  datatable_q;
	 
	 wire					o_ack_flag;

core_bridge_cmd icb (

    .clk                ( clk_74a ),
    .reset_n            ( reset_n ),

    .bridge_endian_little   ( bridge_endian_little ),
    .bridge_addr            ( bridge_addr ),
    .bridge_rd              ( bridge_rd ),
    .bridge_rd_data         ( cmd_bridge_rd_data ),
    .bridge_wr              ( bridge_wr ),
    .bridge_wr_data         ( bridge_wr_data ),
    
    .status_boot_done       ( status_boot_done ),
    .status_setup_done      ( status_setup_done ),
    .status_running         ( status_running ),

    .dataslot_requestread       ( dataslot_requestread ),
    .dataslot_requestread_id    ( dataslot_requestread_id ),
    .dataslot_requestread_ack   ( dataslot_requestread_ack ),
    .dataslot_requestread_ok    ( dataslot_requestread_ok ),

    .dataslot_requestwrite      ( dataslot_requestwrite ),
    .dataslot_requestwrite_id   ( dataslot_requestwrite_id ),
    .dataslot_requestwrite_size ( dataslot_requestwrite_size ),
    .dataslot_requestwrite_ack  ( dataslot_requestwrite_ack ),
    .dataslot_requestwrite_ok   ( dataslot_requestwrite_ok ),

    .dataslot_update            ( dataslot_update ),
    .dataslot_update_id         ( dataslot_update_id ),
    .dataslot_update_size       ( dataslot_update_size ),
	 
	 .dataslot_update_s            ( dataslot_update_s ),
    .dataslot_update_id_s        ( dataslot_update_id_s ),
    .dataslot_update_size_s       ( dataslot_update_size_s ),
    
    .dataslot_allcomplete   ( dataslot_allcomplete ),

    .rtc_epoch_seconds      ( rtc_epoch_seconds ),
    .rtc_date_bcd           ( rtc_date_bcd ),
    .rtc_time_bcd           ( rtc_time_bcd ),
    .rtc_valid              ( rtc_valid ),
    
    .savestate_supported    ( savestate_supported ),
    .savestate_addr         ( savestate_addr ),
    .savestate_size         ( savestate_size ),
    .savestate_maxloadsize  ( savestate_maxloadsize ),

    .savestate_start        ( savestate_start ),
    .savestate_start_ack    ( savestate_start_ack ),
    .savestate_start_busy   ( savestate_start_busy ),
    .savestate_start_ok     ( savestate_start_ok ),
    .savestate_start_err    ( savestate_start_err ),

    .savestate_load         ( savestate_load ),
    .savestate_load_ack     ( savestate_load_ack ),
    .savestate_load_busy    ( savestate_load_busy ),
    .savestate_load_ok      ( savestate_load_ok ),
    .savestate_load_err     ( savestate_load_err ),

    .osnotify_inmenu        ( osnotify_inmenu ),
    
    .target_dataslot_read_s     ( target_dataslot_read ),
    .target_dataslot_write_s      ( target_dataslot_write ),
	 .target_dataslot_read_48_s     ( target_dataslot_read_48 ),
    .target_dataslot_write_48_s      ( target_dataslot_write_48 ),
    .target_dataslot_getfile_s    ( target_dataslot_getfile ),
    .target_dataslot_openfile_s   ( target_dataslot_openfile ),
    
    .target_dataslot_ack        ( target_dataslot_ack ),
	 .target_dataslot_ack_s        ( target_dataslot_ack_s ),
    .target_dataslot_done       ( target_dataslot_done ),
	 .target_dataslot_done_s       ( target_dataslot_done_s ),
    .target_dataslot_err        ( target_dataslot_err ),

    .target_dataslot_id_s         ( target_dataslot_id ),
    .target_dataslot_slotoffset_s ( target_dataslot_slotoffset ),
	 .target_dataslot_slotoffset_48_s ( target_dataslot_slotoffset_48 ),
    .target_dataslot_bridgeaddr_s ( target_dataslot_bridgeaddr ),
    .target_dataslot_length_s     ( target_dataslot_length ),

    .target_buffer_param_struct ( target_buffer_param_struct ),
    .target_buffer_resp_struct  ( target_buffer_resp_struct ),
    
    .datatable_addr         ( datatable_addr ),
    .datatable_wren         ( datatable_wren ),
    .datatable_data         ( datatable_data ),
    .datatable_q            ( datatable_q ),
	 
	 .i_clk_sync				 (ce_7mp),//clk_sys),
	 .i_write_strobe         (bridge_wstrb),
	 .i_request_flag			 (bridge_req),
	 .o_ack_flag				 (o_ack_flag)


);







////////////////////////////////////////////////////////////////////////////////////////



/*


//
// audio i2s silence generator
// see other examples for actual audio generation
//

assign audio_mclk = audgen_mclk;
assign audio_dac = audgen_dac;
assign audio_lrck = audgen_lrck;

// generate MCLK = 12.288mhz with fractional accumulator
    reg         [21:0]  audgen_accum;
    reg                 audgen_mclk;
    parameter   [20:0]  CYCLE_48KHZ = 21'd122880 * 2;
always @(posedge clk_74a) begin
    audgen_accum <= audgen_accum + CYCLE_48KHZ;
    if(audgen_accum >= 21'd742500) begin
        audgen_mclk <= ~audgen_mclk;
        audgen_accum <= audgen_accum - 21'd742500 + CYCLE_48KHZ;
    end
end

// generate SCLK = 3.072mhz by dividing MCLK by 4
    reg [1:0]   aud_mclk_divider;*/
    //wire        audgen_sclk = aud_mclk_divider[1] /* synthesis keep*/;
    /*reg         audgen_lrck_1;
always @(posedge audgen_mclk) begin
    aud_mclk_divider <= aud_mclk_divider + 1'b1;
end

// shift out audio data as I2S 
// 32 total bits per channel, but only 16 active bits at the start and then 16 dummy bits
//
    reg     [4:0]   audgen_lrck_cnt;    
    reg             audgen_lrck;
    reg             audgen_dac;
	 reg	   [32:0]  audgen_shift_l;
	 reg	   [32:0]  audgen_shift_r;
always @(negedge audgen_sclk) begin
	 if (audgen_lrck) begin
		audgen_dac <= audgen_shift_l[31];
		audgen_shift_l <= {audgen_shift_l[30:0], 1'b0};
	end else begin
		audgen_dac <= audgen_shift_r[31];
		audgen_shift_r <= {audgen_shift_r[30:0], 1'b0};
	end
    // 48khz * 64
    audgen_lrck_cnt <= audgen_lrck_cnt + 1'b1;
    if(audgen_lrck_cnt == 31) begin
        // switch channels
        audgen_lrck <= ~audgen_lrck;
		  audgen_shift_l <= {audio_l,16'h0};
		  audgen_shift_r <= {audio_r,16'h0};
        
    end 
end*/

  //wire [15:0] audio;

  reg  [15:0] audio_buffer_l = 0;
  reg  [15:0] audio_buffer_r = 0;

  // Buffer audio to have better fitting on audio route
  always @(posedge clk_aud) begin
    audio_buffer_l <= pause_z80?16'h0:audio_l;
	 audio_buffer_r <= pause_z80?16'h0:audio_r;
  end

  audio_mixer #(
      .DW(16),
      .STEREO(1)
  ) audio_mixer (
      .clk_74b  (clk_74b),
      .clk_audio(clk_aud),

      // .reset()

      .vol_att(0),
      .mix(status[3:2]),

      .is_signed(1),
      .core_l(audio_buffer_l),
      .core_r(audio_buffer_r),

      .audio_mclk(audio_mclk),
      .audio_lrck(audio_lrck),
      .audio_dac (audio_dac)
  );


////  IOCTL Emulation for MiSTer Compatibility  //////////



wire [31:0] bridgeram_rd_data_ioctl;
//1024 bytes
bram_block_dp #(
      .DATA(32),
      .ADDR(8)
  ) bridgeram (
      .a_clk (clk_74a),
      .a_wr  (bridge_wr && bridge_addr[31:28] == 4'h6),
      .a_addr(bridge_addr[31:2]),
      .a_din ({bridge_wr_data[7:0], bridge_wr_data[15:8], bridge_wr_data[23:16], bridge_wr_data[31:24]}),		
      //.a_dout(),

      .b_clk (clk_sys),
      .b_wr  (1'b0),
      .b_addr(ioctl_addr[31:2]),
      .b_din (32'h0),
      .b_dout(bridgeram_rd_data_ioctl)
  );

reg [6:0] filename_addr;
wire [31:0] filename_dout;
reg [31:0] filename_din;
reg filename_wr;
  //256 bytes
bram_block_dp #(
      .DATA(32),
      .ADDR(7)
  ) filenameram (
      .a_clk (clk_74a),
      .a_wr  (bridge_wr && bridge_addr[31:28] == 4'h7),
      .a_addr(bridge_addr[31:2]),
      .a_din ({bridge_wr_data[7:0], bridge_wr_data[15:8], bridge_wr_data[23:16], bridge_wr_data[31:24]}),		
      //.a_dout(),

      .b_clk (clk_sys),
      .b_wr  (filename_wr),
      .b_addr(filename_addr),
      .b_din (filename_din),
      .b_dout(filename_dout)
  );

 //state machine to implement MiSTer style IOCTL
 reg [4:0]  ioctl_state;
 reg	ioctl_download;
 //reg	ioctl_upload;
 reg [31:0] ioctl_size;
 reg [15:0] ioctl_id;
 
 reg [31:0] ioctl_addr;
 reg ioctl_wr;
 reg ioctl_wr_old;
 reg ioctl_wr2;
 wire ioctl_wait=rom_loaded?sna_wait:1'b0;//~ram_ready;
 wire sna_wait;
 reg [7:0] ioctl_dout;
 reg [9:0] ioctl_bytes_to_send;
 reg [7:0] ioctl_index;
 
reg [31:0] ioctl_size_req;
reg [15:0] ioctl_addr_h_req;
reg [31:0] ioctl_addr_l_req;
reg [47:0] ioctl_addr_offset;
reg [15:0] ioctl_id_req;
reg ioctl_download_req;
reg ioctl_upload_req;
reg ioctl_old_download_req;
reg ioctl_old_upload_req;
reg ioctl_complete;

//reg [31:0] dataslot_update_size_latch;
//reg [15:0] dataslot_update_id_latch;
//reg old_dataslot_update;

always @(posedge clk_sys) begin				//as IOCTL is running at a slower clk speed
	ioctl_wr2<=ioctl_wr & ~ioctl_wr_old;	//ensure write signal only asserted for one cycle in clk_sys domain
	ioctl_wr_old<=ioctl_wr;
end

initial begin
	ioctl_addr<=32'h0;
	ioctl_id<=16'h0;
	rom_loaded<=0;
	ioctl_state<=0;
	ioctl_download_req<=0;
	ioctl_old_download_req<=0;
	ioctl_old_upload_req<=0;
	ioctl_download<=0;
end

reg rom_loaded;
reg bridge_req;
reg bridge_wstrb;
always @(posedge ce_7mp or negedge reset_n) begin //clk_sys) begin
if (!reset_n) begin
	ioctl_addr<=32'h0;
	ioctl_id<=16'h0;
	ioctl_state<=0;
	ioctl_old_download_req<=0;
	ioctl_old_upload_req<=0;
	ioctl_download<=0;
	ioctl_complete<=0;
end else if (!ioctl_wait) begin		//make sure core isn't asserting wait							
	case (ioctl_state)
		'd0:	begin		//Wait for a download instruction
			bridge_wstrb<=0;
			bridge_req<=~bridge_req;	//while idle -> toggle request line to check for updates
			//else if (dataslot_update_s) begin // & rom_loaded) begin				
			if (ioctl_download_req & ~ioctl_old_download_req) begin	//rising edge of download request
				bridge_req<=0;
				
				ioctl_download<=1;		
				ioctl_size<=ioctl_size_req;	//store file size
				ioctl_id<=ioctl_id_req;			//store file ID
				ioctl_addr<=0;				
				ioctl_wr<=0;								//byte not ready yet
				if (ioctl_addr_l_req==32'hFFFFFFFF) begin
					ioctl_state<=11;		//loading filename
					ioctl_addr_offset<=48'h0;						//set address to start
				end
				else begin
					ioctl_state<=1;		//load actual data
					ioctl_addr_offset<={ioctl_addr_h_req,ioctl_addr_l_req};						//set address to start					
				end
			end
		/*	if (ioctl_upload_req & ~ioctl_old_upload_req) begin	//rising edge of upload request
				bridge_req<=0;
								
				ioctl_size<=ioctl_size_req;	//store file size
				ioctl_id<=ioctl_id_req;			//store file ID
				ioctl_addr<=0;				
				ioctl_wr<=0;								//byte not ready yet
				if (ioctl_addr_l_req==32'hFFFFFFFE) begin
					ioctl_state<=16;		//saving config
					ioctl_addr_offset<=48'h0;						//set address to start
				end
				else begin
					ioctl_state<=0;		//for now - just silently fail
					//ioctl_addr_offset<={ioctl_addr_h_req,ioctl_addr_l_req};						//set address to start					
					//ioctl_download<=1;				//don't need a separate upload flag, just reuse the download one	
				end
			end*/
		end
		'd1: begin	//load next block of data into bridgeram
				target_dataslot_id<=ioctl_id;
				target_dataslot_slotoffset<=ioctl_addr_offset[31:0];//ioctl_addr;									//set address offset into file
				target_dataslot_slotoffset_48<=ioctl_addr_offset[47:32];//ioctl_addr;									//set address offset into file
				//if (ioctl_id=='h100) target_dataslot_bridgeaddr<=32'h70000000; else target_dataslot_bridgeaddr<=32'h60000000;										//set address of bridgeram
				target_dataslot_bridgeaddr<=32'h60000000;
				if (ioctl_size>=32'd1024) begin
					target_dataslot_length<=32'd1024;  //set length to bridgeram size unless remaining file is smaller
					ioctl_size<=ioctl_size-32'd1024;
					ioctl_bytes_to_send<=10'd1023;
				end
				else
				begin
					target_dataslot_length<=ioctl_size;
					ioctl_bytes_to_send<=ioctl_size[9:0]-10'd1;
					ioctl_size<=32'd0;
				end				
				if (ioctl_id=='h200) target_dataslot_read<=1; else target_dataslot_read_48<=1;	//tell bridge to read bytes into BRAM( does't use large file size for ROM)
				bridge_req<=1;	
				bridge_wstrb<=1;				//write data						
				ioctl_state<=2;
		end		
		'd2: begin		//extra cycle to ensure request is synced			
				ioctl_state<=3;			
		end		
		'd3: begin		//wait until bridge acknowledges cmd
			bridge_wstrb<=0;				
			bridge_req<=~bridge_req;	//while idle -> toggle request line to check for updates			
			if (target_dataslot_ack_s) begin				
				ioctl_state<=4;
				bridge_req<=0;	
			   //target_dataslot_read<=0;
				//if (ioctl_id=='h200) 
				target_dataslot_read<=0;
				target_dataslot_read_48<=0;
			end
		end
		'd4: begin		//wait until data read is finished						
			bridge_req<=~bridge_req;	//while idle -> toggle request line to check for updates			
			if (target_dataslot_done_s) begin				
				ioctl_state<=5;
				bridge_req<=0;							
			end
		end
		'd5: begin		//extra cycle to stablise read before writing data			
			//if (ioctl_id=='h100) ioctl_state<=11; else 
			ioctl_state<=6;
		end				
		'd6: begin		//start sending data via ioctl_dout						
				case (ioctl_addr[1:0])
					2'b11:	ioctl_dout<=bridgeram_rd_data_ioctl[31:24];
					2'b10:	ioctl_dout<=bridgeram_rd_data_ioctl[23:16];
					2'b01:	ioctl_dout<=bridgeram_rd_data_ioctl[15:8];
					default:	ioctl_dout<=bridgeram_rd_data_ioctl[7:0];
				endcase				
				if (ram_ready) ioctl_state<=7;	
		end
		'd7: begin		//extra cycle to stablise read before updating address
			ioctl_wr<=1;			
			ioctl_state<=8;
		end
		'd8: begin		//extra cycle to stablise read before updating address
			ioctl_wr<=1;			
			ioctl_state<=9;
		end		
		'd9: begin		//update address			
			ioctl_wr<=0;						
				ioctl_addr<=ioctl_addr+32'd1;
				ioctl_addr_offset<=ioctl_addr_offset+48'd1;
				if (ioctl_bytes_to_send==0) begin		//all bytes from last read have sent
					if (ioctl_size==0) begin	//done
						ioctl_download<=0;		//turn off download flag
						ioctl_complete<=1;		//flag complete
						ioctl_state<=10;			//wait for soft cpu to ack						
						/*if (ioctl_id==16'h200) begin
							rom_loaded<=1;
							//ioctl_addr<=32'hffffffff;
						end*/
					end
					else
					begin
						ioctl_state<=1;		//read next block
					end
				end
				else
				begin
					ioctl_state<=5;
					ioctl_bytes_to_send<=ioctl_bytes_to_send-10'd1;
				end			
			end
		'd10: begin		//wait for soft CPU ack
			if (pico_ioctl_complete_ack) begin
				ioctl_complete<=0;		//clear complete
				ioctl_state<=0;			//go back to IDLE						
			end
		end
		'd11: begin	//load filename bridgeram
				target_dataslot_id<=ioctl_id;				
				target_buffer_resp_struct<=32'h70000000;
				target_dataslot_getfile<=1;	//tell bridge to read bytes into BRAM
				bridge_req<=1;	
				bridge_wstrb<=1;				//write data						
				ioctl_state<=12;				
		end		
		'd12: begin		//extra cycle to ensure request is synced			
				ioctl_state<=13;			
		end						
		'd13: begin		//wait until bridge acknowledges cmd
			bridge_wstrb<=0;				
			bridge_req<=~bridge_req;	//while idle -> toggle request line to check for updates			
			if (target_dataslot_ack_s) begin				
				ioctl_state<=14;
				bridge_req<=0;	
			   target_dataslot_getfile<=0;				
			end
		end
		'd14: begin		//wait until data read is finished						
			bridge_req<=~bridge_req;	//while idle -> toggle request line to check for updates			
			if (target_dataslot_done_s) begin				
				ioctl_state<=15;
				bridge_req<=0;							
			end
		end
		'd15: begin		//extra cycle to stablise read before writing data			
			ioctl_download<=0;		//turn off download flag
			ioctl_complete<=1;		//flag complete
			ioctl_state<=10;
		end				
	/*	'd16: begin		//set filename for config output
				target_dataslot_id<=ioctl_id;				
				target_buffer_param_struct<=32'h70000000;
				target_dataslot_openfile<=1;	//tell bridge to read bytes into BRAM
				bridge_req<=1;	
				bridge_wstrb<=1;				//write data						
				ioctl_state<=17;				
		end
		'd17: begin		//wait until bridge acknowledges cmd
			bridge_wstrb<=0;				
			bridge_req<=~bridge_req;	//while idle -> toggle request line to check for updates			
			if (target_dataslot_ack_s) begin				
				ioctl_complete<=1;
				ioctl_state<=10;//18;
				bridge_req<=0;	
			   target_dataslot_openfile<=0;				
			end
		end*/
		/*'d18: begin		//extra cycle to stablise read before writing data			
			ioctl_download<=0;		//turn off download flag
			ioctl_complete<=1;		//flag complete
			ioctl_state<=10;
		end				*/
	endcase
	ioctl_old_download_req<=ioctl_download_req;
	ioctl_old_upload_req<=ioctl_upload_req;
end
end

    
    



    
endmodule
