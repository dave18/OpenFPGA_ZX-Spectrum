`timescale 1ns/1ps
// Validates the docked-mouse logic that lives directly in core_top.sv
// (not inside mouse.v) - kept as a self-contained copy of those wire/
// always-block expressions rather than pulling in the whole of
// core_top.sv, since this has no other dependencies of its own. If
// core_top.sv's mouse wiring changes, update this file's copy to match.
// Covers two things: the clk_74a-side report-type gating
// (is_mouse_74a/mouse_counter_74a - gates mouse.v's accumulator so an
// attached gamepad's unrelated cont4 traffic is never misread as mouse
// movement), and the clk_sys-side Kempston Mouse port decode
// (mouse_reg_sel/mouse_sel/mouse_data - address decode, active-low
// inversion, left/right swap). mouse.v's own accumulator logic is
// covered separately by tb_mouse.sv.
//
//   iverilog -g2012 -o tb.vvp tb_mouse_gating.sv
//   vvp tb.vvp
module tb_mouse_gating;

    reg [31:0] cont4_key_s = 0; // already-synchronized (clk_74a domain in core_top.sv - synch_3 itself not modeled, just what reads it)
    reg [31:0] cont4_joy_s = 0;
    reg [15:0] status35_34 = 0; // maps to status[35:34]
    reg        status35    = 0; // maps to status[35] (button swap)
    reg [15:0] addr = 0; // full Z80 address bus - addr[7:0] is the port's low byte, addr[10:8] the register-select bits, NOT the same 8 bits doing double duty
    reg [7:0]  mouse_dx_s = 0, mouse_dy_s = 0; // stand-in for the clk_sys-synchronized copy of mouse.v's outputs
    reg [2:0]  mouse_buttons_s = 0;

    // --- clk_74a side: gates mouse.v's accumulator ---
    wire       is_mouse_74a = cont4_key_s[31:28] == 4'h5;
    wire [15:0] mouse_counter_74a = is_mouse_74a ? cont4_key_s[15:0] : 16'h0;
    // The real HID delta byte sits at bits[15:8], not bits[7:0] - same
    // "little endian byte order" convention already confirmed working
    // for the keyboard's modifier byte (cont3_key[15:0] - see
    // apf2hid.sv's usb_kb_mod, which reads the HIGH byte of that field).
    // Two wrong versions preceded this one, both confirmed wrong on real
    // hardware: treating the full 16 bits as signed (huge positive
    // numbers from negative deltas, movement twitched near a stuck
    // saturation extreme) and reading bits[7:0] - the reserved/always-
    // zero byte - as the delta (constant zero, no movement at all,
    // buttons unaffected since they're a different field).
    wire signed [15:0] mouse_dx_74a = {{8{cont4_joy_s[15]}}, cont4_joy_s[15:8]};
    wire  [2:0] mouse_buttons_74a   = cont4_joy_s[18:16];

    // --- clk_sys side: Kempston Mouse I/O port decode ---
    wire       mouse_en      = |status35_34;
    wire       mouse_reg_sel = (addr[10:8] == 3'b011) || (addr[10:8] == 3'b111) || (addr[9:8] == 2'b10);
    wire       mouse_sel     = mouse_en & (addr[7:0] == 8'hDF) & mouse_reg_sel;
    reg  [7:0] mouse_data;
    always @* begin
        casex (addr[10:8])
            3'b011:  mouse_data = mouse_dx_s;
            3'b111:  mouse_data = mouse_dy_s;
            3'bX10:  mouse_data = ~{5'b00000, mouse_buttons_s[2], mouse_buttons_s[~status35], mouse_buttons_s[status35]};
            default: mouse_data = 8'hFF;
        endcase
    end

    integer errors = 0;
    task check(input cond, input [255:0] msg);
        begin
            if (cond) $display("PASS: %0s", msg);
            else begin $display("FAIL: %0s", msg); errors = errors + 1; end
        end
    endtask

    initial begin
        // --- clk_74a-side gating ---
        // Not a mouse report (e.g. a regular gamepad in cont4) - counter
        // must be held at 0 regardless of what garbage cont4_key_s[15:0]
        // contains.
        cont4_key_s = {4'h0, 12'hABC, 16'hBEEF}; #1; // type=0, garbage elsewhere
        check(is_mouse_74a === 1'b0, "non-mouse type: is_mouse_74a false");
        check(mouse_counter_74a === 16'h0, "non-mouse type: counter held at 0 despite garbage cont4_key_s[15:0]");

        cont4_key_s = {4'h5, 12'h000, 16'h1234}; #1;
        check(is_mouse_74a === 1'b1, "mouse type (0x5): is_mouse_74a true");
        check(mouse_counter_74a === 16'h1234, "mouse type: counter passes through cont4_key_s[15:0]");

        cont4_joy_s = {13'h0, 1'b1 /*middle*/, 1'b0 /*right*/, 1'b1 /*left*/, 16'h0}; #1;
        check(mouse_buttons_74a === 3'b101, "button extraction: {middle,right,left} = 101");

        // --- the real bug (v1): full 16-bit signed misreads a negative
        // delta as huge-positive when the low/reserved byte is 0 ---
        cont4_joy_s = {16'h0, 16'hFB00}; #1; // -5 as a raw byte (0xFB) in the HIGH byte, low/reserved byte 0x00
        check(mouse_dx_74a === -16'sd5, "negative delta (high byte 0xFB, low byte 0x00): reads as -5, not +251");
        cont4_joy_s = {16'h0, 16'h0700}; #1; // +7, high byte only, low/reserved byte 0
        check(mouse_dx_74a === 16'sd7, "positive delta (high byte 0x07): reads as +7");
        cont4_joy_s = {16'h0, 16'h8000}; #1; // -128 (most negative 8-bit value), low/reserved byte 0
        check(mouse_dx_74a === -16'sd128, "most-negative 8-bit delta (0x80): reads as -128, not a huge positive number");
        // --- the real bug (v2): reading the WRONG byte (low, reserved,
        // always-zero) instead of the high byte reads as a constant
        // zero regardless of actual movement ---
        cont4_joy_s = {16'h0, 16'hFBAB}; #1; // same -5 in the high byte, but low/reserved byte full of garbage
        check(mouse_dx_74a === -16'sd5, "low/reserved byte is ignored entirely - garbage there doesn't change the result");

        // --- clk_sys-side port decode ---
        // Mouse disabled in menu - mouse_sel must never assert even at
        // the right address with a valid register select
        status35_34 = 0;
        addr = 16'hFBDF; #1; // X register address
        check(mouse_en === 1'b0, "menu disabled (00): mouse_en false");
        check(mouse_sel === 1'b0, "menu disabled: mouse_sel never asserts even at the right address");

        status35_34 = 1; #1;
        check(mouse_en === 1'b1, "menu Kempston L/R (01): mouse_en true");

        // addr[10:8] register select - #FADF/#FBDF/#FFDF map to
        // buttons/X/Y; anything else at the 0xDF low byte must not select
        addr = 16'hFBDF; #1; // addr[10:8] = 011 -> X
        check(mouse_sel === 1'b1, "0xFBDF (X register): mouse_sel asserts");
        addr = 16'hFFDF; #1; // addr[10:8] = 111 -> Y
        check(mouse_sel === 1'b1, "0xFFDF (Y register): mouse_sel asserts");
        addr = 16'hFADF; #1; // addr[10:8] = 010 -> buttons
        check(mouse_sel === 1'b1, "0xFADF (buttons register): mouse_sel asserts");
        addr = 16'hF9DF; #1; // addr[10:8] = 001 -> not a valid mouse register
        check(mouse_sel === 1'b0, "0xF9DF (not a mouse register): mouse_sel deasserts");

        // Wrong low byte - mouse_sel must not assert even when enabled
        addr = 16'h001F; #1; // the joystick port, not the mouse port
        check(mouse_sel === 1'b0, "enabled but wrong low byte (0x1F not 0xDF): mouse_sel deasserts");

        // --- data mux + button swap ---
        mouse_dx_s = 8'h42;
        mouse_dy_s = 8'h99;
        mouse_buttons_s = 3'b001; // left only
        addr = 16'hFBDF; #1; check(mouse_data === 8'h42, "X register reads mouse_dx_s");
        addr = 16'hFFDF; #1; check(mouse_data === 8'h99, "Y register reads mouse_dy_s");
        status35 = 0;
        addr = 16'hFADF; #1; check(mouse_data === 8'hFE, "buttons (no swap): left pressed -> bit0 low, rest high");
        status35 = 1;
        addr = 16'hFADF; #1; check(mouse_data === 8'hFD, "buttons (swap on): physical left now reads as bit1 (right)");

        if (errors == 0) $display("\n==== ALL GATING CHECKS PASSED ====");
        else $display("\n==== %0d GATING CHECK(S) FAILED ====", errors);
        $finish;
    end
endmodule
