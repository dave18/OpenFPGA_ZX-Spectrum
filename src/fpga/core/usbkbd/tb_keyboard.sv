`timescale 1ns/1ps
// Standalone simulation testbench for keyboard.sv's direct USB HID -> Spectrum
// matrix decode, exactly as wired in core_top.sv, used to verify press/hold/
// release behaviour without needing real Pocket hardware. No PS/2 bridge, no
// clk_74a - this whole path runs on clk_sys.
// Run with: iverilog -g2012 -o tb.vvp tb_keyboard.sv keyboard.sv && vvp tb.vvp

module tb_keyboard;

    reg clk_sys = 0;
    always #4.464 clk_sys = ~clk_sys; // 8.928ns period, matches clk_sys

    reg reset = 1;
    reg [7:0] hid_mod = 0, hid_sc1 = 0, hid_sc2 = 0, hid_sc3 = 0, hid_sc4 = 0, hid_sc5 = 0, hid_sc6 = 0;
    reg [15:0] addr = 16'hFFFF;

    integer errors = 0;

    wire [4:0] hw_key_data;
    wire [11:1] Fn_hw;
    wire  [2:0] mod_hw;

    keyboard kbd (
        .reset(reset), .clk_sys(clk_sys),
        .hid_mod(hid_mod), .hid_sc1(hid_sc1), .hid_sc2(hid_sc2), .hid_sc3(hid_sc3),
        .hid_sc4(hid_sc4), .hid_sc5(hid_sc5), .hid_sc6(hid_sc6),
        .recreated_zx(1'b0), .ghosting(1'b0),
        .addr(addr), .key_data(hw_key_data), .Fn(Fn_hw), .mod(mod_hw)
    );

    task set_key(input [7:0] sc);
        begin hid_sc1 = sc; hid_sc2 = 0; hid_sc3 = 0; hid_sc4 = 0; hid_sc5 = 0; hid_sc6 = 0; hid_mod = 0; end
    endtask

    task set_idle;
        begin hid_sc1 = 0; hid_sc2 = 0; hid_sc3 = 0; hid_sc4 = 0; hid_sc5 = 0; hid_sc6 = 0; hid_mod = 0; end
    endtask

    task select_row(input integer row);
        begin
            addr = 16'hFFFF;
            addr[8+row] = 1'b0;
            #1; // let the purely-combinational key_data settle before reading it
        end
    endtask

    task check(input cond, input [255:0] msg);
        begin
            if (cond) $display("PASS: %0s", msg);
            else begin
                $display("FAIL: %0s", msg);
                errors = errors + 1;
            end
        end
    endtask

    integer i;
    initial begin
        set_idle;
        select_row(1); // row1 holds 'a' (col0) and 's' (col1)

        repeat (20) @(posedge clk_sys);
        reset = 0;
        repeat (20) @(posedge clk_sys);

        // 1) Idle: 'a' bit should read released (1)
        check(hw_key_data[0] === 1'b1, "idle: 'a' bit released before any keypress");

        // 2) Press 'a' (USB HID 0x04 -> row1 col0)
        set_key(8'h04);
        repeat (20) @(posedge clk_sys);
        check(hw_key_data[0] === 1'b0, "press 'a': row1 col0 goes low (pressed)");

        // 3) Long hold - nothing should change while held (this whole design
        //    is level-based, re-derived every cycle - there's no repeat/
        //    typematic mechanism or FIFO left to lose a write).
        for (i = 0; i < 40; i = i + 1) begin
            repeat (5000) @(posedge clk_sys);
            check(hw_key_data[0] === 1'b0, "hold 'a': still pressed mid-hold");
        end

        // 4) Release
        set_idle;
        repeat (20) @(posedge clk_sys);
        check(hw_key_data[0] === 1'b1, "release 'a': row1 col0 goes back high (released)");

        // 5) Press a different key ('b', USB 0x05 -> row7 col4) and confirm
        //    'a' does NOT appear stuck/pressed from stale state.
        select_row(7);
        set_key(8'h05);
        repeat (20) @(posedge clk_sys);
        check(hw_key_data[4] === 1'b0, "press 'b': row7 col4 goes low (pressed)");
        select_row(1);
        repeat (10) @(posedge clk_sys);
        check(hw_key_data[0] === 1'b1, "'a' still reads released while 'b' is held");

        set_idle;
        repeat (20) @(posedge clk_sys);
        select_row(7);
        check(hw_key_data[4] === 1'b1, "release 'b': row7 col4 goes back high (released)");

        // 6) Rapid sequential taps of different keys (s, then a) with short holds
        select_row(1);
        set_key(8'h16); // 's' -> row1 col1
        repeat (20) @(posedge clk_sys);
        check(hw_key_data[1] === 1'b0, "rapid: 's' pressed");
        set_idle;
        repeat (20) @(posedge clk_sys);
        check(hw_key_data[1] === 1'b1, "rapid: 's' released");
        set_key(8'h04); // 'a' again
        repeat (20) @(posedge clk_sys);
        check(hw_key_data[0] === 1'b0, "rapid: 'a' pressed again");
        set_idle;
        repeat (20) @(posedge clk_sys);
        check(hw_key_data[0] === 1'b1, "rapid: 'a' released again");

        if (errors == 0) $display("\n==== ALL CHECKS PASSED ====");
        else $display("\n==== %0d CHECK(S) FAILED ====", errors);

        $finish;
    end

    initial begin
        #20000000; // 20ms safety timeout
        $display("FAIL: simulation timeout - something hung");
        $finish;
    end

endmodule
