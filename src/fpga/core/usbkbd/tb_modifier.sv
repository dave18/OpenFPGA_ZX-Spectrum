`timescale 1ns/1ps
// Modifier-key testbench (Shift+letter) - CAPS SHIFT continuously following
// the live shift state, in both press orders, holds, and partial releases.

module tb_modifier;

    reg clk_sys = 0;
    always #4.464 clk_sys = ~clk_sys;

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

    // mods: USB HID modifier byte, bit1=LShift; sc1: scancode slot 1
    task set_mod_key(input [7:0] mods, input [7:0] sc1);
        begin hid_mod = mods; hid_sc1 = sc1; hid_sc2 = 0; hid_sc3 = 0; hid_sc4 = 0; hid_sc5 = 0; hid_sc6 = 0; end
    endtask

    task select_row(input integer row);
        begin
            addr = 16'hFFFF;
            addr[8+row] = 1'b0;
            #1;
        end
    endtask

    task check(input cond, input [255:0] msg);
        begin
            if (cond) $display("PASS: %0s", msg);
            else begin $display("FAIL: %0s", msg); errors = errors + 1; end
        end
    endtask

    initial begin
        repeat (20) @(posedge clk_sys);
        reset = 0;
        repeat (20) @(posedge clk_sys);

        // CAPS SHIFT bit is keys[0][0], row0 col0
        select_row(0);
        check(hw_key_data[0] === 1'b1, "idle: CAPS SHIFT released");

        // Press left shift alone
        set_mod_key(8'h02, 8'h00);
        repeat (20) @(posedge clk_sys);
        select_row(0);
        check(hw_key_data[0] === 1'b0, "shift alone: CAPS SHIFT pressed");

        // Hold shift, also press 'a' (row1 col0) - shift+a combo
        set_mod_key(8'h02, 8'h04);
        repeat (20) @(posedge clk_sys);
        select_row(0); check(hw_key_data[0] === 1'b0, "shift+a: CAPS SHIFT still pressed");
        select_row(1); check(hw_key_data[0] === 1'b0, "shift+a: 'a' pressed");

        // hold for a while
        repeat (5000) @(posedge clk_sys);
        select_row(0); check(hw_key_data[0] === 1'b0, "shift+a hold: CAPS SHIFT still pressed");
        select_row(1); check(hw_key_data[0] === 1'b0, "shift+a hold: 'a' still pressed");

        // Release 'a' only, keep shift held (like releasing a direction but holding run)
        set_mod_key(8'h02, 8'h00);
        repeat (20) @(posedge clk_sys);
        select_row(1); check(hw_key_data[0] === 1'b1, "release a, keep shift: 'a' released");
        select_row(0); check(hw_key_data[0] === 1'b0, "release a, keep shift: CAPS SHIFT still pressed");

        // Now release shift too
        set_mod_key(8'h00, 8'h00);
        repeat (20) @(posedge clk_sys);
        select_row(0); check(hw_key_data[0] === 1'b1, "release shift: CAPS SHIFT released");

        // Press 'a' then shift (reverse order), hold both, release shift first this time
        set_mod_key(8'h00, 8'h04);
        repeat (20) @(posedge clk_sys);
        select_row(1); check(hw_key_data[0] === 1'b0, "a then shift: 'a' pressed");
        set_mod_key(8'h02, 8'h04);
        repeat (20) @(posedge clk_sys);
        select_row(0); check(hw_key_data[0] === 1'b0, "a then shift: CAPS SHIFT pressed");
        select_row(1); check(hw_key_data[0] === 1'b0, "a then shift: 'a' still pressed");

        set_mod_key(8'h00, 8'h04); // release shift, keep a
        repeat (20) @(posedge clk_sys);
        select_row(0); check(hw_key_data[0] === 1'b1, "release shift keep a: CAPS SHIFT released");
        select_row(1); check(hw_key_data[0] === 1'b0, "release shift keep a: 'a' still pressed");

        set_mod_key(8'h00, 8'h00);
        repeat (20) @(posedge clk_sys);
        select_row(1); check(hw_key_data[0] === 1'b1, "final release: 'a' released");

        // Right shift alone must behave identically to left shift
        set_mod_key(8'h20, 8'h00); // RShift
        repeat (20) @(posedge clk_sys);
        select_row(0); check(hw_key_data[0] === 1'b0, "right shift alone: CAPS SHIFT pressed");
        set_mod_key(8'h00, 8'h00);
        repeat (20) @(posedge clk_sys);
        select_row(0); check(hw_key_data[0] === 1'b1, "right shift release: CAPS SHIFT released");

        // Ctrl -> SYMBOL SHIFT (row7 col1)
        set_mod_key(8'h01, 8'h04); // LCtrl + a
        repeat (20) @(posedge clk_sys);
        select_row(7); check(hw_key_data[1] === 1'b0, "ctrl+a: SYMBOL SHIFT pressed");
        select_row(1); check(hw_key_data[0] === 1'b0, "ctrl+a: 'a' pressed");
        set_mod_key(8'h00, 8'h00);
        repeat (20) @(posedge clk_sys);
        select_row(7); check(hw_key_data[1] === 1'b1, "ctrl+a release: SYMBOL SHIFT released");

        if (errors == 0) $display("\n==== ALL MODIFIER CHECKS PASSED ====");
        else $display("\n==== %0d MODIFIER CHECK(S) FAILED ====", errors);
        $finish;
    end

    initial begin
        #20000000;
        $display("FAIL: simulation timeout");
        $finish;
    end

endmodule
