`timescale 1ns/1ps
// Covers the parts of keyboard.sv's direct HID->matrix decode that don't fit
// tb_keyboard/tb_multikey/tb_modifier: punctuation keys that force SYMBOL
// SHIFT and pick a different matrix target depending on live shift state,
// arrow/backspace/caps-lock/escape keys that force CAPS SHIFT (and clear
// SYMBOL even if ctrl is also held), the Tab "G mode" combo, and F-key
// level detection. Does not run the full ~4s "LOAD "" ENTER" auto-type
// macro (46 steps * 7,000,000 clk_sys cycles is impractically slow to
// simulate) - just confirms triggering it doesn't hang or propagate X.

module tb_special;

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

    task clear_all;
        begin hid_mod=0; hid_sc1=0; hid_sc2=0; hid_sc3=0; hid_sc4=0; hid_sc5=0; hid_sc6=0; end
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

        // --- arrow key forces CAPS, clears SYMBOL, even with ctrl also held ---
        clear_all; hid_mod = 8'h10; hid_sc1 = 8'h50; // RCtrl + Left arrow
        repeat (10) @(posedge clk_sys);
        select_row(0); check(hw_key_data[0] === 1'b0, "left arrow: CAPS SHIFT forced on");
        select_row(7); check(hw_key_data[1] === 1'b1, "left arrow: SYMBOL SHIFT forced off despite ctrl held");
        select_row(3); check(hw_key_data[4] === 1'b0, "left arrow: '5' position pressed (CAPS 5)");
        clear_all;
        repeat (10) @(posedge clk_sys);
        select_row(0); check(hw_key_data[0] === 1'b1, "left arrow release: CAPS SHIFT released");
        select_row(3); check(hw_key_data[4] === 1'b1, "left arrow release: '5' position released");

        // --- backspace -> CAPS+0 ---
        clear_all; hid_sc1 = 8'h2A; // Backspace
        repeat (10) @(posedge clk_sys);
        select_row(0); check(hw_key_data[0] === 1'b0, "backspace: CAPS SHIFT forced on");
        select_row(4); check(hw_key_data[0] === 1'b0, "backspace: '0' position pressed");
        clear_all;
        repeat (10) @(posedge clk_sys);

        // --- punctuation forces SYMBOL, target depends on live shift ---
        clear_all; hid_sc1 = 8'h36; // , unshifted
        repeat (10) @(posedge clk_sys);
        select_row(7); check(hw_key_data[1] === 1'b0, ", : SYMBOL SHIFT forced on");
        select_row(7); check(hw_key_data[2] === 1'b0, ", : unshifted target (row7 col2)");
        clear_all;
        repeat (10) @(posedge clk_sys);

        clear_all; hid_mod = 8'h02; hid_sc1 = 8'h36; // Shift+, = <
        repeat (10) @(posedge clk_sys);
        select_row(2); check(hw_key_data[4] === 1'b0, "< : shifted target (row2 col4)");
        select_row(0); check(hw_key_data[0] === 1'b1, "< : CAPS SHIFT stays off despite physical shift held (SYMBOL group wins)");
        clear_all;
        repeat (10) @(posedge clk_sys);

        // --- [ and ] are not shift-dependent: always give ( and ) ---
        clear_all; hid_sc1 = 8'h2F; // [
        repeat (10) @(posedge clk_sys);
        select_row(4); check(hw_key_data[2] === 1'b0, "[ : gives '(' (row4 col2) unshifted");
        clear_all; hid_mod = 8'h02; hid_sc1 = 8'h2F; // Shift+[
        repeat (10) @(posedge clk_sys);
        select_row(4); check(hw_key_data[2] === 1'b0, "{ : still gives '(' (row4 col2) shifted too");
        clear_all;
        repeat (10) @(posedge clk_sys);

        // --- Tab / "G mode": asserts CAPS SHIFT and '9' directly ---
        clear_all; hid_sc1 = 8'h2B; // Tab
        repeat (10) @(posedge clk_sys);
        select_row(0); check(hw_key_data[0] === 1'b0, "Tab: CAPS SHIFT pressed directly");
        select_row(4); check(hw_key_data[1] === 1'b0, "Tab: '9' position pressed directly");
        clear_all;
        repeat (10) @(posedge clk_sys);
        select_row(0); check(hw_key_data[0] === 1'b1, "Tab release: CAPS SHIFT released");
        select_row(4); check(hw_key_data[1] === 1'b1, "Tab release: '9' position released");

        // --- F-keys: level detection, independent of the Spectrum matrix ---
        clear_all; hid_sc1 = 8'h43; // F10
        repeat (10) @(posedge clk_sys);
        check(Fn_hw[10] === 1'b1, "F10 detected as held");
        select_row(0); check(hw_key_data === 5'b11111, "F10 held: no Spectrum matrix key affected");
        clear_all;
        repeat (10) @(posedge clk_sys);
        check(Fn_hw[10] === 1'b0, "F10 released");

        clear_all; hid_sc1 = 8'h3A; // F1
        repeat (10) @(posedge clk_sys);
        check(Fn_hw[1] === 1'b1, "F1 detected as held");
        clear_all;
        repeat (10) @(posedge clk_sys);
        check(Fn_hw[1] === 1'b0, "F1 released");

        // --- triggering the auto-type macro must not hang or propagate X ---
        clear_all; hid_sc1 = 8'h43; // F10 press
        repeat (5) @(posedge clk_sys);
        clear_all; // release F10
        repeat (500) @(posedge clk_sys);
        select_row(5); check(^hw_key_data !== 1'bx, "post-macro-trigger: key_data still valid (no X propagation)");

        if (errors == 0) $display("\n==== ALL SPECIAL-DECODE CHECKS PASSED ====");
        else $display("\n==== %0d SPECIAL-DECODE CHECK(S) FAILED ====", errors);
        $finish;
    end

    initial begin
        #500000;
        $display("FAIL: simulation timeout");
        $finish;
    end

endmodule
