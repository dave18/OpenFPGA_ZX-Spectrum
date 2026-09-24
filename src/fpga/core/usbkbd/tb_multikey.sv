`timescale 1ns/1ps
// Multi-key (simultaneous key hold) testbench - the actual game-controls
// use case: multiple keys held at once, one released while others remain,
// and a HID host reordering/compacting its scancode slots as keys come and
// go. That last scenario used to be a real bug class (see git history and
// src/fpga/core/usbkbd/README.md) when this design tracked state per HID
// slot; it cannot recur now - keyboard.sv re-derives the whole matrix from
// all 6 slots every cycle, with no per-slot state to go stale - but the
// "release one of two held keys, the other must stay pressed" checks are
// kept here as a permanent regression guard.

module tb_multikey;

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

    // USB 0x04='a'->row1 col0, 0x1A='w'->row2 col1, 0x16='s'->row1 col1, 0x07='d'->row1 col2
    task set_keys(input [7:0] sc1, input [7:0] sc2);
        begin hid_sc1 = sc1; hid_sc2 = sc2; hid_sc3 = 0; hid_sc4 = 0; hid_sc5 = 0; hid_sc6 = 0; hid_mod = 0; end
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
            else begin $display("FAIL: %0s", msg); errors = errors + 1; end
        end
    endtask

    initial begin
        repeat (20) @(posedge clk_sys);
        reset = 0;
        repeat (20) @(posedge clk_sys);

        // Hold 'a' (movement) + 'd' (movement) simultaneously - diagonal-style combo
        set_keys(8'h04, 8'h07); // a in slot1, d in slot2
        repeat (20) @(posedge clk_sys);
        select_row(1); check(hw_key_data[0] === 1'b0, "combo: 'a' pressed (slot1)");
        select_row(1); check(hw_key_data[2] === 1'b0, "combo: 'd' pressed (slot2)");

        // Hold both for a while (simulate sustained diagonal movement)
        repeat (5000) @(posedge clk_sys);
        select_row(1); check(hw_key_data[0] === 1'b0, "combo hold: 'a' still pressed");
        select_row(1); check(hw_key_data[2] === 1'b0, "combo hold: 'd' still pressed");

        // Release 'a' only - a HID host may compact: 'd' shifts from slot2 to slot1
        set_keys(8'h07, 8'h00); // d moves to slot1, slot2 now empty
        repeat (20) @(posedge clk_sys);
        select_row(1); check(hw_key_data[0] === 1'b1, "release 'a' (slot reorder): 'a' bit released");
        select_row(1); check(hw_key_data[2] === 1'b0, "release 'a' (slot reorder): 'd' bit STILL pressed after moving slots");

        // Now release 'd' too
        set_keys(8'h00, 8'h00);
        repeat (20) @(posedge clk_sys);
        select_row(1); check(hw_key_data[2] === 1'b1, "release 'd': released cleanly");

        // Rapid combo: press w+a together, hold, release both together, then press s+d
        set_keys(8'h1A, 8'h04); // w slot1, a slot2
        repeat (20) @(posedge clk_sys);
        select_row(2); check(hw_key_data[1] === 1'b0, "w+a: 'w' pressed (row2 col1)");
        select_row(1); check(hw_key_data[0] === 1'b0, "w+a: 'a' pressed (row1 col0)");
        repeat (5000) @(posedge clk_sys);
        select_row(2); check(hw_key_data[1] === 1'b0, "w+a hold: 'w' still pressed");
        select_row(1); check(hw_key_data[0] === 1'b0, "w+a hold: 'a' still pressed");
        set_keys(8'h00, 8'h00);
        repeat (20) @(posedge clk_sys);
        select_row(2); check(hw_key_data[1] === 1'b1, "w+a release: 'w' released");
        select_row(1); check(hw_key_data[0] === 1'b1, "w+a release: 'a' released");

        set_keys(8'h16, 8'h07); // s slot1, d slot2
        repeat (20) @(posedge clk_sys);
        select_row(1); check(hw_key_data[1] === 1'b0, "s+d: 's' pressed (row1 col1)");
        select_row(1); check(hw_key_data[2] === 1'b0, "s+d: 'd' pressed (row1 col2)");
        set_keys(8'h00, 8'h00);
        repeat (20) @(posedge clk_sys);
        select_row(1); check(hw_key_data[1] === 1'b1, "s+d release: 's' released");
        select_row(1); check(hw_key_data[2] === 1'b1, "s+d release: 'd' released");

        if (errors == 0) $display("\n==== ALL MULTIKEY CHECKS PASSED ====");
        else $display("\n==== %0d MULTIKEY CHECK(S) FAILED ====", errors);
        $finish;
    end

    initial begin
        #20000000;
        $display("FAIL: simulation timeout");
        $finish;
    end

endmodule
