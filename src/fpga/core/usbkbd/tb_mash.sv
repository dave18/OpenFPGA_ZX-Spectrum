`timescale 1ns/1ps
// Stress test: QAOP+Space (Q=up/jump, A=down, O=left, P=right, Space=punch)
// - the classic Spectrum beat-em-up scheme. For every pair of these 5 keys,
// hold one while rapidly mashing the other many times, checking both track
// correctly throughout.

module tb_mash;

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

    // Q=USB0x14/row2col0  A=USB0x04/row1col0  O=USB0x12/row5col1
    // P=USB0x13/row5col0  SPACE=USB0x2C/row7col0
    task set4(input [7:0] sc1, input [7:0] sc2, input [7:0] sc3, input [7:0] sc4);
        begin hid_sc1 = sc1; hid_sc2 = sc2; hid_sc3 = sc3; hid_sc4 = sc4; hid_sc5 = 0; hid_sc6 = 0; hid_mod = 0; end
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

    // Hold `held_sc` in slot1, mash `mash_sc` in slot2 N times, verify both
    // read correctly on every iteration, then release both and verify clean.
    task hold_and_mash(
        input [7:0] held_sc, input integer held_row, input integer held_col,
        input [7:0] mash_sc, input integer mash_row, input integer mash_col,
        input [255:0] label
    );
        integer i;
        begin
            set4(held_sc, 8'h00, 8'h00, 8'h00);
            repeat (20) @(posedge clk_sys);
            select_row(held_row);
            check(hw_key_data[held_col] === 1'b0, {label, ": held key pressed before mashing"});

            for (i = 0; i < 15; i = i + 1) begin
                set4(held_sc, mash_sc, 8'h00, 8'h00);
                repeat (20) @(posedge clk_sys);
                select_row(held_row); check(hw_key_data[held_col] === 1'b0, {label, ": held key still pressed during mash-down"});
                select_row(mash_row); check(hw_key_data[mash_col] === 1'b0, {label, ": mashed key pressed"});
                set4(held_sc, 8'h00, 8'h00, 8'h00);
                repeat (20) @(posedge clk_sys);
                select_row(held_row); check(hw_key_data[held_col] === 1'b0, {label, ": held key still pressed during mash-up"});
                select_row(mash_row); check(hw_key_data[mash_col] === 1'b1, {label, ": mashed key released"});
            end

            set4(8'h00, 8'h00, 8'h00, 8'h00);
            repeat (20) @(posedge clk_sys);
            select_row(held_row);
            check(hw_key_data[held_col] === 1'b1, {label, ": held key releases cleanly at the end"});
        end
    endtask

    initial begin
        repeat (20) @(posedge clk_sys);
        reset = 0;
        repeat (20) @(posedge clk_sys);

        // hold O, mash Space (punch while moving)
        hold_and_mash(8'h12, 5, 1,  8'h2C, 7, 0, "hold-O_mash-Space");
        // hold P, mash Space
        hold_and_mash(8'h13, 5, 0,  8'h2C, 7, 0, "hold-P_mash-Space");
        // hold O, mash Q (jump while moving left)
        hold_and_mash(8'h12, 5, 1,  8'h14, 2, 0, "hold-O_mash-Q");
        // hold P, mash Q (jump while moving right)
        hold_and_mash(8'h13, 5, 0,  8'h14, 2, 0, "hold-P_mash-Q");
        // hold Q, mash Space (jump-punch)
        hold_and_mash(8'h14, 2, 0,  8'h2C, 7, 0, "hold-Q_mash-Space");
        // hold Space, mash O (weird order, but why not)
        hold_and_mash(8'h2C, 7, 0,  8'h12, 5, 1, "hold-Space_mash-O");
        // hold O, mash P (opposite directions rapidly)
        hold_and_mash(8'h12, 5, 1,  8'h13, 5, 0, "hold-O_mash-P");
        // hold A, mash Space (duck-punch)
        hold_and_mash(8'h04, 1, 0,  8'h2C, 7, 0, "hold-A_mash-Space");

        if (errors == 0) $display("\n==== ALL MASH CHECKS PASSED ====");
        else $display("\n==== %0d MASH CHECK(S) FAILED ====", errors);
        $finish;
    end

    initial begin
        #40000000;
        $display("FAIL: simulation timeout");
        $finish;
    end

endmodule
