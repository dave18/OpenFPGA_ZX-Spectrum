`timescale 1ns/1ps
// Three-key stress test: hold two keys (e.g. move + jump) simultaneously,
// then mash a third (punch) repeatedly, checking all three track correctly
// - beat-em-ups routinely need move+jump+attack all at once.

module tb_mash3;

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

    // O=USB0x12/row5col1  Q=USB0x14/row2col0  SPACE=USB0x2C/row7col0
    integer i;
    initial begin
        repeat (20) @(posedge clk_sys);
        reset = 0;
        repeat (20) @(posedge clk_sys);

        // Hold O (move) then add Q (jump) - both held, like jumping while moving
        set4(8'h12, 8'h00, 8'h00, 8'h00);
        repeat (20) @(posedge clk_sys);
        set4(8'h12, 8'h14, 8'h00, 8'h00);
        repeat (20) @(posedge clk_sys);
        select_row(5); check(hw_key_data[1] === 1'b0, "O+Q: O pressed");
        select_row(2); check(hw_key_data[0] === 1'b0, "O+Q: Q pressed");

        // Now mash Space (punch) repeatedly while O+Q both stay held (3rd slot)
        for (i = 0; i < 15; i = i + 1) begin
            set4(8'h12, 8'h14, 8'h2C, 8'h00);
            repeat (20) @(posedge clk_sys);
            select_row(5); check(hw_key_data[1] === 1'b0, "O+Q+mash: O still pressed (down)");
            select_row(2); check(hw_key_data[0] === 1'b0, "O+Q+mash: Q still pressed (down)");
            select_row(7); check(hw_key_data[0] === 1'b0, "O+Q+mash: Space pressed");
            set4(8'h12, 8'h14, 8'h00, 8'h00);
            repeat (20) @(posedge clk_sys);
            select_row(5); check(hw_key_data[1] === 1'b0, "O+Q+mash: O still pressed (up)");
            select_row(2); check(hw_key_data[0] === 1'b0, "O+Q+mash: Q still pressed (up)");
            select_row(7); check(hw_key_data[0] === 1'b1, "O+Q+mash: Space released");
        end

        // Release Q only (stop jumping, keep moving + still able to punch)
        set4(8'h12, 8'h00, 8'h00, 8'h00);
        repeat (20) @(posedge clk_sys);
        select_row(2); check(hw_key_data[0] === 1'b1, "release Q: Q released");
        select_row(5); check(hw_key_data[1] === 1'b0, "release Q: O still pressed");

        // Punch again after releasing Q - must still work
        set4(8'h12, 8'h2C, 8'h00, 8'h00);
        repeat (20) @(posedge clk_sys);
        select_row(7); check(hw_key_data[0] === 1'b0, "post-release: Space pressed");
        select_row(5); check(hw_key_data[1] === 1'b0, "post-release: O still pressed");
        set4(8'h12, 8'h00, 8'h00, 8'h00);
        repeat (20) @(posedge clk_sys);
        select_row(7); check(hw_key_data[0] === 1'b1, "post-release: Space released cleanly");

        // Release everything
        set4(8'h00, 8'h00, 8'h00, 8'h00);
        repeat (20) @(posedge clk_sys);
        select_row(5); check(hw_key_data[1] === 1'b1, "final: O released");
        select_row(2); check(hw_key_data[0] === 1'b1, "final: Q released");
        select_row(7); check(hw_key_data[0] === 1'b1, "final: Space released");

        if (errors == 0) $display("\n==== ALL 3-KEY MASH CHECKS PASSED ====");
        else $display("\n==== %0d 3-KEY MASH CHECK(S) FAILED ====", errors);
        $finish;
    end

    initial begin
        #40000000;
        $display("FAIL: simulation timeout");
        $finish;
    end

endmodule
