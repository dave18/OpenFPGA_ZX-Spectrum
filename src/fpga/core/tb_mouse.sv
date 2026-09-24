`timescale 1ns/1ps
// Standalone Icarus Verilog testbench for mouse.v's accumulate-and-
// saturate logic, driven directly with HID-shaped inputs exactly as
// core_top.sv wires it on clk_74a (no PS/2 packet format anywhere).
// mouse.v is now a pure accumulator (dx_out/dy_out/buttons_out level
// outputs, no address decode) - the Kempston Mouse I/O-port mux that
// used to live inside this module moved to core_top.sv, which runs it
// on clk_sys against a synchronized copy of these outputs; see
// tb_mouse_gating.sv for that side.
//
// mouse.v's dx_out/dy_out/buttons_out `assign` statements are declared
// before the dx/dy/buttons regs they reference (matching this project's
// usual vendored-file style, which Quartus accepts but Icarus doesn't -
// see usbkbd/README.md for the general pattern). Make an Icarus-only
// copy with the assigns moved after the reg declarations, then:
//
//   iverilog -g2012 -o tb.vvp tb_mouse.sv mouse.v
//   vvp tb.vvp

module tb_mouse;

    reg clk_74a = 0;
    always #6.734 clk_74a = ~clk_74a; // 13.468ns period, matches clk_74a

    reg reset = 1;
    reg [15:0] hid_counter = 0;
    reg signed [15:0] hid_dx = 0;
    reg signed [15:0] hid_dy = 0;
    reg [2:0] hid_buttons = 0;
    wire [7:0] dx_out, dy_out;
    wire [2:0] buttons_out;

    mouse u (
        .clk(clk_74a), .reset(reset),
        .hid_counter(hid_counter), .hid_dx(hid_dx), .hid_dy(hid_dy),
        .hid_buttons(hid_buttons),
        .dx_out(dx_out), .dy_out(dy_out), .buttons_out(buttons_out)
    );

    integer errors = 0;
    task check(input cond, input [255:0] msg);
        begin
            if (cond) $display("PASS: %0s", msg);
            else begin $display("FAIL: %0s", msg); errors = errors + 1; end
        end
    endtask

    task send_report(input signed [15:0] dx, input signed [15:0] dy, input [2:0] btns);
        begin
            hid_dx = dx; hid_dy = dy; hid_buttons = btns;
            hid_counter = hid_counter + 1;
            repeat (5) @(posedge clk_74a);
        end
    endtask

    initial begin
        repeat (10) @(posedge clk_74a);
        reset = 0;
        repeat (10) @(posedge clk_74a);

        // reset defaults: dx=128, dy=0 (dx != dy is deliberate - some
        // software uses this asymmetry to detect a genuine Kempston
        // mouse is present)
        check(dx_out === 8'd128, "reset: dx == 128");
        check(dy_out === 8'd0, "reset: dy == 0");
        check(buttons_out === 3'b000, "reset: no buttons pressed (raw, active-high)");

        // --- basic positive movement ---
        send_report(16'sd10, 16'sd5, 3'b000);
        check(dx_out === 8'd138, "move +10,+5: dx == 138");
        check(dy_out === 8'd5, "move +10,+5: dy == 5");

        // --- negative movement ---
        send_report(-16'sd20, -16'sd3, 3'b000);
        check(dx_out === 8'd118, "move -20,-3: dx == 118 (138-20)");
        check(dy_out === 8'd2, "move -20,-3: dy == 2 (5-3)");

        // --- buttons pass through raw (active-high, no swap - that's
        // core_top.sv's job now) ---
        send_report(16'sd0, 16'sd0, 3'b001);
        check(buttons_out === 3'b001, "left button: bit0 set");
        send_report(16'sd0, 16'sd0, 3'b010);
        check(buttons_out === 3'b010, "right button: bit1 set");
        send_report(16'sd0, 16'sd0, 3'b100);
        check(buttons_out === 3'b100, "middle button: bit2 set");

        // --- report counter must NOT retrigger without a change ---
        repeat (5) @(posedge clk_74a);
        check(dx_out === 8'd118, "no new report: dx unchanged");

        // --- large positive delta saturates dx to 0xFF (upper byte clamp) ---
        reset = 1; repeat(4) @(posedge clk_74a); reset = 0; repeat(4) @(posedge clk_74a);
        send_report(16'sd20000, 16'sd0, 3'b000);
        check(dx_out === 8'hFF, "huge positive delta: dx saturates to 0xFF");

        // --- large negative delta saturates dx to 0x00 ---
        reset = 1; repeat(4) @(posedge clk_74a); reset = 0; repeat(4) @(posedge clk_74a);
        send_report(-16'sd20000, 16'sd0, 3'b000);
        check(dx_out === 8'h00, "huge negative delta: dx saturates to 0x00");

        if (errors == 0) $display("\n==== ALL MOUSE CHECKS PASSED ====");
        else $display("\n==== %0d MOUSE CHECK(S) FAILED ====", errors);
        $finish;
    end

    initial begin
        #200000;
        $display("FAIL: simulation timeout");
        $finish;
    end

endmodule
