`timescale 1ns / 1ps

/**
 * tb_mf_chirp_trigger.v
 *
 * Regression for the chirp-start contract of matched_filter_multi_segment:
 *   1. mc_new_chirp is a TOGGLE: both a 0->1 and a 1->0 transition start a
 *      chirp (rising-edge-only detection dropped every second chirp).
 *   2. A short chirp produces exactly one 1024-sample range profile and the
 *      FSM returns to IDLE (it previously looped in ST_COLLECT_DATA forever).
 *   3. No transition -> no capture.
 */

module tb_mf_chirp_trigger;

localparam CLK_PERIOD = 10.0;  // 100 MHz

reg clk = 1'b0;
reg reset_n = 1'b0;
always #(CLK_PERIOD / 2) clk = ~clk;

reg signed [17:0] ddc_i = 0, ddc_q = 0;
reg ddc_valid = 1'b0;
reg use_long_chirp = 1'b0;
reg mc_new_chirp = 1'b0;

wire [1:0] segment_request;
wire [9:0] sample_addr;
wire mem_request;
reg  mem_ready = 1'b0;
wire signed [15:0] pc_i, pc_q;
wire pc_valid;
wire [3:0] status;

always @(posedge clk) mem_ready <= mem_request;

matched_filter_multi_segment dut (
    .clk(clk), .reset_n(reset_n),
    .ddc_i(ddc_i), .ddc_q(ddc_q), .ddc_valid(ddc_valid),
    .use_long_chirp(use_long_chirp), .chirp_counter(6'd0),
    .mc_new_chirp(mc_new_chirp), .mc_new_elevation(1'b0), .mc_new_azimuth(1'b0),
    .long_chirp_real(16'd100), .long_chirp_imag(16'd0),
    .short_chirp_real(16'd100), .short_chirp_imag(16'd0),
    .segment_request(segment_request), .sample_addr_out(sample_addr),
    .mem_request(mem_request), .mem_ready(mem_ready),
    .pc_i_w(pc_i), .pc_q_w(pc_q), .pc_valid_w(pc_valid), .status(status)
);

// Continuous ADC stream
always @(posedge clk) begin
    ddc_valid <= reset_n;
    ddc_i <= $random % 2000;
    ddc_q <= $random % 2000;
end

integer outputs = 0;
always @(posedge clk) if (pc_valid) outputs = outputs + 1;

integer pass_count = 0, fail_count = 0;
task check(input cond, input [8*72-1:0] label);
    begin
        if (cond) begin pass_count = pass_count + 1; $display("[PASS] %0s", label); end
        else      begin fail_count = fail_count + 1; $display("[FAIL] %0s", label); end
    end
endtask

// Wait for the FSM to finish a chirp (back in IDLE with outputs produced)
task wait_chirp_done(input integer timeout_cycles, output reg done);
    integer n;
    begin
        done = 1'b0;
        for (n = 0; n < timeout_cycles && !done; n = n + 1) begin
            @(posedge clk);
            if (dut.state == 4'd0 && outputs > 0 && !pc_valid) done = 1'b1;
        end
    end
endtask

reg done;
integer before;

initial begin
    repeat (10) @(posedge clk);
    reset_n = 1'b1;
    repeat (10) @(posedge clk);

    // ---- No transition: stays idle ----
    repeat (200) @(posedge clk);
    check(dut.state == 4'd0, "No toggle: FSM stays in IDLE");

    // ---- Short chirp on a RISING toggle ----
    #1;
    use_long_chirp = 1'b0;
    mc_new_chirp = 1'b1;   // drive away from the clock edge
    repeat (3) @(posedge clk);
    check(dut.state != 4'd0, "Rising toggle starts a short chirp");
    wait_chirp_done(400000, done);
    check(done, "Short chirp returns to IDLE (no endless re-collect)");
    check(outputs == 1024, "Short chirp yields exactly 1024 range samples");

    // ---- Short chirp on a FALLING toggle ----
    repeat (100) @(posedge clk);
    check(dut.state == 4'd0, "Still IDLE before second toggle");
    before = outputs;
    outputs = 0;
    #1;
    mc_new_chirp = 1'b0;
    repeat (3) @(posedge clk);
    check(dut.state != 4'd0, "Falling toggle also starts a chirp");
    wait_chirp_done(400000, done);
    check(done, "Second short chirp returns to IDLE");
    check(outputs == 1024, "Second short chirp yields exactly 1024 range samples");

    $display("\n  PASSED: %0d  FAILED: %0d", pass_count, fail_count);
    if (fail_count == 0) $display("  ** ALL TESTS PASSED **");
    else                 $display("  ** SOME TESTS FAILED **");
    $finish;
end

initial begin
    #50_000_000;
    $display("[FAIL] Global timeout");
    $display("  ** SOME TESTS FAILED **");
    $finish;
end

endmodule
