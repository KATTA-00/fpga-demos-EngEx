`timescale 1ns/1ps
`include "main.v"

module testbench_single_frame;
    reg CLOCK_50 = 0;
    reg RESET_N = 1'b1; // kept for port compatibility; not used as reset inside DUT
    wire VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N;
    wire [7:0] VGA_R, VGA_G, VGA_B;
    reg [3:0] KEY = 4'hF;
    reg [17:0] SW = 18'h0;

    // DUT
    top dut(
        .CLOCK_50(CLOCK_50),
        .RESET_N(RESET_N),
        .VGA_CLK(VGA_CLK),
        .VGA_HS(VGA_HS),
        .VGA_VS(VGA_VS),
        .VGA_BLANK_N(VGA_BLANK_N),
        .VGA_SYNC_N(VGA_SYNC_N),
        .VGA_R(VGA_R),
        .VGA_G(VGA_G),
        .VGA_B(VGA_B),
        .KEY(KEY),
        .SW(SW)
    );

    // Clock generation (25 MHz)
    always #20 CLOCK_50 = ~CLOCK_50;

    // Drive reset via KEY[0] (active-low)
    initial begin
        KEY[0] = 1'b0; // assert reset
        repeat (10) @(posedge CLOCK_50);
        KEY[0] = 1'b1; // deassert reset
    end

    // Hook internal signals
    wire fb_we    = dut.fb_we;
    wire [18:0] fb_waddr = dut.fb_waddr;
    wire [7:0] fb_wdata = dut.fb_wdata;

    // set small resolution & iteration count
    defparam dut.sc0.H_VISIBLE = 64;
    defparam dut.sc0.V_VISIBLE = 48;
    defparam dut.fc0.MAX_ITER = 16;
    defparam dut.fc0.H_VISIBLE = 64;
    defparam dut.fc0.V_VISIBLE = 48;

    integer write_count = 0;
    integer frame_writes_expected = 64*48;
    reg [7:0] last_writes [0:31];
    integer i;

    initial begin
        $display("TB: start. Waiting for one full frame (%0d pixels)...", frame_writes_expected);
        // wait until we've observed an entire frame of writes
    @(posedge KEY[0]);
        // run until write_count reaches one frame
        wait (write_count >= frame_writes_expected);
        // give a cycle to capture final write
        @(posedge CLOCK_50);
        $display("TB: Completed one frame. Total writes observed: %0d", write_count);

        // print first 32 writes and last 8 writes (if available)
        $display("First up to 32 writes:");
        for (i = 0; i < 32 && i < frame_writes_expected; i = i + 1) begin
            $display("  idx=%0d addr=%0d data=%06h", i, i, dut.fb_wdata /* placeholder; hierarchical read below */);
        end

        // Actually sample the buffer content by reading mem via hierarchical name (if simulator supports)
        $display("Note: to inspect actual framebuffer array contents, open waveform or modify module to expose memory read ports.");

        $finish;
    end

    // Count writes
    always @(posedge CLOCK_50) begin
        if (KEY[0] && fb_we) begin
            write_count <= write_count + 1;
            // store last writes for debugging (optional)
            last_writes[write_count % 32] <= fb_wdata;
        end
    end

endmodule
