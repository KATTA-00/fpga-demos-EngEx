`timescale 1ns/1ps
// =======================================================
// Double-buffered animated test top (320x240) for DE2-115
// - Two framebuffers (ping-pong) to avoid tearing
// - Writer re-renders every frame using frame_phase parameter
// - Palette animation / phase updates on VSYNC (speed via switches)
// - Controls: KEY[0]=reset, KEY[1]=manual step, SW[0]=animate enable,
//             SW[1:0]=speed select (00:+1,01:+2,10:+4,11:+8)
// =======================================================

// ---------- VGA controller (same centered 320x240) ----------
module vga_controller(
    input  wire clk,
    input  wire resetn,
    output reg  hsync,
    output reg  vsync,
    output reg  display_on,
    output reg [9:0] px,
    output reg [8:0] py
);
    parameter H_VISIBLE = 640;
    parameter H_FRONT   = 16;
    parameter H_SYNC    = 96;
    parameter H_BACK    = 48;
    parameter H_TOTAL   = 800;

    parameter V_VISIBLE = 480;
    parameter V_FRONT   = 10;
    parameter V_SYNC    = 2;
    parameter V_BACK    = 33;
    parameter V_TOTAL   = 525;

    parameter DISPLAY_WIDTH = 320;
    parameter DISPLAY_HEIGHT = 240;

    localparam H_OFFSET = (H_VISIBLE - DISPLAY_WIDTH) / 2;
    localparam V_OFFSET = (V_VISIBLE - DISPLAY_HEIGHT) / 2;

    reg [10:0] hcount;
    reg [9:0]  vcount;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            hcount <= 0; vcount <= 0;
            hsync <= 1; vsync <= 1;
            display_on <= 0;
            px <= 0; py <= 0;
        end else begin
            // horizontal
            if (hcount == H_TOTAL - 1) begin
                hcount <= 0;
                if (vcount == V_TOTAL - 1) vcount <= 0; else vcount <= vcount + 1;
            end else begin
                hcount <= hcount + 1;
            end

            // sync (active low)
            if (hcount >= H_VISIBLE + H_FRONT && hcount < H_VISIBLE + H_FRONT + H_SYNC) hsync <= 0; else hsync <= 1;
            if (vcount >= V_VISIBLE + V_FRONT && vcount < V_VISIBLE + V_FRONT + V_SYNC) vsync <= 0; else vsync <= 1;

            // display area centered
            if ((hcount >= H_OFFSET) && (hcount < H_OFFSET + DISPLAY_WIDTH) &&
                (vcount >= V_OFFSET) && (vcount < V_OFFSET + DISPLAY_HEIGHT)) begin
                display_on <= 1;
                px <= hcount - H_OFFSET;
                py <= vcount - V_OFFSET;
            end else begin
                display_on <= 0;
                px <= 0; py <= 0;
            end
        end
    end
endmodule


// ---------- Rainbow LUT (rotated channels) ----------
module color_lut_rainbow (
    input  wire [7:0] index,
    output reg  [17:0] rgb   // {r[5:0], g[5:0], b[5:0]}
);
    wire [7:0] i0 = index;
    wire [7:0] i1 = index + 8'd85;
    wire [7:0] i2 = index + 8'd170;
    wire [5:0] r6 = i0[7:2];
    wire [5:0] g6 = i1[7:2];
    wire [5:0] b6 = i2[7:2];
    always @(*) rgb = {r6, g6, b6};
endmodule


// ---------- framebuffer (synchronous read) ----------
module framebuffer #(
    parameter integer DATA_WIDTH = 8,
    parameter integer ADDR_WIDTH = 17
)(
    input  wire clk,
    input  wire we,
    input  wire [ADDR_WIDTH-1:0] waddr,
    input  wire [DATA_WIDTH-1:0] wdata,
    input  wire [ADDR_WIDTH-1:0] raddr,
    output reg  [DATA_WIDTH-1:0] rdata
);
    localparam DEPTH = (1 << ADDR_WIDTH);
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    always @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end
endmodule


// ---------- screen_writer: writes 320x240 gradient + frame_phase ----------
module screen_writer (
    input  wire clk,
    input  wire resetn,
    input  wire start_frame,           // pulse high to start rendering into the assigned buffer
    input  wire [7:0] frame_phase,     // per-frame parameter added to pixel index
    output reg  we,
    output reg  [16:0] waddr,
    output reg  [7:0]  wdata,
    output reg  frame_done
);
    parameter H_VISIBLE = 320;
    parameter V_VISIBLE = 240;

    reg [9:0] x;
    reg [8:0] y;
    reg running;
    // row_base = y*320 = (y<<8) + (y<<6)
    wire [16:0] row_base = (y << 8) + (y << 6);
    wire [16:0] current_addr = row_base + x;

    // scale 0..319 -> 0..255 approx: scaled = (x*3276) >> 12
    wire [25:0] prod_x = x * 17'd3276;
    wire [25:0] prod_y = y * 17'd3276;
    wire [7:0] scaled_x = prod_x[25:12];
    wire [7:0] scaled_y = prod_y[25:12];

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            x <= 0; y <= 0; running <= 0;
            we <= 0; waddr <= 17'd0; wdata <= 8'd0; frame_done <= 1'b0;
        end else begin
            // default outputs
            we <= 1'b0;

            if (!running) begin
                if (start_frame) begin
                    // start render
                    running <= 1'b1;
                    x <= 0; y <= 0;
                    frame_done <= 1'b0;
                end
            end else begin
                // write pixel
                we <= 1'b1;
                waddr <= current_addr;
                // writer pattern: additive blend + frame_phase to animate visible content as it re-renders
                wdata <= scaled_x + scaled_y + frame_phase;

                // advance coordinates
                if (x == H_VISIBLE - 1) begin
                    x <= 0;
                    if (y == V_VISIBLE - 1) begin
                        // done
                        running <= 0;
                        frame_done <= 1'b1;
                    end else begin
                        y <= y + 1;
                    end
                end else begin
                    x <= x + 1;
                end
            end
        end
    end
endmodule


// ---------- top: double-buffer + swap on VSYNC ----------
module top (
    input  wire CLOCK_25,
    input  wire RESET_N,
    output wire VGA_CLK,
    output wire VGA_HS,
    output wire VGA_VS,
    output wire VGA_BLANK_N,
    output wire VGA_SYNC_N,
    output wire [7:0] VGA_R,
    output wire [7:0] VGA_G,
    output wire [7:0] VGA_B,
    input  wire [3:0] KEY,
    input  wire [17:0] SW
);
    // reset & pixel clock
    wire resetn = KEY[0]; // active-low pushbutton (released=1)
    wire pixel_clk = CLOCK_25; // replace with PLL output for precise 25.175 MHz if needed

    // VGA controller
    wire display_on;
    wire [9:0] px;
    wire [8:0] py;
    vga_controller vga0 (
        .clk(pixel_clk),
        .resetn(resetn),
        .hsync(VGA_HS),
        .vsync(VGA_VS),
        .display_on(display_on),
        .px(px),
        .py(py)
    );

    // compute read address for current display buffer
    wire [16:0] vga_row_base = (py << 8) + (py << 6); // py*320
    wire [16:0] vga_addr = (display_on && frame_display_ready) ? (vga_row_base + px) : 17'd0;

    // instantiate two framebuffers
    wire fb0_we, fb1_we;
    wire [16:0] fb0_waddr, fb1_waddr;
    wire [7:0]  fb0_wdata, fb1_wdata;
    wire [7:0]  fb0_rdata, fb1_rdata;

    framebuffer #(.DATA_WIDTH(8), .ADDR_WIDTH(17)) fb0 (
        .clk(pixel_clk),
        .we(fb0_we),
        .waddr(fb0_waddr),
        .wdata(fb0_wdata),
        .raddr(vga_addr),
        .rdata(fb0_rdata)
    );

    framebuffer #(.DATA_WIDTH(8), .ADDR_WIDTH(17)) fb1 (
        .clk(pixel_clk),
        .we(fb1_we),
        .waddr(fb1_waddr),
        .wdata(fb1_wdata),
        .raddr(vga_addr),
        .rdata(fb1_rdata)
    );

    // read mux: select which fb is currently displayed
    reg display_buffer; // 0 -> fb0 read, 1 -> fb1 read
    wire [7:0] fb_rdata = display_buffer ? fb1_rdata : fb0_rdata;

    // ---------- writer control (ping-pong) ----------
    reg write_buffer;   // 0 -> writer writes fb0, 1 -> writer writes fb1
    wire writer_we;
    wire [16:0] writer_waddr;
    wire [7:0] writer_wdata;
    wire writer_frame_done;

    // start_frame controls when writer begins; we gate it to avoid overlapping renders
    reg start_frame_reg;

    screen_writer writer0 (
        .clk(pixel_clk),
        .resetn(resetn),
        .start_frame(start_frame_reg),
        .frame_phase(frame_phase),
        .we(writer_we),
        .waddr(writer_waddr),
        .wdata(writer_wdata),
        .frame_done(writer_frame_done)
    );

    // route writer outputs to the chosen RAM
    assign fb0_we    = (write_buffer == 1'b0) ? writer_we    : 1'b0;
    assign fb0_waddr  = (write_buffer == 1'b0) ? writer_waddr  : 17'd0;
    assign fb0_wdata  = (write_buffer == 1'b0) ? writer_wdata  : 8'd0;

    assign fb1_we    = (write_buffer == 1'b1) ? writer_we    : 1'b0;
    assign fb1_waddr  = (write_buffer == 1'b1) ? writer_waddr  : 17'd0;
    assign fb1_wdata  = (write_buffer == 1'b1) ? writer_wdata  : 8'd0;

    // ---------- frame display gating ----------
    // frame_display_ready = true when at least one buffer has been filled so we can show it
    reg frame_display_ready;
    // On reset: start by writing to fb0 and display nothing until first frame completes
    // Flow:
    //  - start_frame_reg = 1 to start writer writing into write_buffer
    //  - when writer_frame_done becomes 1: set swap_pending and toggle write_buffer (so next render writes other buffer)
    //  - wait for VSYNC rising; on VSYNC rising apply display_buffer <= ~display_buffer (so display the newly finished buffer) and clear swap_pending; then start_frame_reg <= 1 to begin new render into next buffer

    reg swap_pending;
    reg [1:0] vsync_sync;
    reg writer_frame_done_prev;

    // frame_phase: per-frame animation parameter (updated on vsync or manual key)
    reg [7:0] frame_phase;
    wire animation_enable = SW[0];
    reg [7:0] phase_inc;
    // manual step detection on KEY[1] (active-low press)
    reg [1:0] key1_sync;

    // initial conditions & main control FSM
    always @(posedge pixel_clk or negedge resetn) begin
        if (!resetn) begin
            write_buffer <= 1'b0; // writer starts with fb0
            display_buffer <= 1'b0; // nothing displayed yet
            start_frame_reg <= 1'b1; // start initial render immediately
            swap_pending <= 1'b0;
            frame_display_ready <= 1'b0;
            vsync_sync <= 2'b11;
            writer_frame_done_prev <= 1'b0;
            frame_phase <= 8'd0;
            phase_inc <= 8'd1;
            key1_sync <= 2'b11;
        end else begin
            // sync vsync and key into pixel_clk domain
            vsync_sync <= {vsync_sync[0], VGA_VS};
            key1_sync <= {key1_sync[0], KEY[1]};

            // map SW[1:0] to increments
            case (SW[1:0])
                2'b00: phase_inc <= 8'd1;
                2'b01: phase_inc <= 8'd2;
                2'b10: phase_inc <= 8'd4;
                default: phase_inc <= 8'd8;
            endcase

            // detect writer frame completion (rising edge)
            if (writer_frame_done && !writer_frame_done_prev) begin
                // first completed frame: if no display yet, set frame_display_ready
                frame_display_ready <= 1'b1;
                // request swap: show the newly finished buffer on next VSYNC
                swap_pending <= 1'b1;
                // immediately flip write_buffer so the writer will now start writing to the other buffer
                write_buffer <= ~write_buffer;
                // stop the writer until we swap (prevent writer from immediately starting next frame before swap)
                start_frame_reg <= 1'b0;
            end

            writer_frame_done_prev <= writer_frame_done;

            // manual KEY[1] press to step phase (falling edge 1->0)
            if (key1_sync == 2'b10) begin
                frame_phase <= frame_phase + phase_inc;
            end

            // On VSYNC rising (low->high), perform the display swap if pending
            if (vsync_sync == 2'b01) begin
                if (swap_pending) begin
                    display_buffer <= ~display_buffer; // show the most recently completed buffer
                    swap_pending <= 1'b0;
                end
                // advance phase if animation enabled
                if (animation_enable) frame_phase <= frame_phase + phase_inc;
                // after vsync, start next render if we are not currently rendering
                // start_frame_reg is used to kick the writer; writer stops itself when done, so we can safely restart
                if (!writer_frame_done) begin
                    // if writer currently running, do nothing; otherwise allow next render
                    start_frame_reg <= 1'b1;
                end else begin
                    // if writer finished and we already swapped above, start next frame
                    start_frame_reg <= 1'b1;
                end
            end
        end
    end

    // pipeline synchronous RAM read (one-cycle)
    reg [7:0] fb_rdata_pipe;
    reg [16:0] raddr_pipe;
    always @(posedge pixel_clk) begin
        raddr_pipe <= vga_addr;
        fb_rdata_pipe <= fb_rdata;
    end

    // color lookup with animated palette: (fb value) + (frame_phase)
    wire [7:0] lut_index = fb_rdata_pipe + frame_phase;
    wire [17:0] rgb18;
    color_lut_rainbow lut0 (.index(lut_index), .rgb(rgb18));

    // VGA outputs
    assign VGA_R = (display_on && frame_display_ready) ? {rgb18[17:12], 2'b00} : 8'd0;
    assign VGA_G = (display_on && frame_display_ready) ? {rgb18[11:6],  2'b00} : 8'd0;
    assign VGA_B = (display_on && frame_display_ready) ? {rgb18[5:0],   2'b00} : 8'd0;

    assign VGA_CLK = pixel_clk;
    assign VGA_BLANK_N = 1'b1;
    assign VGA_SYNC_N  = 1'b0;

endmodule
