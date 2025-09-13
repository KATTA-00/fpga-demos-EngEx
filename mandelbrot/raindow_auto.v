`timescale 1ns/1ps
// ==================================================
// Animated rainbow palette test for DE2-115
// - Single framebuffer (320x240) filled once with smooth gradient
// - Color LUT maps pixel index -> rainbow color (rotated channels)
// - Animation: phase increments on VSYNC => palette cycles without re-render
// - Simple speed control via SW[0] (0 = slow, 1 = fast)
// ==================================================

module vga_controller(
    input  wire clk,          // pixel clock (~25 MHz)
    input  wire resetn,
    output reg  hsync,
    output reg  vsync,
    output reg  display_on,
    output reg [9:0] px,     // 0..319 when display area
    output reg [8:0] py      // 0..239 when display area
);
    // Standard 640x480@60 timings; center a 320x240 display area
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
            hcount <= 0;
            vcount <= 0;
            hsync <= 1;
            vsync <= 1;
            display_on <= 0;
            px <= 0;
            py <= 0;
        end else begin
            // horizontal counter
            if (hcount == H_TOTAL - 1) begin
                hcount <= 0;
                if (vcount == V_TOTAL - 1) vcount <= 0; else vcount <= vcount + 1;
            end else begin
                hcount <= hcount + 1;
            end

            // syncs (active low)
            if (hcount >= H_VISIBLE + H_FRONT && hcount < H_VISIBLE + H_FRONT + H_SYNC) hsync <= 0; else hsync <= 1;
            if (vcount >= V_VISIBLE + V_FRONT && vcount < V_VISIBLE + V_FRONT + V_SYNC) vsync <= 0; else vsync <= 1;

            // display area centered 320x240
            if ((hcount >= H_OFFSET) && (hcount < H_OFFSET + DISPLAY_WIDTH) &&
                (vcount >= V_OFFSET) && (vcount < V_OFFSET + DISPLAY_HEIGHT)) begin
                display_on <= 1;
                px <= hcount - H_OFFSET;
                py <= vcount - V_OFFSET;
            end else begin
                display_on <= 0;
                px <= 0;
                py <= 0;
            end
        end
    end
endmodule


// Rainbow-ish LUT: index -> 18-bit RGB.
// Simple approach: take index, form r/g/b by rotating the index (offsets 0,85,170)
// then truncate to 6 bits per channel.
module color_lut_rainbow (
    input  wire [7:0] index,
    output reg  [17:0] rgb   // {r[5:0], g[5:0], b[5:0]}
);
    wire [7:0] i0 = index;
    wire [7:0] i1 = index + 8'd85;   // +120 degrees phase ~ 85/256 of circle
    wire [7:0] i2 = index + 8'd170;  // +240 degrees

    // map 0..255 -> 6-bit value by shifting away low 2 bits
    wire [5:0] r6 = i0[7:2];
    wire [5:0] g6 = i1[7:2];
    wire [5:0] b6 = i2[7:2];

    always @(*) begin
        rgb = { r6, g6, b6 };
    end
endmodule


// single-port write, synchronous read inferred RAM
module framebuffer #(
    parameter integer DATA_WIDTH = 8,
    parameter integer ADDR_WIDTH = 17  // 2^17 = 131072 > 320*240 = 76800
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
        rdata <= mem[raddr]; // synchronous read
    end
endmodule


// screen_controller: writes a smooth 320x240 gradient into framebuffer once.
// Combines scaled X and Y to produce 0..255 indices.
module screen_controller (
    input  wire clk,
    input  wire resetn,
    output reg  we,
    output reg  [16:0] waddr, // enough for 320*240
    output reg  [7:0]  wdata,
    output reg  frame_done
);
    parameter H_VISIBLE = 320;
    parameter V_VISIBLE = 240;

    reg [9:0] x;
    reg [8:0] y;
    reg [1:0] state;
    localparam S_IDLE = 2'd0, S_WR = 2'd1, S_DONE = 2'd2;

    // row_base = y * 320 = y*256 + y*64 = (y << 8) + (y << 6)
    wire [16:0] row_base = (y << 8) + (y << 6);
    wire [16:0] current_addr = row_base + x;

    // scale 0..319 -> 0..255 approximated: scaled = (x * 3276) >> 12
    wire [25:0] prod_x = x * 17'd3276;
    wire [25:0] prod_y = y * 17'd3276;
    wire [7:0] scaled_x = prod_x[25:12]; // >>12
    wire [7:0] scaled_y = prod_y[25:12];

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            x <= 0; y <= 0;
            state <= S_IDLE;
            we <= 1'b0;
            waddr <= 17'd0;
            wdata <= 8'd0;
            frame_done <= 1'b0;
        end else begin
            we <= 1'b0;
            case (state)
                S_IDLE: begin
                    // start immediately after reset (single frame)
                    x <= 0; y <= 0;
                    frame_done <= 1'b0;
                    state <= S_WR;
                end

                S_WR: begin
                    we <= 1'b1;
                    waddr <= current_addr;
                    // combine scaled X and Y; change operator for different patterns:
                    wdata <= scaled_x + scaled_y; // additive blend -> smooth 2D ramp

                    // advance coordinates
                    if (x == H_VISIBLE - 1) begin
                        x <= 0;
                        if (y == V_VISIBLE - 1) begin
                            state <= S_DONE;
                        end else begin
                            y <= y + 1;
                        end
                    end else begin
                        x <= x + 1;
                    end
                end

                S_DONE: begin
                    we <= 1'b0;
                    frame_done <= 1'b1; // latched
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule


// ====================================================================
// Top: VGA + framebuffer + writer + animated palette
// ====================================================================
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
    // reset and pixel clock
    wire resetn = KEY[0];   // active-low button: released=1 -> normal operation
    wire pixel_clk = CLOCK_25; // if you have a 25.175 MHz PLL, use that instead

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

    // framebuffer read address: raddr = py*320 + px (shift-add)
    wire [16:0] vga_row_base = (py << 8) + (py << 6); // py*320
    wire [16:0] vga_addr = (display_on && frame_done) ? (vga_row_base + px) : 17'd0;

    // instantiate framebuffer
    wire fb_we;
    wire [16:0] fb_waddr;
    wire [7:0]  fb_wdata;
    wire [7:0]  fb_rdata;

    framebuffer #(.DATA_WIDTH(8), .ADDR_WIDTH(17)) fb_inst (
        .clk(pixel_clk),
        .we(fb_we),
        .waddr(fb_waddr),
        .wdata(fb_wdata),
        .raddr(vga_addr),
        .rdata(fb_rdata)
    );

    // screen controller writes gradient once
    wire sc_frame_done;
    screen_controller sc0 (
        .clk(pixel_clk),
        .resetn(resetn),
        .we(fb_we),
        .waddr(fb_waddr),
        .wdata(fb_wdata),
        .frame_done(sc_frame_done)
    );

    // latch frame_done so we display only after it's ready
    reg frame_done;
    always @(posedge pixel_clk or negedge resetn) begin
        if (!resetn) frame_done <= 1'b0;
        else if (sc_frame_done) frame_done <= 1'b1;
    end

    // pipeline read: synchronous RAM => data valid next cycle
    reg [7:0] fb_rdata_pipe;
    reg [16:0] raddr_pipe;
    always @(posedge pixel_clk) begin
        raddr_pipe <= vga_addr;
        fb_rdata_pipe <= fb_rdata;
    end

    // ------------------------
    // Palette animation (phase)
    // ------------------------
    // phase increments on VSYNC rising edge; speed selectable by SW[0]
    reg [1:0] vsync_sync;
    reg [7:0] phase;
    reg [7:0] phase_inc; // amount to add each vsync

    always @(posedge pixel_clk or negedge resetn) begin
        if (!resetn) begin
            vsync_sync <= 2'b11;
            phase <= 8'd0;
            phase_inc <= 8'd1;
        end else begin
            // sync external vsync into pixel_clk domain
            vsync_sync <= {vsync_sync[0], VGA_VS};

            // speed selection via SW[0] (0: slow, 1: fast)
            phase_inc <= SW[0] ? 8'd4 : 8'd1;

            // detect rising edge of VSYNC (low->high) 2'b01
            if (vsync_sync == 2'b01) begin
                phase <= phase + phase_inc;
            end
        end
    end

    // form LUT index: framebuffer value + phase (wraparound)
    wire [7:0] lut_index = fb_rdata_pipe + phase;

    // color LUT (rainbow)
    wire [17:0] rgb18;
    color_lut_rainbow clr0 (.index(lut_index), .rgb(rgb18));

    // output to VGA (pad 6-bit -> 8-bit by left-shift 2)
    assign VGA_R = (display_on && frame_done) ? {rgb18[17:12], 2'b00} : 8'd0;
    assign VGA_G = (display_on && frame_done) ? {rgb18[11:6],  2'b00} : 8'd0;
    assign VGA_B = (display_on && frame_done) ? {rgb18[5:0],   2'b00} : 8'd0;

    assign VGA_CLK = pixel_clk;
    assign VGA_BLANK_N = 1'b1; // keep; gating via display_on & frame_done
    assign VGA_SYNC_N  = 1'b0;

endmodule
