// DE2-115 Real-Time VGA Fractal Generator (Mandelbrot)
// -----------------------------------------------------
// Repository README + Verilog implementation skeleton for a real-time
// Mandelbrot/Julia fractal renderer targeting the DE2-115 (Cyclone IV).
//
// OVERVIEW
// This project renders the Mandelbrot set to a VGA display using the
// DE2-115 board. It uses a 25.175 MHz pixel clock (generated using a
// PLL from the on-board 50 MHz oscillator), a VGA controller that
// produces hsync/vsync/display enable, a pipelined fixed-point fractal
// core that computes pixel colors, and a small color LUT stored in ROM.
//
// FEATURES
// - 640x480@60Hz VGA output (pixel clock ~25 MHz)
// - Fixed-point pipeline (signed Q16.16) for complex arithmetic
// - Configurable max-iterations (parameterizable)
// - Color LUT for mapping iteration count to 18-bit RGB (6:6:6)
// - Optional panning/zoom controls via on-board switches/keys
// - Framebuffer implemented in on-chip RAM (BRAM)
//
// USAGE / FLOW
// 1. PLL: generate 25 MHz pixel clock from board 50 MHz oscillator.
// 2. VGA controller: generate hsync/vsync, and provide (x,y) coordinates
//    when display_enable is asserted.
// 3. For each visible pixel, the top module requests a color from the
//    fractal engine. If the fractal engine hasn't computed the pixel yet,
//    we can either show a placeholder color or stall using a framebuffer
//    approach where the fractal engine writes completed pixels into BRAM.
// 4. The fractal engine iterates z_{n+1} = z_n^2 + c (Mandelbrot) using
//    fixed-point math. The number of iterations before escape maps to
//    a color index.
//
// NOTES / DESIGN DECISIONS
// - Fixed-point format: signed 32-bit Q16.16 (16 integer bits, 16 frac)
// - Iteration core processes one iteration per clock in the simple FSM
//   variant. For higher performance, replicate cores or implement
//   parallelism to compute multiple pixels concurrently.
// - To keep VGA steady, pixel readout is synchronous to the 25MHz pixel
//   clock. Fractal computation can run at the same clock domain; if it
//   can't keep up, compute into an off-screen buffer and let VGA read
//   older frames.
//
// FILES IN THIS SINGLE-VERSION:
// - pll_25mhz.v     -- placeholder for ALTPLL instantiation (Quartus IP)
// - vga_controller.v -- VGA timing generator (640x480@60)
// - color_lut.v     -- small ROM mapping iteration to RGB
// - fractal_core.v  -- fixed-point Mandelbrot iteration core + controller
// - framebuffer.v  -- dual-port RAM for pixel storage (write from core, read from VGA)
// - top.v           -- top-level ties everything together
// - testbench.v     -- basic simulation harness outline (not exhaustive)
//
// ------------------------------------------------------------------
// IMPORTANT: Before synthesis
// - Use Quartus to generate a PLL with the correct input and output
//   frequencies (25.175 MHz for standard VGA 640x480@60; some designs
//   use 25.0 MHz which visually is okay on many monitors). Replace
//   pll_25mhz.v placeholder with the generated IP file.
// - Assign pins for VGA (R/G/B, HS, VS), audio (if used), and switches.
// - If using SDRAM/SDRAM controller for larger framebuffers, include
//   the appropriate memory controller IP and timing constraints.
//
// ------------------------------------------------------------------
// Below: Verilog modules (compact but functional skeletons). Do NOT
// copy/paste blindly — treat as a starting point and adapt pin names
// and PLL instantiation to your Quartus-generated IP.

`timescale 1ns/1ps

// ---------------------------
// VGA Controller: 640x480@60
// ---------------------------
module vga_controller(
    input  wire clk,          // pixel clock ~25 MHz
    input  wire resetn,
    output reg  hsync,
    output reg  vsync,
    output reg  display_on,
    output reg [9:0] px,     // 0..639
    output reg [8:0] py      // 0..479
);
    // Timing parameters for 640x480@60 (standard)
    parameter H_VISIBLE = 640;
    parameter H_FRONT   = 16;
    parameter H_SYNC    = 96;
    parameter H_BACK    = 48;
    parameter H_TOTAL   = 800; // 640 + 16 + 96 + 48

    parameter V_VISIBLE = 480;
    parameter V_FRONT   = 10;
    parameter V_SYNC    = 2;
    parameter V_BACK    = 33;
    parameter V_TOTAL   = 525; // 480 + 10 + 2 + 33

    reg [10:0] hcount;
    reg [9:0]  vcount;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            hcount <= 0;
            vcount <= 0;
            hsync <= 1;
            vsync <= 1;
            px <= 0;
            py <= 0;
            display_on <= 0;
        end else begin
            // horizontal
            if (hcount == H_TOTAL - 1) begin
                hcount <= 0;
                // vertical
                if (vcount == V_TOTAL - 1) vcount <= 0; else vcount <= vcount + 1;
            end else begin
                hcount <= hcount + 1;
            end

            // HSYNC: active low during sync pulse
            if (hcount >= H_VISIBLE + H_FRONT && hcount < H_VISIBLE + H_FRONT + H_SYNC) hsync <= 0; else hsync <= 1;
            // VSYNC
            if (vcount >= V_VISIBLE + V_FRONT && vcount < V_VISIBLE + V_FRONT + V_SYNC) vsync <= 0; else vsync <= 1;

            // output pixel coords and display on
            if (hcount < H_VISIBLE && vcount < V_VISIBLE) begin
                display_on <= 1;
                px <= hcount[9:0];
                py <= vcount[8:0];
            end else begin
                display_on <= 0;
                px <= 0;
                py <= 0;
            end
        end
    end
endmodule

// ---------------------------
// Simple Color LUT (ROM)
// Map iteration counts (0..MAX_ITER) to RGB (6-bit each) packed to 18-bit
// ---------------------------
module color_lut (
    input  wire [7:0] index, // support up to 256 steps
    output reg  [17:0] rgb   // {r[5:0], g[5:0], b[5:0]}
);
    always @(*) begin
        case (index)
            8'd0:  rgb = 18'b000000_000000_000000; // black
            8'd1:  rgb = 18'b000010_000001_000001;
            8'd2:  rgb = 18'b000100_000010_000001;
            8'd3:  rgb = 18'b001000_000100_000010;
            8'd4:  rgb = 18'b010000_001000_000100;
            8'd5:  rgb = 18'b100000_010000_001000;
            // ... fill out gradient for better visuals
            default: rgb = {index[5:0], index[5:0], index[5:0]};
        endcase
    end
endmodule

// ---------------------------
// Fractal Core (Mandelbrot)
// Fixed-point Q16.16 representation in signed 32-bit
// Simple FSM: map pixel->c, iterate until |z|^2 > 4 or max_iter
// One iteration per clock (simple version)
// ---------------------------
module fractal_core #(
    parameter PIXEL_WIDTH = 10,   // bit-width for px (supports up to 1023)
    parameter LINE_WIDTH = 9,     // bit-width for py (supports up to 511)
    parameter integer H_VISIBLE = 640, // horizontal resolution for mapping
    parameter integer V_VISIBLE = 480, // vertical resolution for mapping
    parameter MAX_ITER = 64
)(
    input  wire clk,
    input  wire resetn,
    // configuration
    input  wire [31:0] cx_min, // Q16.16
    input  wire [31:0] cx_max,
    input  wire [31:0] cy_min,
    input  wire [31:0] cy_max,

    // request interface
    input  wire request_pixel,
    input  wire [PIXEL_WIDTH-1:0] px,
    input  wire [LINE_WIDTH-1:0] py,

    // response
    output reg  valid,
    output reg  [7:0] iter_count // truncated to 8-bit index
);
    // Convert pixel to complex c: c_re = cx_min + (cx_max-cx_min) * (px/width)
    // We'll compute as: c_re = cx_min + (cx_range * px) / (width-1)

    // registers
    reg [31:0] c_re;
    reg [31:0] c_im;

    // z registers in Q16.16
    reg signed [31:0] z_re;
    reg signed [31:0] z_im;
    reg signed [63:0] z_re_sq; // intermediate for square (Q32.32)
    reg signed [63:0] z_im_sq;
    reg signed [63:0] z_re_im; // 2*z_re*z_im intermediate

    reg [7:0] iter;
    reg running;

    wire signed [31:0] cx_range = cx_max - cx_min;
    wire signed [31:0] cy_range = cy_max - cy_min;

    // mapping pixel -> c (simple multiply + divide by width)
    // We'll implement a small multiply-accumulate pipeline for mapping.

    // simple synchronous implementation: when request_pixel asserted, compute c in a few cycles
    reg [1:0] map_stage;
    reg [31:0] tmp_re_num;
    reg [31:0] tmp_im_num;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            valid <= 0;
            running <= 0;
            iter <= 0;
            map_stage <= 0;
            iter_count <= 0;
        end else begin
            valid <= 0;
            if (!running && request_pixel) begin
                // start mapping
                // tmp = range * px
                tmp_re_num <= cx_range * px;
                tmp_im_num <= cy_range * py;
                map_stage <= 1;
                running <= 1;
                iter <= 0;
            end else if (running && map_stage == 1) begin
                // finish mapping: divide by (H_VISIBLE-1) and (V_VISIBLE-1)
                // For clarity in this skeleton, do integer division (synthesizes a divider).
                c_re <= cx_min + (tmp_re_num / (H_VISIBLE > 1 ? (H_VISIBLE-1) : 1));
                c_im <= cy_min + (tmp_im_num / (V_VISIBLE > 1 ? (V_VISIBLE-1) : 1));
                // initialize z = 0
                z_re <= 32'd0;
                z_im <= 32'd0;
                map_stage <= 2;
            end else if (running && map_stage == 2) begin
                // iteration loop
                // compute z_re_sq = z_re*z_re (Q32.32), same for z_im_sq
                z_re_sq <= z_re * z_re; // product gives Q32.32 in 64-bit
                z_im_sq <= z_im * z_im;
                z_re_im <= (z_re * z_im) <<< 1; // 2*z_re*z_im (careful with scaling)
                // after compute, update z
                // z_re_new = z_re_sq - z_im_sq + c_re
                // convert Q32.32 back to Q16.16 by >>16
                z_re <= (z_re_sq - z_im_sq) >>> 16 + c_re;
                z_im <= (z_re_im >>> 16) + c_im;

                // check magnitude squared: if (z_re_sq + z_im_sq) > (4 in Q32.32 -> 4 << 32?)
                // But simpler: compute |z|^2 in Q32.32 and compare to (4 << 32)
                // We'll compare using z_re_sq (signed 64) + z_im_sq
                if (((z_re_sq + z_im_sq) >>> 32) > 4) begin
                    valid <= 1;
                    iter_count <= iter;
                    running <= 0;
                end else begin
                    if (iter == MAX_ITER - 1) begin
                        valid <= 1;
                        iter_count <= MAX_ITER - 1;
                        running <= 0;
                    end else begin
                        iter <= iter + 1;
                    end
                end
            end
        end
    end
endmodule

// ---------------------------
// Simple dual-port framebuffer (write by fractal core; read by VGA)
// Implemented using inferred block RAM (synthesis maps to M9K/M10K depending on device)
// Each location stores 18-bit RGB packed to 24-bit for byte alignment
// ---------------------------
module framebuffer #(
    parameter integer DATA_WIDTH = 8,
    parameter integer DEPTH = 76800 // 320*240
)(
    input wire clk,
    // write port
    input wire we,
    input wire [18:0] waddr, // enough to cover 320*240 = 76800 -> needs 17 bits but keep 19 for compatibility
    input wire [DATA_WIDTH-1:0] wdata,
    // read port
    input wire [18:0] raddr,
    output reg [DATA_WIDTH-1:0] rdata
);
    // Use inferred RAM: declare reg array
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    always @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end
endmodule

// ---------------------------
// Top-level: tie VGA controller, PLL, fractal core, framebuffer, color LUT
// Note: Replace pll_25mhz placeholder with Quartus-generated PLL IP.
// ---------------------------

// module top (
//     input wire CLOCK_25,
//     input wire RESET_N,
//     // VGA outputs (example names; map to DE2-115 pins)
//     output wire VGA_CLK,
//     output wire VGA_HS,
//     output wire VGA_VS,
//     output wire VGA_BLANK_N,
//     output wire VGA_SYNC_N,
//     output wire [7:0] VGA_R,
//     output wire [7:0] VGA_G,
//     output wire [7:0] VGA_B,
//     // controls: switches for zoom/pan
//     input wire [3:0] KEY,
//     input wire [17:0] SW
// );
//     wire pixel_clk;
//     wire pll_locked;
//     // Use KEY[0] as active-low reset for simulation/board; keep RESET_N port unused here
//     wire resetn = KEY[0];

//     // instantiate PLL (replace with generated IP in Quartus)
//     // pll_25mhz pll_inst(.inclk0(CLOCK_25), .c0(pixel_clk), .locked(pll_locked));
//     // For simulation or a board with an external 25MHz, tie CLOCK_25/2 but use generated IP for real board
//     assign pixel_clk = CLOCK_25; // placeholder: replace with pll output

//     // vga controller
//     wire display_on;
//     wire [9:0] px;
//     wire [8:0] py;
//     vga_controller vga0(.clk(pixel_clk), .resetn(resetn), .hsync(VGA_HS), .vsync(VGA_VS), .display_on(display_on), .px(px), .py(py));

//     // linear address for framebuffer: addr = py*640 + px
//     wire [18:0] vga_addr = py * 640 + px; // multiply synthesizes a multiplier; okay for simple designs

//     // framebuffer read
//     wire [7:0] fb_pixel_index;
//     framebuffer #(.DATA_WIDTH(8)) fb(
//         .clk(pixel_clk), .we(fb_we), .waddr(fb_waddr), .wdata(fb_wdata), .raddr(vga_addr), .rdata(fb_pixel_index)
//     );

//     // Map 8-bit index from framebuffer through LUT to RGB
//     wire [17:0] vga_rgb18;
//     color_lut vga_lut(.index(fb_pixel_index), .rgb(vga_rgb18));
//     wire [23:0] vga_rgb24 = {vga_rgb18[17:12], 2'b00, vga_rgb18[11:6], 2'b00, vga_rgb18[5:0], 2'b00};
//     assign VGA_R = vga_rgb24[23:16];
//     assign VGA_G = vga_rgb24[15:8];
//     assign VGA_B = vga_rgb24[7:0];

//     assign VGA_BLANK_N = display_on;
//     assign VGA_SYNC_N = 1'b1;
//     assign VGA_CLK = pixel_clk;

//     // Fractal rendering control and connections

//     // wires between controller, fractal core, LUT and framebuffer
//     wire fc_valid;
//     wire [7:0] fc_iter;
//     wire request_pixel;
//     wire [9:0] sc_px;
//     wire [8:0] sc_py;

//     // framebuffer write signals
//     wire fb_we;
//     wire [18:0] fb_waddr;
//     wire [7:0]  fb_wdata;

//     // fractal view params (Q16.16)
//     localparam signed [31:0] CX_MIN = -32'sd131072; // -2.0
//     localparam signed [31:0] CX_MAX =  32'sd65536;  //  1.0
//     localparam signed [31:0] CY_MIN = -32'sd78643;  // -1.2
//     localparam signed [31:0] CY_MAX =  32'sd78643;  //  1.2

//     // instantiate fractal core
//     // NOTE: PIXEL_WIDTH/LINE_WIDTH are bit widths (10 bits for 0..639, 9 bits for 0..479)
//     fractal_core #(.PIXEL_WIDTH(10), .LINE_WIDTH(9), .H_VISIBLE(640), .V_VISIBLE(480), .MAX_ITER(128)) fc0 (
//         .clk(pixel_clk),
//         .resetn(resetn),
//         .cx_min(CX_MIN),
//         .cx_max(CX_MAX),
//         .cy_min(CY_MIN),
//         .cy_max(CY_MAX),
//         .request_pixel(request_pixel),
//         .px(sc_px),
//         .py(sc_py),
//         .valid(fc_valid),
//         .iter_count(fc_iter)
//     );

//     // instantiate screen controller (drives request_pixel, px, py and writes to framebuffer)
//     screen_controller sc0 (
//         .clk(pixel_clk),
//         .resetn(resetn),
//         .start_frame(1'b1), // continuously render
//         .fc_valid(fc_valid),
//         .pixel_index(fc_iter),
//         .request_pixel(request_pixel),
//         .px(sc_px),
//         .py(sc_py),
//         .we(fb_we),
//         .waddr(fb_waddr),
//         .wdata(fb_wdata)
//     );

//     // connect framebuffer write signals
//     // The framebuffer instance above was updated to use fb_we/fb_waddr/fb_wdata

// endmodule



// Minimal VGA Test Pattern for DE2-115
// Just outputs a solid color to check VGA works.

// module top (
//     input wire CLOCK_25,
//     input wire RESET_N,
//     output wire VGA_CLK,
//     output wire VGA_HS,
//     output wire VGA_VS,
//     output wire VGA_BLANK_N,
//     output wire VGA_SYNC_N,
//     output wire [7:0] VGA_R,
//     output wire [7:0] VGA_G,
//     output wire [7:0] VGA_B,
//     input wire [3:0] KEY,
//     input wire [17:0] SW
// );

//     wire pixel_clk = CLOCK_25;
//     wire resetn = KEY[0];  // active low

//     // VGA controller
//     wire display_on;
//     wire [9:0] px;   // 0..639
//     wire [8:0] py;   // 0..479

//     vga_controller vga0 (
//         .clk(pixel_clk),
//         .resetn(resetn),
//         .hsync(VGA_HS),
//         .vsync(VGA_VS),
//         .display_on(display_on),
//         .px(px),
//         .py(py)
//     );

//    // Checkerboard test pattern
//     assign VGA_R = (display_on && (px[5] ^ py[5])) ? 8'hFF : 8'h00;
//     assign VGA_G = (display_on && (px[6] ^ py[6])) ? 8'hFF : 8'h00;
//     assign VGA_B = (display_on && (px[7] ^ py[7])) ? 8'hFF : 8'h00;

//     assign VGA_CLK = pixel_clk;
//     assign VGA_BLANK_N = display_on;
//     assign VGA_SYNC_N  = 1'b0;

// endmodule


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

    // -----------------------------
    // Clock / Reset
    // -----------------------------
    wire pixel_clk = CLOCK_25;     // TODO: replace with PLL 25.175 MHz for proper VGA
    wire resetn    = KEY[0];       // active-low reset

    // -----------------------------
    // VGA Controller
    // -----------------------------
    wire display_on;
    wire [9:0] px;
    wire [8:0] py;

    vga_controller #(.H_VISIBLE(320), .V_VISIBLE(240)) vga0 (
        .clk(pixel_clk),
        .resetn(resetn),
        .hsync(VGA_HS),
        .vsync(VGA_VS),
        .display_on(display_on),
        .px(px),
        .py(py)
    );

    // Linear VGA address (for framebuffer read) - 320x240 = 76,800 pixels
    wire [18:0] vga_addr = py * 320 + px;

    // -----------------------------
    // Framebuffers (Ping-Pong)
    // -----------------------------
    // fb0 signals
    wire fb0_we;
    wire [18:0] fb0_waddr;
    wire [7:0]  fb0_wdata;
    wire [7:0]  fb_rdata0;

    framebuffer #(.DATA_WIDTH(8)) fb0 (
        .clk(pixel_clk),
        .we(fb0_we),
        .waddr(fb0_waddr),
        .wdata(fb0_wdata),
        .raddr(vga_addr),
        .rdata(fb_rdata0)
    );

    // fb1 signals
    wire fb1_we;
    wire [18:0] fb1_waddr;
    wire [7:0]  fb1_wdata;
    wire [7:0]  fb_rdata1;

    framebuffer #(.DATA_WIDTH(8)) fb1 (
        .clk(pixel_clk),
        .we(fb1_we),
        .waddr(fb1_waddr),
        .wdata(fb1_wdata),
        .raddr(vga_addr),
        .rdata(fb_rdata1)
    );

    // mux read port (VGA reads from selected buffer)
    reg reader_sel; // 0 -> fb0, 1 -> fb1
    wire [7:0] fb_rdata = reader_sel ? fb_rdata1 : fb_rdata0;

    // -----------------------------
    // Screen Controller (Writer)
    // -----------------------------
    wire fb_write_we;
    wire [18:0] fb_write_waddr;
    wire [7:0]  fb_write_wdata;
    wire sc_frame_done;

    screen_controller #(.H_VISIBLE(320), .V_VISIBLE(240)) sc0 (
        .clk(pixel_clk),
        .resetn(resetn),
        .start_frame(1'b1),
        .fc_valid(1'b1),                    // always valid for gradient test
        .pixel_index(px[7:0] ^ py[7:0]),    // simple XOR pattern
        .request_pixel(),                   // unused here
        .px(), .py(),
        .we(fb_write_we),
        .waddr(fb_write_waddr),
        .wdata(fb_write_wdata),
        .frame_done(sc_frame_done)
    );

    // -----------------------------
    // Writer Muxing
    // -----------------------------
    reg writer_sel; // 0 -> fb0, 1 -> fb1

    assign fb0_we    = (writer_sel == 1'b0) ? fb_write_we   : 1'b0;
    assign fb0_waddr = (writer_sel == 1'b0) ? fb_write_waddr: 19'd0;
    assign fb0_wdata = (writer_sel == 1'b0) ? fb_write_wdata: 8'd0;

    assign fb1_we    = (writer_sel == 1'b1) ? fb_write_we   : 1'b0;
    assign fb1_waddr = (writer_sel == 1'b1) ? fb_write_waddr: 19'd0;
    assign fb1_wdata = (writer_sel == 1'b1) ? fb_write_wdata: 8'd0;

    // -----------------------------
    // Ping-Pong Swap Logic
    // -----------------------------
    // Ping-pong swap control (combined into single always block)
    // -----------------------------
    reg pending_swap;
    reg sc_frame_done_d;
    reg vsync_d;

    always @(posedge pixel_clk or negedge resetn) begin
        if (!resetn) begin
            writer_sel <= 1'b0;
            reader_sel <= 1'b0;
            pending_swap <= 1'b0;
            sc_frame_done_d <= 1'b0;
            vsync_d <= 1'b0;
        end else begin
            // Detect frame completion
            sc_frame_done_d <= sc_frame_done;
            if (sc_frame_done && !sc_frame_done_d) begin
                // finished writing a frame
                pending_swap <= writer_sel;
                writer_sel <= ~writer_sel;  // switch writer to other buffer
            end

            // Detect VSYNC and swap reader
            vsync_d <= VGA_VS;
            if (VGA_VS && !vsync_d) begin
                // rising edge of VSYNC - safe time to swap displayed buffer
                reader_sel <= pending_swap;
                pending_swap <= 1'b0;
            end
        end
    end

    // -----------------------------
    // Color LUT → VGA
    // -----------------------------
    wire [17:0] vga_rgb18;
    color_lut lut_inst (.index(fb_rdata), .rgb(vga_rgb18));

    wire [23:0] vga_rgb24 = {
        vga_rgb18[17:12], 2'b00,
        vga_rgb18[11:6],  2'b00,
        vga_rgb18[5:0],   2'b00
    };

    assign VGA_R = vga_rgb24[23:16];
    assign VGA_G = vga_rgb24[15:8];
    assign VGA_B = vga_rgb24[7:0];

    assign VGA_CLK     = pixel_clk;
    assign VGA_BLANK_N = display_on;
    assign VGA_SYNC_N  = 1'b0;

endmodule






// ---------------------------
// Screen controller: issues pixel requests and writes computed colors into framebuffer
// ---------------------------
// module screen_controller (
//     input wire clk,
//     input wire resetn,
//     input wire start_frame,
//     input wire fc_valid,
//     input wire [7:0] pixel_index,
//     output reg request_pixel,
//     output reg [9:0] px,
//     output reg [8:0] py,
//     output reg we,
//     output reg [18:0] waddr,
//     output reg [7:0]  wdata
// );
//     // Parameterize the render area for faster simulation (defaults to 640x480)
//     parameter integer H_VISIBLE = 640;
//     parameter integer V_VISIBLE = 480;
//     // state machine
//     localparam IDLE = 2'd0;
//     localparam REQ  = 2'd1;
//     localparam WAIT = 2'd2;
//     localparam WR   = 2'd3;

//     reg [1:0] state;
//     reg [9:0] x;
//     reg [8:0] y;
//     reg [18:0] row_base; // y * H_VISIBLE

//     always @(posedge clk or negedge resetn) begin
//         if (!resetn) begin
//             state <= IDLE;
//             x <= 10'd0;
//             y <= 9'd0;
//             row_base <= 19'd0;
//             request_pixel <= 1'b0;
//             px <= 10'd0;
//             py <= 9'd0;
//             we <= 1'b0;
//             waddr <= 19'd0;
//             wdata <= 8'h00;
//         end else begin
//             // default outputs
//             request_pixel <= 1'b0;
//             we <= 1'b0;

//             case (state)
//                 IDLE: begin
//                     if (start_frame) begin
//                         state <= REQ;
//                     end
//                 end
//                 REQ: begin
//                     // request current pixel
//                     request_pixel <= 1'b1;
//                     px <= x;
//                     py <= y;
//                     state <= WAIT;
//                 end
//                 WAIT: begin
//                     if (fc_valid) begin
//                         // received pixel color in pixel_color
//                         state <= WR;
//                     end
//                 end
//                 WR: begin
//                     // write pixel into framebuffer
//                     we <= 1'b1;
//                     waddr <= row_base + x; // base + x
//                     wdata <= pixel_index;
//                     // advance to next pixel
//                     if (x == H_VISIBLE-1) begin
//                         x <= 10'd0;
//                         if (y == V_VISIBLE-1) begin
//                             y <= 9'd0;
//                             row_base <= 19'd0;
//                         end else begin
//                             y <= y + 1;
//                             row_base <= row_base + H_VISIBLE[18:0];
//                         end
//                     end else begin
//                         x <= x + 1;
//                     end
//                     state <= REQ;
//                 end
//                 default: state <= IDLE;
//             endcase
//         end
//     end
// endmodule

module screen_controller (
    input wire clk,
    input wire resetn,
    input wire start_frame,
    input wire fc_valid,
    input wire [7:0] pixel_index,
    output reg request_pixel,
    output reg [9:0] px,
    output reg [8:0] py,
    output reg we,
    output reg [18:0] waddr,
    output reg [7:0]  wdata,
    output reg frame_done    // <- NEW: pulses high for one cycle when full frame written
);
    parameter integer H_VISIBLE = 640;
    parameter integer V_VISIBLE = 480;

    localparam IDLE = 2'd0;
    localparam REQ  = 2'd1;
    localparam WAIT = 2'd2;
    localparam WR   = 2'd3;

    reg [1:0] state;
    reg [9:0] x;
    reg [8:0] y;
    reg [18:0] row_base; // y * H_VISIBLE

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            state <= IDLE;
            x <= 10'd0;
            y <= 9'd0;
            row_base <= 19'd0;
            request_pixel <= 1'b0;
            px <= 10'd0;
            py <= 9'd0;
            we <= 1'b0;
            waddr <= 19'd0;
            wdata <= 8'h00;
            frame_done <= 1'b0;
        end else begin
            // default outputs
            request_pixel <= 1'b0;
            we <= 1'b0;
            frame_done <= 1'b0;

            case (state)
                IDLE: begin
                    if (start_frame) begin
                        state <= REQ;
                        x <= 10'd0;
                        y <= 9'd0;
                        row_base <= 19'd0;
                    end
                end
                REQ: begin
                    // request current pixel
                    request_pixel <= 1'b1;
                    px <= x;
                    py <= y;
                    state <= WAIT;
                end
                WAIT: begin
                    if (fc_valid) begin
                        // received pixel data (pixel_index is valid)
                        state <= WR;
                    end
                end
                WR: begin
                    // write pixel into framebuffer
                    we <= 1'b1;
                    waddr <= row_base + x; // base + x
                    wdata <= pixel_index;
                    // advance to next pixel
                    if (x == H_VISIBLE-1) begin
                        x <= 10'd0;
                        if (y == V_VISIBLE-1) begin
                            // finished entire frame
                            y <= 9'd0;
                            row_base <= 19'd0;
                            frame_done <= 1'b1;   // pulse to indicate full frame written
                            state <= IDLE;        // wait for next start_frame
                        end else begin
                            y <= y + 1;
                            row_base <= row_base + H_VISIBLE[18:0];
                            state <= REQ;
                        end
                    end else begin
                        x <= x + 1;
                        state <= REQ;
                    end
                end
                default: state <= IDLE;
            endcase
        end
    end
endmodule

