`timescale 1ns/1ps
// ==================================================
// Single-frame framebuffer test (fixed, smoother gradient)
// - Writes a smooth scaled gradient into a single framebuffer
// - VGA reads back framebuffer with one-cycle pipeline to align rdata
// - Centered 320x240 area inside 640x480 timings
// ==================================================

module vga_controller(
    input  wire clk,          // pixel clock (~25 MHz)
    input  wire resetn,
    output reg  hsync,
    output reg  vsync,
    output reg  display_on,
    output reg [9:0] px,     // 0..319 when DISPLAY area
    output reg [8:0] py      // 0..239 when DISPLAY area
);
    // Standard 640x480@60 timings; we center a 320x240 display area
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
            // horizontal
            if (hcount == H_TOTAL - 1) begin
                hcount <= 0;
                if (vcount == V_TOTAL - 1) vcount <= 0; else vcount <= vcount + 1;
            end else begin
                hcount <= hcount + 1;
            end

            // generate sync (active low)
            if (hcount >= H_VISIBLE + H_FRONT && hcount < H_VISIBLE + H_FRONT + H_SYNC) hsync <= 0; else hsync <= 1;
            if (vcount >= V_VISIBLE + V_FRONT && vcount < V_VISIBLE + V_FRONT + V_SYNC) vsync <= 0; else vsync <= 1;

            // display area (centered 320x240)
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


// simple color LUT: map 8-bit index -> 18-bit RGB {R[5:0],G[5:0],B[5:0]}
// Smoother continuous grayscale-ish mapping (no special-case stripes).
module color_lut (
    input  wire [7:0] index,
    output reg  [17:0] rgb
);
    always @(*) begin
        // map 8-bit index to 6-bit channels by truncation to preserve smoothness
        rgb = { index[7:2], index[7:2], index[7:2] }; // smooth gray ramp
    end
endmodule


// single-port write, synchronous read inferred RAM (depth via ADDR_WIDTH)
module framebuffer #(
    parameter integer DATA_WIDTH = 8,
    parameter integer ADDR_WIDTH = 17  // 2^17 = 131072 > 320*240 = 76800
)(
    input  wire clk,
    // write port
    input  wire we,
    input  wire [ADDR_WIDTH-1:0] waddr,
    input  wire [DATA_WIDTH-1:0] wdata,
    // read port
    input  wire [ADDR_WIDTH-1:0] raddr,
    output reg  [DATA_WIDTH-1:0] rdata
);
    localparam DEPTH = (1 << ADDR_WIDTH);
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    always @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
        rdata <= mem[raddr]; // synchronous read -> value available next cycle
    end
endmodule


// screen_controller: writes a 320x240 smooth gradient into the framebuffer once,
// reports frame_done when complete.
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

    // scaling constant to map 0..319 -> 0..255 approximately:
    // scaled = (x * 3276) >> 12  -> approx x * (255/319)
    // (3276 = round(255/319 * 4096))
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
                    // start immediately after reset (one-shot)
                    x <= 0; y <= 0;
                    frame_done <= 1'b0;
                    state <= S_WR;
                end

                S_WR: begin
                    // write a smooth gradient: combine scaled_x and scaled_y
                    we <= 1'b1;
                    waddr <= current_addr;
                    // combine X and Y smoothly. This produces a gradual 2D gradient:
                    wdata <= scaled_x ^ scaled_y; // XOR of scaled values -> smooth variation

                    // increment x,y
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
                    waddr <= 17'd0;
                    wdata <= 8'd0;
                    frame_done <= 1'b1; // latched
                end

                default: state <= S_IDLE;
            endcase
        end
    end
endmodule


// (Complete top module continued)
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
    wire resetn = KEY[0];
    wire pixel_clk = CLOCK_25;

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

    // framebuffer addressing: raddr = py*320 + px using shift-add
    wire [16:0] vga_row_base = (py << 8) + (py << 6); // py*256 + py*64 = py*320
    wire [16:0] vga_addr = (display_on && frame_done) ? (vga_row_base + px) : 17'd0;

    // instantiate framebuffer (single)
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

    // screen controller writes into the same framebuffer
    wire sc_frame_done;
    screen_controller sc0 (
        .clk(pixel_clk),
        .resetn(resetn),
        .we(fb_we),
        .waddr(fb_waddr),
        .wdata(fb_wdata),
        .frame_done(sc_frame_done)
    );

    // latch frame_done so VGA only shows after first frame complete
    reg frame_done;
    always @(posedge pixel_clk or negedge resetn) begin
        if (!resetn) frame_done <= 1'b0;
        else if (sc_frame_done) frame_done <= 1'b1;
    end

    // Pipeline the framebuffer read one cycle to match synchronous read latency
    reg [7:0] fb_rdata_pipe;
    reg [16:0] raddr_pipe;
    always @(posedge pixel_clk) begin
        // pipeline the raddr and rdata: rdata becomes valid one cycle after raddr
        raddr_pipe <= vga_addr;
        fb_rdata_pipe <= fb_rdata;
    end

    // Color lookup and VGA outputs
    wire [17:0] rgb18;
    color_lut lut0 (.index(fb_rdata_pipe), .rgb(rgb18));

    assign VGA_R = (display_on && frame_done) ? {rgb18[17:12], 2'b00} : 8'd0;
    assign VGA_G = (display_on && frame_done) ? {rgb18[11:6],  2'b00} : 8'd0;
    assign VGA_B = (display_on && frame_done) ? {rgb18[5:0],   2'b00} : 8'd0;

    assign VGA_CLK = pixel_clk;
    assign VGA_BLANK_N = 1'b1; // keep enabled; RGB gated by display_on & frame_done
    assign VGA_SYNC_N  = 1'b0;

endmodule
