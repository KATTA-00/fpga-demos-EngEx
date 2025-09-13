`timescale 1ns/1ps
// =======================================================
// Debug top (updated): writer-select moved to SW[1], speed moved to SW[3:2]
// - SW[1] = 1 => gradient writer (fast test)
// - SW[1] = 0 => fractal writer (real fractal)
// - SW[0] = animation enable
// - SW[3:2] = speed select: 00=+1, 01=+2, 10=+4, 11=+8 per VSYNC
// =======================================================

// ----------------- VGA controller (centered 320x240) -----------------
module vga_controller(
    input  wire clk,
    input  wire resetn,
    output reg  hsync,
    output reg  vsync,
    output reg  display_on,
    output reg [9:0] px,
    output reg [8:0] py,
    // new: full visible coordinates and visible region flag
    output reg        vis,
    output reg [9:0]  sx,
    output reg [8:0]  sy
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
            vis <= 1'b0; sx <= 10'd0; sy <= 9'd0;
        end else begin
            if (hcount == H_TOTAL - 1) begin
                hcount <= 0;
                if (vcount == V_TOTAL - 1) vcount <= 0; else vcount <= vcount + 1;
            end else hcount <= hcount + 1;

            hsync <= ~((hcount >= H_VISIBLE + H_FRONT) && (hcount < H_VISIBLE + H_FRONT + H_SYNC));
            vsync <= ~((vcount >= V_VISIBLE + V_FRONT) && (vcount < V_VISIBLE + V_FRONT + V_SYNC));

            // visible region within 640x480
            vis <= (hcount < H_VISIBLE) && (vcount < V_VISIBLE);
            if (vis) begin
                sx <= hcount[9:0];
                sy <= vcount[8:0];
            end else begin
                sx <= 10'd0;
                sy <= 9'd0;
            end

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


// ----------------- Bottom Text overlay: "ESCAL PERAMORPHIQ" (8x8 font) -----------------
module text_overlay_bottom #(
    parameter integer SCALE_LOG2 = 2
)(
    input  wire [9:0] sx,
    input  wire [8:0] sy,
    output reg        text_on
);
    localparam integer SCALE = (1 << SCALE_LOG2);
    localparam integer NCHARS = 17; // ESCAL (5) + space (1) + PERAMORPHIQ (11)
    localparam integer RAW_W = NCHARS * 8;
    localparam integer RAW_H = 8;
    localparam integer W = RAW_W * SCALE;   // 136 * SCALE
    localparam integer H = RAW_H * SCALE;   // 8 * SCALE

    // Visible screen geometry
    localparam integer SCREEN_W = 640;
    localparam integer TOP_BORDER = 120; // (480-240)/2
    // Place below the 320x240 box with 8px padding: Y = 120 + 240 + 8 = 368
    localparam [9:0] X0 = (SCREEN_W - W)/2;          // center horizontally
    localparam [8:0] Y0 = TOP_BORDER + 9'd240 + 9'd8 + 9'd20;

    wire [9:0] x_local = sx - X0;
    wire [8:0] y_local = sy - Y0;
    wire [7:0] x_char = x_local >> SCALE_LOG2; // 0..(RAW_W-1)
    wire [3:0] y_char = y_local >> SCALE_LOG2; // 0..7
    wire [4:0] char_pos = x_char[7:3];         // character position 0..(NCHARS-1)
    wire [2:0] col_idx  = x_char[2:0];         // column within glyph 0..7

    // Letter enumeration
    localparam [4:0]
        L_P = 5'd0, L_E = 5'd1, L_R = 5'd2, L_A = 5'd3, L_C = 5'd4, L_O = 5'd5, L_M = 5'd6,
        L_S = 5'd7, L_L = 5'd8, L_H = 5'd9, L_I = 5'd10, L_Q = 5'd11, L_SPACE = 5'd31;

    // Map character position into a letter enum for the phrase "ESCAL PERAMORPHIQ"
    reg [4:0] letter_idx;
    always @(*) begin
        case (char_pos)
            // "ESCAL"
            5'd0:  letter_idx = L_E;
            5'd1:  letter_idx = L_S;
            5'd2:  letter_idx = L_C;
            5'd3:  letter_idx = L_A;
            5'd4:  letter_idx = L_L;
            // space
            5'd5:  letter_idx = L_SPACE;
            // "PERAMORPHIQ"
            5'd6:  letter_idx = L_P;
            5'd7:  letter_idx = L_E;
            5'd8:  letter_idx = L_R;
            5'd9:  letter_idx = L_A;
            5'd10: letter_idx = L_M;
            5'd11: letter_idx = L_O;
            5'd12: letter_idx = L_R;
            5'd13: letter_idx = L_P;
            5'd14: letter_idx = L_H;
            5'd15: letter_idx = L_I;
            5'd16: letter_idx = L_Q;
            default: letter_idx = L_SPACE;
        endcase
    end

    // temp to hold glyph row bits for Verilog-2001 indexing
    reg [7:0] glyph_bits;

    // Return one 8-bit row of the glyph for a given letter index at row y
    function [7:0] glyph_row2;
        input [4:0] idx;
        input [3:0] y;
        begin
            case (idx)
                L_P: begin
                    case (y)
                        0: glyph_row2 = 8'b11111110;
                        1: glyph_row2 = 8'b10000010;
                        2: glyph_row2 = 8'b10000010;
                        3: glyph_row2 = 8'b11111110;
                        4: glyph_row2 = 8'b10000000;
                        5: glyph_row2 = 8'b10000000;
                        6: glyph_row2 = 8'b10000000;
                        default: glyph_row2 = 8'b00000000;
                    endcase
                end
                L_E: begin
                    case (y)
                        0: glyph_row2 = 8'b11111110;
                        1: glyph_row2 = 8'b10000000;
                        2: glyph_row2 = 8'b10000000;
                        3: glyph_row2 = 8'b11111110;
                        4: glyph_row2 = 8'b10000000;
                        5: glyph_row2 = 8'b10000000;
                        6: glyph_row2 = 8'b11111110;
                        default: glyph_row2 = 8'b00000000;
                    endcase
                end
                L_R: begin
                    case (y)
                        0: glyph_row2 = 8'b11111100;
                        1: glyph_row2 = 8'b10000010;
                        2: glyph_row2 = 8'b10000010;
                        3: glyph_row2 = 8'b11111100;
                        4: glyph_row2 = 8'b10010000;
                        5: glyph_row2 = 8'b10001000;
                        6: glyph_row2 = 8'b10000100;
                        default: glyph_row2 = 8'b00000000;
                    endcase
                end
                L_A: begin
                    case (y)
                        0: glyph_row2 = 8'b01111100;
                        1: glyph_row2 = 8'b10000010;
                        2: glyph_row2 = 8'b10000010;
                        3: glyph_row2 = 8'b11111110;
                        4: glyph_row2 = 8'b10000010;
                        5: glyph_row2 = 8'b10000010;
                        6: glyph_row2 = 8'b10000010;
                        default: glyph_row2 = 8'b00000000;
                    endcase
                end
                L_C: begin
                    case (y)
                        0: glyph_row2 = 8'b01111110;
                        1: glyph_row2 = 8'b10000000;
                        2: glyph_row2 = 8'b10000000;
                        3: glyph_row2 = 8'b10000000;
                        4: glyph_row2 = 8'b10000000;
                        5: glyph_row2 = 8'b10000000;
                        6: glyph_row2 = 8'b01111110;
                        default: glyph_row2 = 8'b00000000;
                    endcase
                end
                L_O: begin
                    case (y)
                        0: glyph_row2 = 8'b01111100;
                        1: glyph_row2 = 8'b10000010;
                        2: glyph_row2 = 8'b10000010;
                        3: glyph_row2 = 8'b10000010;
                        4: glyph_row2 = 8'b10000010;
                        5: glyph_row2 = 8'b10000010;
                        6: glyph_row2 = 8'b01111100;
                        default: glyph_row2 = 8'b00000000;
                    endcase
                end
                L_Q: begin
                    case (y)
                        0: glyph_row2 = 8'b01111100; // like 'O' with a tail
                        1: glyph_row2 = 8'b10000010;
                        2: glyph_row2 = 8'b10000010;
                        3: glyph_row2 = 8'b10000010;
                        4: glyph_row2 = 8'b10010010; // add diagonal pixel
                        5: glyph_row2 = 8'b10001010; // add diagonal pixel
                        6: glyph_row2 = 8'b01111101; // bottom-right tail
                        default: glyph_row2 = 8'b00000000;
                    endcase
                end
                L_M: begin
                    case (y)
                        0: glyph_row2 = 8'b10000010;
                        1: glyph_row2 = 8'b11000110;
                        2: glyph_row2 = 8'b10101010;
                        3: glyph_row2 = 8'b10010010;
                        4: glyph_row2 = 8'b10000010;
                        5: glyph_row2 = 8'b10000010;
                        6: glyph_row2 = 8'b10000010;
                        default: glyph_row2 = 8'b00000000;
                    endcase
                end
                // New glyphs
                L_S: begin
                    case (y)
                        0: glyph_row2 = 8'b01111110;
                        1: glyph_row2 = 8'b10000000;
                        2: glyph_row2 = 8'b10000000;
                        3: glyph_row2 = 8'b01111100;
                        4: glyph_row2 = 8'b00000010;
                        5: glyph_row2 = 8'b00000010;
                        6: glyph_row2 = 8'b11111100;
                        default: glyph_row2 = 8'b00000000;
                    endcase
                end
                L_L: begin
                    case (y)
                        0: glyph_row2 = 8'b10000000;
                        1: glyph_row2 = 8'b10000000;
                        2: glyph_row2 = 8'b10000000;
                        3: glyph_row2 = 8'b10000000;
                        4: glyph_row2 = 8'b10000000;
                        5: glyph_row2 = 8'b10000000;
                        6: glyph_row2 = 8'b11111110;
                        default: glyph_row2 = 8'b00000000;
                    endcase
                end
                L_H: begin
                    case (y)
                        0: glyph_row2 = 8'b10000010;
                        1: glyph_row2 = 8'b10000010;
                        2: glyph_row2 = 8'b10000010;
                        3: glyph_row2 = 8'b11111110;
                        4: glyph_row2 = 8'b10000010;
                        5: glyph_row2 = 8'b10000010;
                        6: glyph_row2 = 8'b10000010;
                        default: glyph_row2 = 8'b00000000;
                    endcase
                end
                L_I: begin
                    case (y)
                        0: glyph_row2 = 8'b11111110;
                        1: glyph_row2 = 8'b00010000;
                        2: glyph_row2 = 8'b00010000;
                        3: glyph_row2 = 8'b00010000;
                        4: glyph_row2 = 8'b00010000;
                        5: glyph_row2 = 8'b00010000;
                        6: glyph_row2 = 8'b11111110;
                        default: glyph_row2 = 8'b00000000;
                    endcase
                end
                default: glyph_row2 = 8'b00000000; // space or undefined
            endcase
        end
    endfunction

    always @(*) begin
        text_on = 1'b0;
        glyph_bits = 8'b0;
        if (sy >= Y0 && sy < (Y0 + H) && sx >= X0 && sx < (X0 + W)) begin
            glyph_bits = glyph_row2(letter_idx, y_char);
            text_on = glyph_bits[7 - col_idx];
        end
    end
endmodule

// ----------------- Rainbow LUT (rotated channels) -----------------
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


// ----------------- Synchronous framebuffer -----------------
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


// ----------------- Text overlay: hardcoded PERACOM (8x8 font, 7 chars) -----------------
module text_overlay #(
    // SCALE_LOG2=2 => scale x4 (each font pixel becomes 4x4 block)
    parameter integer SCALE_LOG2 = 2
)(
    // full-screen visible coordinates (0..639, 0..479)
    input  wire [9:0] sx,
    input  wire [8:0] sy,
    output reg        text_on
);
    localparam integer SCALE = (1 << SCALE_LOG2);
    // 7 chars * 8px = 56px wide, 8px tall (before scaling)
    localparam integer RAW_W = 56;
    localparam integer RAW_H = 8;
    localparam integer W = RAW_W * SCALE;
    localparam integer H = RAW_H * SCALE;

    // Place the banner centered in the 640-wide visible area,
    // and vertically above the 320x240 active box (black top border = 120 lines).
    localparam integer SCREEN_W = 640;
    localparam integer TOP_BORDER = 120; // (480-240)/2
    localparam [9:0] X0 = (SCREEN_W - W)/2;           // center horizontally: (640 - W)/2 = 208 when W=224
    localparam [8:0] Y0 = TOP_BORDER - H - 9'd8;      // 8px padding above the box: 120 - H - 8 = 80 when H=32

    wire [9:0] x_local = sx - X0;
    wire [8:0] y_local = sy - Y0;
    wire [5:0] x_char = x_local >> SCALE_LOG2; // 0..55
    wire [3:0] y_char = y_local >> SCALE_LOG2; // 0..7
    wire [2:0] char_idx = x_char[5:3];        // which of the 7 letters (0..6)
    wire [2:0] col_idx  = x_char[2:0];        // column within 8px glyph (0..7)

    // temp to hold glyph row bits for Verilog-2001 indexing
    reg [7:0] glyph_bits;

    // Return one 8-bit row of the glyph for a given letter index at row y
    function [7:0] glyph_row;
        input [2:0] idx; // 0:P, 1:E, 2:R, 3:A, 4:C, 5:O, 6:M
        input [3:0] y;   // 0..7
        begin
            case (idx)
                3'd0: begin // P
                    case (y)
                        0: glyph_row = 8'b11111110;
                        1: glyph_row = 8'b10000010;
                        2: glyph_row = 8'b10000010;
                        3: glyph_row = 8'b11111110;
                        4: glyph_row = 8'b10000000;
                        5: glyph_row = 8'b10000000;
                        6: glyph_row = 8'b10000000;
                        default: glyph_row = 8'b00000000;
                    endcase
                end
                3'd1: begin // E
                    case (y)
                        0: glyph_row = 8'b11111110;
                        1: glyph_row = 8'b10000000;
                        2: glyph_row = 8'b10000000;
                        3: glyph_row = 8'b11111110;
                        4: glyph_row = 8'b10000000;
                        5: glyph_row = 8'b10000000;
                        6: glyph_row = 8'b11111110;
                        default: glyph_row = 8'b00000000;
                    endcase
                end
                3'd2: begin // R
                    case (y)
                        0: glyph_row = 8'b11111100;
                        1: glyph_row = 8'b10000010;
                        2: glyph_row = 8'b10000010;
                        3: glyph_row = 8'b11111100;
                        4: glyph_row = 8'b10010000;
                        5: glyph_row = 8'b10001000;
                        6: glyph_row = 8'b10000100;
                        default: glyph_row = 8'b00000000;
                    endcase
                end
                3'd3: begin // A
                    case (y)
                        0: glyph_row = 8'b01111100;
                        1: glyph_row = 8'b10000010;
                        2: glyph_row = 8'b10000010;
                        3: glyph_row = 8'b11111110;
                        4: glyph_row = 8'b10000010;
                        5: glyph_row = 8'b10000010;
                        6: glyph_row = 8'b10000010;
                        default: glyph_row = 8'b00000000;
                    endcase
                end
                3'd4: begin // C
                    case (y)
                        0: glyph_row = 8'b01111110;
                        1: glyph_row = 8'b10000000;
                        2: glyph_row = 8'b10000000;
                        3: glyph_row = 8'b10000000;
                        4: glyph_row = 8'b10000000;
                        5: glyph_row = 8'b10000000;
                        6: glyph_row = 8'b01111110;
                        default: glyph_row = 8'b00000000;
                    endcase
                end
                3'd5: begin // O
                    case (y)
                        0: glyph_row = 8'b01111100;
                        1: glyph_row = 8'b10000010;
                        2: glyph_row = 8'b10000010;
                        3: glyph_row = 8'b10000010;
                        4: glyph_row = 8'b10000010;
                        5: glyph_row = 8'b10000010;
                        6: glyph_row = 8'b01111100;
                        default: glyph_row = 8'b00000000;
                    endcase
                end
                3'd6: begin // M
                    case (y)
                        0: glyph_row = 8'b10000010;
                        1: glyph_row = 8'b11000110;
                        2: glyph_row = 8'b10101010;
                        3: glyph_row = 8'b10010010;
                        4: glyph_row = 8'b10000010;
                        5: glyph_row = 8'b10000010;
                        6: glyph_row = 8'b10000010;
                        default: glyph_row = 8'b00000000;
                    endcase
                end
                default: glyph_row = 8'b00000000;
            endcase
        end
    endfunction

    always @(*) begin
        text_on = 1'b0;
        glyph_bits = 8'b0;
        if (sy >= Y0 && sy < (Y0 + H) && sx >= X0 && sx < (X0 + W)) begin
            // sample the appropriate glyph row for this character
            glyph_bits = glyph_row(char_idx, y_char);
            // left-most pixel is MSB (bit 7)
            text_on = glyph_bits[7 - col_idx];
        end
    end
endmodule


// ----------------- gradient screen writer (fast test writer) -----------------
module screen_writer (
    input  wire clk,
    input  wire resetn,
    input  wire start_frame,
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

    wire [16:0] row_base = (y << 8) + (y << 6);
    wire [16:0] current_addr = row_base + x;

    // scale 0..319 -> 0..255: scaled = (x * 3276) >>12
    wire [25:0] prod_x = x * 17'd3276;
    wire [25:0] prod_y = y * 17'd3276;
    wire [7:0] scaled_x = prod_x[25:12];
    wire [7:0] scaled_y = prod_y[25:12];

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            x <= 0; y <= 0; running <= 1'b0;
            we <= 1'b0; waddr <= 17'd0; wdata <= 8'd0; frame_done <= 1'b0;
        end else begin
            we <= 1'b0;
            if (!running) begin
                frame_done <= 1'b0;
                if (start_frame) begin
                    running <= 1'b1;
                    x <= 0; y <= 0;
                end
            end else begin
                we <= 1'b1;
                waddr <= current_addr;
                wdata <= scaled_x + scaled_y; // smooth additive blend

                if (x == H_VISIBLE - 1) begin
                    x <= 0;
                    if (y == V_VISIBLE - 1) begin
                        running <= 1'b0;
                        frame_done <= 1'b1;
                    end else y <= y + 1;
                end else x <= x + 1;
            end
        end
    end
endmodule


// ----------------- Mandelbrot fractal core (single iteration per clock) -----------------
module fractal_core #(
    parameter PIXEL_WIDTH = 10,
    parameter LINE_WIDTH  = 9,
    parameter integer H_VISIBLE = 320,
    parameter integer V_VISIBLE = 240,
    parameter integer MAX_ITER = 512  // reasonable default for debug
)(
    input  wire clk,
    input  wire resetn,
    input  wire signed [31:0] cx_min,
    input  wire signed [31:0] cx_max,
    input  wire signed [31:0] cy_min,
    input  wire signed [31:0] cy_max,
    input  wire request_pixel,
    input  wire [PIXEL_WIDTH-1:0] px,
    input  wire [LINE_WIDTH-1:0] py,
    output reg  valid,
    output reg  [7:0] iter_count
);
    reg signed [31:0] c_re, c_im;   // Q16.16
    reg signed [31:0] z_re, z_im;   // Q16.16
    // keep old regs declared (not used after fix) to minimize diff
    reg signed [63:0] z_re_sq, z_im_sq, z_re_im;
    reg [15:0] iter;
    reg running;
    reg [1:0] map_stage;
    reg signed [63:0] tmp_re_num, tmp_im_num;
    wire signed [31:0] cx_range = cx_max - cx_min;
    wire signed [31:0] cy_range = cy_max - cy_min;

    // Combinational math from current z (Q16.16)
    wire signed [63:0] z_re_sq_w = $signed(z_re) * $signed(z_re);      // Q32.32
    wire signed [63:0] z_im_sq_w = $signed(z_im) * $signed(z_im);      // Q32.32
    wire signed [63:0] z_re_im_w = $signed(z_re) * $signed(z_im);      // Q32.32 (before *2)
    wire signed [63:0] mag2_w    = z_re_sq_w + z_im_sq_w;              // Q32.32
    localparam signed [63:0] ESC2_Q32 = 64'sd4 <<< 32;                 // 4.0 in Q32.32

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            valid <= 0; iter_count <= 0;
            running <= 0; iter <= 0; map_stage <= 0;
            c_re <= 0; c_im <= 0; z_re <= 0; z_im <= 0;
            tmp_re_num <= 0; tmp_im_num <= 0;
        end else begin
            valid <= 0;
            if (!running && request_pixel) begin
                tmp_re_num <= cx_range * px;
                tmp_im_num <= cy_range * py;
                map_stage <= 1;
                running <= 1;
                iter <= 0;
            end else if (running && map_stage == 1) begin
                if ((H_VISIBLE > 1) && (V_VISIBLE > 1)) begin
                    c_re <= cx_min + $signed(tmp_re_num / (H_VISIBLE - 1));
                    c_im <= cy_min + $signed(tmp_im_num / (V_VISIBLE - 1));
                end else begin
                    c_re <= cx_min; c_im <= cy_min;
                end
                z_re <= 32'sd0; z_im <= 32'sd0;
                map_stage <= 2;
            end else if (running && map_stage == 2) begin
                // Escape test using current z in Q32.32: |z|^2 > 4.0
                if (mag2_w > ESC2_Q32) begin
                    valid <= 1'b1;
                    iter_count <= iter[7:0];
                    running <= 1'b0; map_stage <= 2'd0;
                end else if (iter == MAX_ITER - 1) begin
                    valid <= 1'b1;
                    iter_count <= MAX_ITER[7:0];
                    running <= 1'b0; map_stage <= 2'd0;
                end else begin
                    // z = z^2 + c; bring back to Q16.16 (>>>16). For imaginary do (2*z_re*z_im)>>16
                    z_re <= ($signed(z_re_sq_w - z_im_sq_w) >>> 16) + c_re;
                    z_im <= ($signed(z_re_im_w) >>> 15) + c_im; // (2*z_re*z_im)>>16
                    iter <= iter + 16'd1;
                end
            end
        end
    end
endmodule


// ----------------- fractal_writer (requests from core, writes iter to RAM) -----------------
module fractal_writer (
    input  wire clk,
    input  wire resetn,
    input  wire start_frame,
    output reg  request_pixel,
    output reg  [9:0] fc_px,
    output reg  [8:0] fc_py,
    input  wire fc_valid,
    input  wire [7:0] fc_iter,
    output reg  we,
    output reg  [16:0] waddr,
    output reg  [7:0] wdata,
    output reg  frame_done
);
    parameter H_VISIBLE = 320;
    parameter V_VISIBLE = 240;

    reg [9:0] x;
    reg [8:0] y;
    reg running;
    reg pending_request;

    wire [16:0] row_base = (y << 8) + (y << 6);
    wire [16:0] current_addr = row_base + x;

    always @(posedge clk or negedge resetn) begin
        if (!resetn) begin
            x <= 0; y <= 0;
            request_pixel <= 1'b0; fc_px <= 10'd0; fc_py <= 9'd0;
            we <= 1'b0; waddr <= 17'd0; wdata <= 8'd0;
            frame_done <= 1'b0;
            running <= 1'b0; pending_request <= 1'b0;
        end else begin
            request_pixel <= 1'b0;
            we <= 1'b0;
            if (!running) begin
                frame_done <= 1'b0;
                if (start_frame) begin
                    running <= 1'b1;
                    x <= 0; y <= 0;
                    pending_request <= 1'b0;
                end
            end else begin
                if (!pending_request) begin
                    request_pixel <= 1'b1;
                    fc_px <= x; fc_py <= y;
                    pending_request <= 1'b1;
                end else begin
                    if (fc_valid) begin
                        we <= 1'b1;
                        waddr <= current_addr;
                        wdata <= fc_iter;
                        if (x == H_VISIBLE - 1) begin
                            x <= 10'd0;
                            if (y == V_VISIBLE - 1) begin
                                running <= 1'b0;
                                frame_done <= 1'b1;
                                pending_request <= 1'b0;
                            end else begin
                                y <= y + 1;
                                pending_request <= 1'b0;
                            end
                        end else begin
                            x <= x + 1;
                            pending_request <= 1'b0;
                        end
                    end
                end
            end
        end
    end
endmodule


// ----------------- Top with both writers and muxing (SW mapping updated) -----------------
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
    // Controls (updated):
    // SW[1] = 1 => gradient writer (fast test)
    // SW[1] = 0 => fractal writer (real fractal)
    // SW[0] = animation enable
    // SW[3:2] = speed select (00=+1,01=+2,10=+4,11=+8)
    wire use_gradient = SW[1];

    // reset and pixel clock
    wire resetn = KEY[0];
    wire pixel_clk = CLOCK_25;

    // VGA controller
    wire display_on;
    wire [9:0] px;
    wire [8:0] py;
    wire vis;
    wire [9:0] sx;
    wire [8:0] sy;
    vga_controller vga0 (
        .clk(pixel_clk), .resetn(resetn),
        .hsync(VGA_HS), .vsync(VGA_VS),
        .display_on(display_on), .px(px), .py(py),
        .vis(vis), .sx(sx), .sy(sy)
    );

    // read address
    wire [16:0] vga_row_base = (py << 8) + (py << 6);
    wire [16:0] vga_addr = (display_on && frame_display_ready) ? (vga_row_base + px) : 17'd0;

    // two framebuffers
    wire fb0_we, fb1_we;
    wire [16:0] fb0_waddr, fb1_waddr;
    wire [7:0]  fb0_wdata, fb1_wdata;
    wire [7:0]  fb0_rdata, fb1_rdata;

    framebuffer #(.DATA_WIDTH(8), .ADDR_WIDTH(17)) fb0 (.clk(pixel_clk), .we(fb0_we), .waddr(fb0_waddr), .wdata(fb0_wdata),
                                                      .raddr(vga_addr), .rdata(fb0_rdata));
    framebuffer #(.DATA_WIDTH(8), .ADDR_WIDTH(17)) fb1 (.clk(pixel_clk), .we(fb1_we), .waddr(fb1_waddr), .wdata(fb1_wdata),
                                                      .raddr(vga_addr), .rdata(fb1_rdata));

    // read mux
    reg display_buffer; // 0->fb0, 1->fb1
    wire [7:0] fb_rdata = display_buffer ? fb1_rdata : fb0_rdata;

    // ---------------- Writers: gradient writer + fractal writer (both present) ----------------
    wire grad_we, frac_we;
    wire [16:0] grad_waddr, frac_waddr;
    wire [7:0] grad_wdata, frac_wdata;
    wire grad_done, frac_done;

    reg start_frame_reg;
    // gradient writer instance
    screen_writer writer_grad (.clk(pixel_clk), .resetn(resetn), .start_frame(start_frame_grad),
                               .we(grad_we), .waddr(grad_waddr), .wdata(grad_wdata), .frame_done(grad_done));
    // fractal core & writer
    // fractal core signals
    wire fc_request;
    wire [9:0] fc_px;
    wire [8:0] fc_py;
    wire fc_valid;
    wire [7:0] fc_iter;

    // fractal core instance (lowered MAX_ITER to 32 for testing)
    localparam signed [31:0] CX_MIN = -32'sd131072; // -2.0 Q16.16
    localparam signed [31:0] CX_MAX =  32'sd65536;  //  1.0
    localparam signed [31:0] CY_MIN = -32'sd78643;  // -1.2
    localparam signed [31:0] CY_MAX =  32'sd78643;  //  1.2

    fractal_core #(.PIXEL_WIDTH(10), .LINE_WIDTH(9), .H_VISIBLE(320), .V_VISIBLE(240), .MAX_ITER(32))
        fc0 (.clk(pixel_clk), .resetn(resetn),
             .cx_min(CX_MIN), .cx_max(CX_MAX), .cy_min(CY_MIN), .cy_max(CY_MAX),
             .request_pixel(fc_request), .px(fc_px), .py(fc_py),
             .valid(fc_valid), .iter_count(fc_iter));

    // fractal writer instance
    fractal_writer writer_frac (.clk(pixel_clk), .resetn(resetn), .start_frame(start_frame_frac),
                                .request_pixel(fc_request), .fc_px(fc_px), .fc_py(fc_py),
                                .fc_valid(fc_valid), .fc_iter(fc_iter),
                                .we(frac_we), .waddr(frac_waddr), .wdata(frac_wdata), .frame_done(frac_done));

    // ----------------- Mux writer outputs based on SW[1] -----------------
    wire writer_we      = use_gradient ? grad_we  : frac_we;
    wire [16:0] writer_waddr = use_gradient ? grad_waddr : frac_waddr;
    wire [7:0]  writer_wdata = use_gradient ? grad_wdata : frac_wdata;
    wire writer_done     = use_gradient ? grad_done : frac_done;

    // Gate start_frame per selected writer
    wire start_frame_grad = start_frame_reg & use_gradient;
    wire start_frame_frac = start_frame_reg & ~use_gradient;

    // route writer to active RAM (write_buffer chooses fb0 or fb1)
    reg write_buffer; // 0->fb0 ; 1->fb1
    assign fb0_we    = (write_buffer == 1'b0) ? writer_we    : 1'b0;
    assign fb0_waddr  = (write_buffer == 1'b0) ? writer_waddr  : 17'd0;
    assign fb0_wdata  = (write_buffer == 1'b0) ? writer_wdata  : 8'd0;

    assign fb1_we    = (write_buffer == 1'b1) ? writer_we    : 1'b0;
    assign fb1_waddr  = (write_buffer == 1'b1) ? writer_waddr  : 17'd0;
    assign fb1_wdata  = (write_buffer == 1'b1) ? writer_wdata  : 8'd0;

    // ----------------- Frame & swap control (ping-pong) -----------------
    reg frame_display_ready;
    reg swap_pending;
    reg writer_done_prev;
    reg [1:0] vsync_sync;
    reg [7:0] frame_phase;
    reg [7:0] phase_inc;
    reg [1:0] key1_sync;

    // initialize in reset branch
    always @(posedge pixel_clk or negedge resetn) begin
        if (!resetn) begin
            display_buffer <= 1'b0;
            write_buffer <= 1'b0;
            start_frame_reg <= 1'b1;
            swap_pending <= 1'b0;
            frame_display_ready <= 1'b0;
            vsync_sync <= 2'b11;
            writer_done_prev <= 1'b0;
            frame_phase <= 8'd0;
            phase_inc <= 8'd1;
            key1_sync <= 2'b11;
        end else begin
            vsync_sync <= {vsync_sync[0], VGA_VS};
            key1_sync <= {key1_sync[0], KEY[1]};

            // speed mapping now on SW[3:2]
            case (SW[3:2])
                2'b00: phase_inc <= 8'd1;
                2'b01: phase_inc <= 8'd2;
                2'b10: phase_inc <= 8'd4;
                default: phase_inc <= 8'd8;
            endcase

            // detect writer done (rising edge)
            if (writer_done && !writer_done_prev) begin
                frame_display_ready <= 1'b1;
                swap_pending <= 1'b1;
                write_buffer <= ~write_buffer;
                start_frame_reg <= 1'b0;
            end
            writer_done_prev <= writer_done;

            // manual step on KEY[1]
            if (key1_sync == 2'b10) frame_phase <= frame_phase + phase_inc;

            // on VSYNC rising edge do swap and phase advance if enabled
            if (vsync_sync == 2'b01) begin
                if (swap_pending) begin
                    display_buffer <= ~display_buffer;
                    swap_pending <= 1'b0;
                end
                if (SW[0]) frame_phase <= frame_phase + phase_inc;
                start_frame_reg <= 1'b1; // allow next render
            end
        end
    end

    // pipeline synchronous ram read
    reg [7:0] fb_rdata_pipe;
    reg [16:0] raddr_pipe;
    always @(posedge pixel_clk) begin
        raddr_pipe <= vga_addr;
        fb_rdata_pipe <= fb_rdata;
    end

    // palette index and color
    wire [7:0] lut_index = fb_rdata_pipe + frame_phase;
    wire [17:0] rgb18;
    color_lut_rainbow lut0 (.index(lut_index), .rgb(rgb18));
    // Text overlays in borders
    wire text_on_top;
    wire text_on_bottom;
    text_overlay #(.SCALE_LOG2(2))        txt_top (.sx(sx), .sy(sy), .text_on(text_on_top));
    text_overlay_bottom #(.SCALE_LOG2(2)) txt_bot (.sx(sx), .sy(sy), .text_on(text_on_bottom));
    // Draw overlay anywhere in the visible 640x480 region; image only inside display_on
    wire [7:0] img_r = (display_on && frame_display_ready) ? {rgb18[17:12], 2'b00} : 8'd0;
    wire [7:0] img_g = (display_on && frame_display_ready) ? {rgb18[11:6],  2'b00} : 8'd0;
    wire [7:0] img_b = (display_on && frame_display_ready) ? {rgb18[5:0],   2'b00} : 8'd0;
    wire overlay_on = text_on_top | text_on_bottom;
    assign VGA_R = vis ? (overlay_on ? 8'hFF : img_r) : 8'd0;
    assign VGA_G = vis ? (overlay_on ? 8'hFF : img_g) : 8'd0;
    assign VGA_B = vis ? (overlay_on ? 8'hFF : img_b) : 8'd0;

    assign VGA_CLK = pixel_clk;
    assign VGA_BLANK_N = 1'b1;
    assign VGA_SYNC_N  = 1'b0;

endmodule
