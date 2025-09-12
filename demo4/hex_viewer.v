// Demo 4: Hex Switch Viewer with Animation
// Board: DE2-115

module hex_viewer (
    input  wire        CLOCK_50,  // 50 MHz clock
    input  wire [17:0] SW,        // Use SW[3:0] for hex value; SW[4] optional speed (1=fast)
    input  wire [3:0]  KEY,       // KEY[0] = active-low reset
    output reg  [6:0]  HEX0       // 7-seg segments (active low)
);

    // Extract nibble
    wire [3:0] nibble = SW[3:0];

    // Previous state for change detection
    reg [3:0] prev_sw = 4'd0;

    // Animation control
    reg [3:0] anim_count = 4'd0;
    reg       anim_active = 1'b0;

    // Clock divider for animation speed (fast/slow selectable)
    reg  [25:0] div_cnt = 26'd0;
    wire [25:0] limit   = SW[4] ? 26'd5_000_000 : 26'd25_000_000; // ~10 Hz or ~2 Hz
    wire        tick    = (div_cnt >= limit);

    // Divider
    always @(posedge CLOCK_50) begin
        if (!KEY[0]) begin
            div_cnt <= 26'd0;
        end else if (tick) begin
            div_cnt <= 26'd0;
        end else begin
            div_cnt <= div_cnt + 1'b1;
        end
    end

    // Change detection and animation state
    always @(posedge CLOCK_50) begin
        if (!KEY[0]) begin
            prev_sw     <= 4'd0;
            anim_count  <= 4'd0;
            anim_active <= 1'b0;
        end else begin
            if (nibble != prev_sw) begin
                prev_sw     <= nibble;   // latch new value
                anim_count  <= 4'd0;
                anim_active <= 1'b1;     // start animation
            end else if (anim_active && tick) begin
                if (anim_count < 4'd6)
                    anim_count <= anim_count + 1'b1;
                else
                    anim_active <= 1'b0; // done
            end
        end
    end

    // Hex to 7-seg decoder (active low)
    function [6:0] hex_to_7seg;
        input [3:0] val;
        case (val)
            4'h0: hex_to_7seg = 7'b100_0000;
            4'h1: hex_to_7seg = 7'b111_1001;
            4'h2: hex_to_7seg = 7'b010_0100;
            4'h3: hex_to_7seg = 7'b011_0000;
            4'h4: hex_to_7seg = 7'b001_1001;
            4'h5: hex_to_7seg = 7'b001_0010;
            4'h6: hex_to_7seg = 7'b000_0010;
            4'h7: hex_to_7seg = 7'b111_1000;
            4'h8: hex_to_7seg = 7'b000_0000;
            4'h9: hex_to_7seg = 7'b001_0000;
            4'hA: hex_to_7seg = 7'b000_1000;
            4'hB: hex_to_7seg = 7'b000_0011;
            4'hC: hex_to_7seg = 7'b100_0110;
            4'hD: hex_to_7seg = 7'b010_0001;
            4'hE: hex_to_7seg = 7'b000_0110;
            4'hF: hex_to_7seg = 7'b000_1110;
            default: hex_to_7seg = 7'b111_1111; // blank
        endcase
    endfunction

    // Output logic with simple reveal animation
    always @(*) begin
        if (anim_active) begin
            case (anim_count)
                4'd0: HEX0 = 7'b111_1111; // blank
                4'd1: HEX0 = 7'b111_1110; // reveal segment a
                4'd2: HEX0 = 7'b111_1100; // a + b
                4'd3: HEX0 = 7'b111_1000; // a + b + c
                4'd4: HEX0 = 7'b111_0000; // a + b + c + d
                4'd5: HEX0 = 7'b110_0000; // a + b + c + d + e
                default: HEX0 = hex_to_7seg(prev_sw);
            endcase
        end else begin
            HEX0 = hex_to_7seg(prev_sw);
        end
    end

endmodule
