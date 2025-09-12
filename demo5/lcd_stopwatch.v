// Demo 5: LCD Stopwatch / Reaction Timer
// Board: DE2-115
// Notes:
// - Simplified HD44780 LCD parallel interface (8-bit mode), adequate for demo use.
// - Shows two modes selected by SW[0]:
//     0 = Stopwatch (SS.hh), 1 = Reaction Timer
// - Controls: KEY[0]=start/stop, KEY[1]=reset (both active-low)

module lcd_stopwatch (
    input  wire        CLOCK_50,   // 50 MHz system clock
    input  wire [17:0] SW,         // SW[0]=mode: 0=stopwatch, 1=reaction
    input  wire [3:0]  KEY,        // KEY[0]=start/stop, KEY[1]=reset (active-low)
    output reg         LCD_ON,
    output reg         LCD_EN,
    output reg         LCD_RS,
    output reg         LCD_RW,
    output reg  [7:0]  LCD_DATA
);

    // =============================================================
    // Timing generators
    // =============================================================

    // 10ms tick (for hundredths counting and LCD pacing)
    reg [19:0] div_10ms = 20'd0;          // 50e6 / 10ms = 500_000
    wire tick_10ms = (div_10ms == 20'd499_999);
    always @(posedge CLOCK_50) begin
        if (tick_10ms) div_10ms <= 20'd0; else div_10ms <= div_10ms + 1'b1;
    end

    // ~1ms tick for LCD enable pulse width and step pacing
    reg [15:0] div_1ms = 16'd0;           // 50e6 / 1ms = 50_000
    wire tick_1ms = (div_1ms == 16'd49_999);
    always @(posedge CLOCK_50) begin
        if (tick_1ms) div_1ms <= 16'd0; else div_1ms <= div_1ms + 1'b1;
    end

    // =============================================================
    // LFSR for reaction random delay
    // =============================================================
    reg [15:0] lfsr = 16'hACE1;
    wire fb = lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10];
    always @(posedge CLOCK_50) begin
        lfsr <= {lfsr[14:0], fb};
    end

    // Generate random wait in 0.5s..2.5s range (in 10ms units)
    wire [8:0] random_10ms = 9'd50 + {1'b0, lfsr[7:0]}; // 500ms + 0..2550ms

    // =============================================================
    // Stopwatch time: SS.hh (seconds, hundredths)
    // =============================================================
    reg [6:0] sec  = 7'd0;   // 0..99 (wrap)
    reg [6:0] hund = 7'd0;   // 0..99

    // =============================================================
    // Debounce/edge detect for buttons (simple, tick_1ms domain)
    // =============================================================
    reg key0_d, key0_dd; // start/stop
    reg key1_d, key1_dd; // reset
    always @(posedge CLOCK_50) begin
        if (tick_1ms) begin
            key0_d  <= ~KEY[0]; // active high when pressed
            key0_dd <= key0_d;
            key1_d  <= ~KEY[1];
            key1_dd <= key1_d;
        end
    end
    wire key0_rise = (key0_d & ~key0_dd);
    wire key1_rise = (key1_d & ~key1_dd);

    // =============================================================
    // High-level FSM
    // =============================================================
    localparam S_INIT        = 4'd0;
    localparam S_INIT1       = 4'd1;
    localparam S_INIT2       = 4'd2;
    localparam S_INIT3       = 4'd3;
    localparam S_INIT4       = 4'd4;
    localparam S_IDLE        = 4'd5;
    localparam S_RUN         = 4'd6;
    localparam S_PAUSE       = 4'd7;
    localparam S_WAIT_RANDOM = 4'd8;
    localparam S_REACTION    = 4'd9;

    reg [3:0] state = S_INIT;

    // Wait counters
    reg [9:0] wait_10ms = 10'd0; // counts 10ms ticks up to ~10.23s

    // Reaction measurement
    reg [9:0] react_10ms = 10'd0;

    // Mode
    wire reaction_mode = SW[0];

    // Run latch for stopwatch
    reg running = 1'b0;

    // =============================================================
    // LCD low-level write interface (8-bit, RS/RW/EN)
    // Simple one-shot write staged by tick_1ms for pulse width
    // =============================================================
    reg        lcd_req  = 1'b0;
    reg        lcd_is_data = 1'b0; // 0=command, 1=data
    reg [7:0]  lcd_byte = 8'h00;
    reg [1:0]  lcd_phase = 2'd0;   // 0:setup, 1:EN high, 2:EN low, 3:done
    reg        busy = 1'b0;

    always @(posedge CLOCK_50) begin
        // Default outputs
        LCD_ON <= 1'b1;
        LCD_RW <= 1'b0; // write only

        if (!busy && lcd_req) begin
            busy       <= 1'b1;
            lcd_phase  <= 2'd0;
        end

        if (busy && tick_1ms) begin
            case (lcd_phase)
                2'd0: begin
                    LCD_RS   <= lcd_is_data;
                    LCD_DATA <= lcd_byte;
                    LCD_EN   <= 1'b1;  // rising edge after setup
                    lcd_phase <= 2'd1;
                end
                2'd1: begin
                    LCD_EN   <= 1'b0;  // latch on falling edge
                    lcd_phase <= 2'd2;
                end
                2'd2: begin
                    // settle period
                    lcd_phase <= 2'd3;
                end
                2'd3: begin
                    busy     <= 1'b0;  // done
                end
            endcase
        end

        if (!busy)
            lcd_req <= 1'b0; // auto clear request
    end

    task lcd_cmd(input [7:0] b);
    begin
        lcd_is_data = 1'b0; lcd_byte = b; lcd_req = 1'b1;
    end
    endtask

    task lcd_data(input [7:0] b);
    begin
        lcd_is_data = 1'b1; lcd_byte = b; lcd_req = 1'b1;
    end
    endtask

    // =============================================================
    // Helpers to write strings and position cursor (blocking style)
    // =============================================================
    reg [7:0] cursor; // 0x00..0x27 line1, 0x40..0x67 line2 typical

    // Decimal to ASCII helpers (limited)
    function [7:0] to_digit(input [3:0] d);
        to_digit = 8'h30 + d[3:0];
    endfunction

    // Split SS.hh into ASCII bytes
    reg [7:0] a_s10, a_s1, a_h10, a_h1;
    
    // Conversion variables (moved outside always block)
    reg [6:0] s_local, h_local;
    reg [3:0] s10, s1, h10, h1;

    // LCD string writing state machine
    reg [3:0] str_idx = 4'd0;
    reg [7:0] str_len = 8'd0;
    reg [8*16-1:0] str_data;
    reg str_writing = 1'b0;

    // =============================================================
    // Main control FSM
    // =============================================================
    always @(posedge CLOCK_50) begin
        // Encode time continuously
        s_local = sec; h_local = hund;
        s10 = s_local / 7'd10; s1 = s_local % 7'd10;
        h10 = h_local / 7'd10; h1 = h_local % 7'd10;
        a_s10 = to_digit(s10);
        a_s1  = to_digit(s1);
        a_h10 = to_digit(h10);
        a_h1  = to_digit(h1);

        case (state)
            S_INIT: begin
                if (!busy) begin
                    lcd_cmd(8'h38); // function set
                    state <= S_INIT1;
                end
            end
            
            S_INIT1: begin
                if (!busy) begin
                    lcd_cmd(8'h0C); // display on, cursor off
                    state <= S_INIT2;
                end
            end
            
            S_INIT2: begin
                if (!busy) begin
                    lcd_cmd(8'h01); // clear display
                    state <= S_INIT3;
                end
            end
            
            S_INIT3: begin
                if (!busy) begin
                    lcd_cmd(8'h06); // entry mode
                    state <= S_INIT4;
                end
            end
            
            S_INIT4: begin
                if (!busy) begin
                    state <= S_IDLE;
                end
            end

            S_IDLE: begin
                if (!busy && !str_writing) begin
                    // Position and start writing line 1
                    lcd_cmd(8'h80); // line 1 position 0
                    str_writing <= 1'b1;
                    str_idx <= 4'd0;
                    if (reaction_mode) begin
                        str_data <= {"REACTION        "};
                        str_len <= 8'd16;
                    end else begin
                        str_data <= {"STOPWATCH       "};
                        str_len <= 8'd16;
                    end
                end else if (str_writing && !busy) begin
                    if (str_idx < str_len) begin
                        lcd_data(str_data[8*(15-str_idx) +: 8]);
                        str_idx <= str_idx + 1'b1;
                    end else begin
                        str_writing <= 1'b0;
                    end
                end
                
                if (key1_rise) begin
                    sec <= 0; hund <= 0; running <= 1'b0;
                end else if (key0_rise) begin
                    if (reaction_mode) begin
                        wait_10ms <= 10'd0;
                        state <= S_WAIT_RANDOM;
                    end else begin
                        running <= 1'b1; state <= S_RUN;
                    end
                end
            end

            S_RUN: begin
                // Count every 10ms
                if (tick_10ms) begin
                    hund <= hund + 1'b1;
                    if (hund == 7'd99) begin
                        hund <= 0; sec <= sec + 1'b1;
                        if (sec == 7'd99) sec <= 0;
                    end
                end
                
                // Update display
                if (!busy && !str_writing) begin
                    lcd_cmd(8'hC0); // line 2 position 0
                    str_writing <= 1'b1;
                    str_idx <= 4'd0;
                    str_data <= {"TIME ", a_s10, a_s1, ".", a_h10, a_h1, "      "};
                    str_len <= 8'd16;
                end else if (str_writing && !busy) begin
                    if (str_idx < str_len) begin
                        lcd_data(str_data[8*(15-str_idx) +: 8]);
                        str_idx <= str_idx + 1'b1;
                    end else begin
                        str_writing <= 1'b0;
                    end
                end
                
                if (key1_rise) begin
                    sec <= 0; hund <= 0; running <= 1'b0; state <= S_IDLE;
                end else if (key0_rise) begin
                    running <= 1'b0; state <= S_PAUSE;
                end
            end

            S_PAUSE: begin
                // Frozen display - no updates needed
                if (key1_rise) begin
                    sec <= 0; hund <= 0; state <= S_IDLE;
                end else if (key0_rise) begin
                    state <= S_RUN; // resume
                end
            end

            S_WAIT_RANDOM: begin
                // Wait for random_10ms ticks, then display GO and start measuring
                if (tick_10ms) begin
                    if (wait_10ms < random_10ms) begin
                        wait_10ms <= wait_10ms + 1'b1;
                    end else begin
                        react_10ms <= 10'd0;
                        state <= S_REACTION;
                    end
                end
                // If reset pressed, go back
                if (key1_rise) begin
                    sec <= 0; hund <= 0; state <= S_IDLE;
                end
            end

            S_REACTION: begin
                // Measure in 10ms units until button pressed
                if (tick_10ms) begin
                    react_10ms <= react_10ms + 1'b1;
                end
                
                // Update display with reaction time
                if (!busy && !str_writing) begin
                    // Convert react_10ms to display format
                    s_local = react_10ms / 10'd100;
                    h_local = react_10ms % 10'd100;
                    s10 = s_local / 7'd10; s1 = s_local % 7'd10;
                    h10 = h_local / 7'd10; h1 = h_local % 7'd10;
                    
                    lcd_cmd(8'hC0); // line 2 position 0
                    str_writing <= 1'b1;
                    str_idx <= 4'd0;
                    str_data <= {"REA ", to_digit(s10), to_digit(s1), ".", to_digit(h10), to_digit(h1), "      "};
                    str_len <= 8'd16;
                end else if (str_writing && !busy) begin
                    if (str_idx < str_len) begin
                        lcd_data(str_data[8*(15-str_idx) +: 8]);
                        str_idx <= str_idx + 1'b1;
                    end else begin
                        str_writing <= 1'b0;
                    end
                end
                
                if (key0_rise) begin
                    state <= S_IDLE;
                end else if (key1_rise) begin
                    sec <= 0; hund <= 0; state <= S_IDLE;
                end
            end

            default: state <= S_INIT;
        endcase
    end

endmodule
