// Demo 3: Up/Down Binary Counter
// Board: DE2-115

module updown_counter (
    input  wire        CLOCK_50,  // 50 MHz clock
    input  wire [2:0]  SW,        // Controls: SW[0]=run, SW[1]=direction (1=up), SW[2]=speed (1=fast)
    output reg  [17:0] LEDR       // LEDs display the counter value
);

    // Clock divider for a visible tick
    // limit selects fast/slow via SW[2]
    reg  [25:0] div_cnt = 26'd0;
    wire [25:0] limit   = SW[2] ? 26'd5_000_000 : 26'd25_000_000; // ~10 Hz or ~2 Hz
    wire        tick    = (div_cnt == limit);

    always @(posedge CLOCK_50) begin
        if (tick) begin
            div_cnt <= 26'd0;
        end else begin
            div_cnt <= div_cnt + 1'b1;
        end
    end

    // Controls
    wire run    = SW[0];
    wire dir_up = SW[1];

    // 18-bit up/down counter shown on LEDR
    always @(posedge CLOCK_50) begin
        if (run && tick) begin
            if (dir_up)
                LEDR <= LEDR + 18'd1;
            else
                LEDR <= LEDR - 18'd1;
        end
    end

endmodule
