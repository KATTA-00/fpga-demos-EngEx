// Demo 1: Knight Rider LED Sweep
// Board: DE2-115
module knight_rider (
    input  wire CLOCK_50,     // 50 MHz clock
    input  wire SW0,          // Speed select switch (0 = slow, 1 = fast)
    output reg  [17:0] LEDR   // LEDs
);

    // Clock divider
    reg [25:0] counter;       // 26-bit counter
    reg        direction;     // 0 = left, 1 = right
    reg [17:0] pattern;


    wire [25:0] limit = SW0 ? 26'd5_000_000 : 26'd25_000_000;  
    // ~0.1s fast, ~0.5s slow (adjust for your clock)

    always @(posedge CLOCK_50) begin
        if (counter >= limit) begin
            counter <= 0;

            // Move pattern
            if (direction == 0) begin
                // moving left
                pattern <= pattern << 1;
                if (pattern[17]) direction <= 1; // reached left end
            end else begin
                // moving right
                pattern <= pattern >> 1;
                if (pattern[0]) direction <= 0; // reached right end
            end
        end else begin
            counter <= counter + 1;
        end
    end

    // Initialize pattern
    initial begin
        pattern   = 18'b000000000000000001; // start at rightmost LED
        direction = 0;
        counter   = 0;
    end

    // Drive LEDs
    always @(*) begin
        LEDR = pattern;
    end

endmodule
