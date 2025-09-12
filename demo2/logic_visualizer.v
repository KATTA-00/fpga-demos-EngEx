// Demo 2: Logic Visualizer (Switches -> LEDs)
// Board: DE2-115

module logic_visualizer (
    input  wire [17:0] SW,     // Use SW[7:0] in this demo; others unused
    output wire [17:0] LEDR    // Drive LEDs
);

    // Basic logic (AND/OR/XOR/NOT)
    assign LEDR[0] = SW[0] & SW[1];
    assign LEDR[1] = SW[0] | SW[1];
    assign LEDR[2] = SW[0] ^ SW[1];
    assign LEDR[3] = ~SW[0];

    // 4:1 MUX (select with SW[3:2])
    // D0=SW[4], D1=SW[5], D2=SW[6], D3=SW[7]
    wire [3:0] D = {SW[7], SW[6], SW[5], SW[4]};
    wire [1:0] S = {SW[3], SW[2]};
    wire mux_y = (S == 2'b00) ? D[0] :
                 (S == 2'b01) ? D[1] :
                 (S == 2'b10) ? D[2] :
                                 D[3];

    // Replicate MUX output on a nibble for visibility
    assign LEDR[7:4] = {4{mux_y}};

    // 2-bit adder demo (A=SW[1:0], B=SW[3:2])
    wire [1:0] A = SW[1:0];
    wire [1:0] B = SW[3:2];

    wire [2:0] SUM = A + B; // 2-bit + 2-bit -> 3-bit result

    // Sum[1:0] -> LEDR[9:8], Carry -> LEDR[10]
    assign LEDR[9:8]  = SUM[1:0];
    assign LEDR[10]   = SUM[2];

    // Pass-through and defaults
    // Upper nibble SW[7:4] -> LEDR[15:12]
    assign LEDR[15:12] = SW[7:4];

    // Unused LED bits off
    assign LEDR[11] = 1'b0;
    assign LEDR[16] = 1'b0;
    assign LEDR[17] = 1'b0;

endmodule
