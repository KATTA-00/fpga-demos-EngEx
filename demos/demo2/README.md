### Demo 2: Logic Visualizer (Switches -> LEDs)

#### Overview:

This combinational-logic demo maps switch inputs to LED outputs to visualize basic Boolean operations and a small multiplexer. It complements Demo 1 by focusing on pure combinational behavior.

#### Hardware Used:

- LEDs: LEDR[17:0] (lower bits used)
- Switches: SW[7:0]
- Clock: Not used

#### Functions Demonstrated:

- LEDR[0] = SW[0] AND SW[1]
- LEDR[1] = SW[0] OR SW[1]
- LEDR[2] = SW[0] XOR SW[1]
- LEDR[3] = NOT SW[0]
- LEDR[7:4] = 4:1 MUX of SW[4..7], selected by {SW[3], SW[2]}
- LEDR[10:8] = 2-bit adder result of A=SW[1:0], B=SW[3:2] (Sum[1:0] -> LEDR[9:8], Carry -> LEDR[10])
- LEDR[15:12] = Pass-through of SW[7:4]

#### File:

- logic_visualizer.v — Top-level Verilog module implementing the demo

#### Pin Assignments:

Use your DE2-115 base project or System Builder .qsf. Ensure:

- SW[17:0] -> board switches (we use SW[7:0])
- LEDR[17:0] -> board red LEDs

If creating from scratch, set the top-level entity to logic_visualizer and assign pins per the DE2-115 manual.

#### Build and Load (Quartus):

1. Create a Quartus project (device: Cyclone IV EP4CE115F29C7).
2. Add logic_visualizer.v. Set top-level entity to logic_visualizer.
3. Assign pins for SW[17:0] and LEDR[17:0].
4. Compile.
5. Program via USB-Blaster.

#### How to Use:

- Flip SW[0] and SW[1] to see LEDR[0..3] reflect AND/OR/XOR/NOT.
- Use SW[2] and SW[3] to select which of SW[4..7] shows on LEDR[7:4].
- Treat SW[1:0] and SW[3:2] as two 2-bit numbers; observe their sum on LEDR[10:8].
- SW[7:4] also mirror onto LEDR[15:12].

#### Notes:

- No clock or reset required.
- Unused LED bits are tied low.
