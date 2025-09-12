# Demo 4: Hex Switch Viewer with Animation

## Goal

Display a 4-bit value from switches on the HEX0 7-segment display, with a short reveal animation whenever the value changes.

## Hardware Used

- 7-segment display: HEX0[6:0] (active-low segments)
- Switches: SW[3:0] value, SW[4] speed select (optional)
- Buttons: KEY[0] active-low reset
- Clock: 50 MHz (CLOCK_50)

## Concepts Demonstrated

- Combinational hex-to-7-seg decoding
- Simple change detection with a short animation
- Clock division for human-visible timing
- Active-low button reset handling

## File

- hex_viewer.v — Top-level Verilog module implementing the demo

## Pin Assignments

Assign the following top-level ports in Quartus (use DE2 System Builder .qsf or Pin Planner):

- CLOCK_50 -> 50 MHz oscillator pin
- SW[17:0] -> board switches (uses SW[4:0])
- KEY[3:0] -> push buttons (uses KEY[0] as active-low reset)
- HEX0[6:0] -> seven-segment display 0 (active-low)

Top-level entity: hex_viewer

## Build and Load (Quartus)

1. Create a Quartus project (device: Cyclone IV EP4CE115F29C7).
2. Add hex_viewer.v. Set top-level entity to hex_viewer.
3. Assign pins for CLOCK_50, SW[17:0], KEY[3:0], and HEX0[6:0].
4. Compile.
5. Program via USB-Blaster.

## How to Use

- Set SW[3:0] to the desired hex value (0..F). HEX0 shows the value.
- When SW[3:0] changes, a short reveal animation plays, then the final digit is shown.
- SW[4]=1 uses fast animation (~10 Hz); SW[4]=0 uses slower animation (~2 Hz).
- Press KEY[0] to reset animation and timing (active-low).

## Notes

- The decoder drives active-low segments; ensure HEX0 pins are mapped as active-low in constraints.
- Timing divisors can be tuned in the code by editing the limit constant.
