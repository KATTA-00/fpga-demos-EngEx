# Demo 3: Up/Down Binary Counter

## Goal

Show an up/down counter on the LEDs with run/pause, direction, reset, and speed control.

## Hardware Used

- LEDs: LEDR[17:0]
- Switches: SW[2:0] (SW[0]=run, SW[1]=dir, SW[2]=speed)
- Buttons: KEY[0] (active-low reset)
- Clock: 50 MHz (CLOCK_50)

## Concepts Demonstrated

- Clock division to produce a visible tick
- Register-based up/down counting
- Synchronous reset and simple control inputs

## File

- updown_counter.v — Top-level Verilog module implementing the demo

## Pin Assignments

Assign the following top-level ports in Quartus (use DE2 System Builder .qsf or Pin Planner):

- CLOCK_50 -> 50 MHz oscillator pin
- SW[17:0] -> board switches (uses SW[2:0])
- KEY[3:0] -> push buttons (uses KEY[0] as active-low reset)
- LEDR[17:0] -> red LEDs

Top-level entity: updown_counter

## Build and Load (Quartus)

1. Create a Quartus project (device: Cyclone IV EP4CE115F29C7).
2. Add updown_counter.v. Set top-level entity to updown_counter.
3. Assign pins for CLOCK_50, SW[17:0], KEY[3:0], and LEDR[17:0].
4. Compile.
5. Program via USB-Blaster.

## How to Use

- Set SW[0]=1 to run; SW[0]=0 to pause.
- SW[1]=1 counts up; SW[1]=0 counts down.
- SW[2]=1 uses fast speed (~10 Hz); SW[2]=0 uses slow speed (~2 Hz).
- Press KEY[0] to reset the counter to 0 (active-low).

## Notes

- Adjust the divider constants in the code if you want different speeds.
- Counter width matches LEDR[17:0]; it wraps naturally on overflow/underflow.
