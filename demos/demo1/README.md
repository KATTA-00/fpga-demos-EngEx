### Demo 1: Knight Rider LED Sweep

#### Goal

Create a moving LED pattern (like Knight Rider / KITT car lights), sweeping left → right → left continuously.
Add a switch to control speed (slow vs fast). This makes the demo visually engaging.

#### Hardware Used

- LEDs: 18 user LEDs (LEDR[17:0]).
- Switches: 1 slide switch (SW[0]) → selects speed.
- Clock: 50 MHz system clock (CLOCK_50).

#### Concepts Demonstrated

- Clock division (slowing down 50 MHz to human-visible rate).
- Sequential shift register for pattern movement.
- Direction control (left ↔ right).
- Using switches as inputs.

#### File

- knight_rider.v — Top-level Verilog module implementing the demo

#### Pin Assignments

Assign the following top-level ports in Quartus (use DE2 System Builder .qsf or Pin Planner):

- CLOCK_50 -> 50 MHz oscillator pin
- SW0 -> Switch SW[0] pin (map this single port to the SW[0] signal)
- LEDR[17:0] -> Red LED pins

Top-level entity: knight_rider

#### Build and Load (Quartus)

1. Create a Quartus project (device: Cyclone IV EP4CE115F29C7).
2. Add knight_rider.v. Set top-level entity to knight_rider.
3. Assign pins for CLOCK_50, SW0, and LEDR[17:0].
4. Compile.
5. Program via USB-Blaster.

#### How to Use

- Observe LEDs sweep left ↔ right on LEDR[17:0].
- Flip SW0 to change speed (0 = slow, 1 = fast).

#### Notes

- Speed is set by the divide values in knight_rider.v (limit constant). Adjust to taste.
- Module drives LEDR (red LEDs). If you prefer LEDG, retarget the output bus and pin assignments accordingly.
