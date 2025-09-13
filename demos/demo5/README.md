# Demo 5: LCD Stopwatch / Reaction Timer

## Goal

- Stopwatch mode: Count seconds and hundredths (SS.hh) and display on LCD.
- Reaction mode: After a random delay, show "GO!" and measure time until the user presses the button.

## Hardware Used

- Inputs:
  - KEY[0]: Start/Stop (active-low pushbutton)
  - KEY[1]: Reset (active-low pushbutton)
  - SW[0]: Mode select (0 = Stopwatch, 1 = Reaction Timer)
- Outputs:
  - LCD 16×2 character display (LCD_RS, LCD_RW, LCD_EN, LCD_DATA[7:0], LCD_ON)
- Clock: 50 MHz system clock (CLOCK_50)

## Concepts Demonstrated

- FSM design (stopwatch and reaction timer states)
- Counters with clock division (10 ms tick)
- LCD initialization and ASCII writing (HD44780 8-bit mode)
- Simple random delay via LFSR

## File

- lcd_stopwatch.v — Top-level Verilog module implementing the demo

## Pin Assignments

Map the LCD and control pins per DE2-115 documentation:

- CLOCK_50 -> 50 MHz oscillator
- SW[17:0] -> board switches (uses SW[0])
- KEY[3:0] -> pushbuttons (uses KEY[0], KEY[1])
- LCD_DATA[7:0] -> LCD data pins
- LCD_RS, LCD_RW, LCD_EN, LCD_ON -> LCD control pins

Top-level entity: lcd_stopwatch

## Build and Load (Quartus)

1. Create a Quartus project (device: Cyclone IV EP4CE115F29C7).
2. Add lcd_stopwatch.v. Set top-level entity to lcd_stopwatch.
3. Assign pins for CLOCK_50, SW[17:0], KEY[3:0], LCD_DATA[7:0], LCD_RS, LCD_RW, LCD_EN, LCD_ON.
4. Compile.
5. Program via USB-Blaster.

## How to Use

- Mode select: SW[0]=0 stopwatch, SW[0]=1 reaction timer.
- Stopwatch mode:
  - KEY[0] toggles running; KEY[1] resets time to 00.00.
  - The display shows: line1 "STOPWATCH", line2 "TIME SS.hh".
- Reaction mode:
  - Press KEY[0] to arm; after a random delay, the display shows "GO!".
  - Press KEY[0] as fast as possible; the measured time is shown on line2.
  - Press KEY[1] to reset.

## Notes

- The LCD controller here is intentionally simple. For a robust demo, extend the LCD interface with proper busy-flag checking or conservative delays per HD44780 init sequence.
- Timing constants (10 ms, 1 ms) can be tuned for your board if needed.
