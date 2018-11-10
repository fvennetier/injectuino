Arduino based trip computer.

## Requirements

### Hardware

- Arduino UNO, preferably with a crystal oscillator
- MTK33x9 GPS module (or equivalent)
- LCD 2004 display module
- Shift-register based controller for LCD display
  (or I2C, but way slower)
- resistor bridge (or optocoupler) for injection probing
- resistor bridge for brownout detection
- push buttons and appropriate resistors
- super capacitor

### Software

- Arduino IDE
- TinyGPS
- LiquidCrystal
- EEPROMAnything

## Inputs

### Injector line
Injector line has to be connected to pin 3 of the Arduino UNO board,
through a voltage divider. I recommend dividing the voltage by 3:
maximum injector voltage should be capped at 15V which gives 5V after
division.

### Buttons
A0 pin.

### Input voltage
For brownout detection, input voltage has to be connected to pin 2 of the
Arduino UNO board, through a voltage divider. It must also be connected
(after the same voltage divider) to the A1 pin, for measurments.

### GPS module
Serial pins: 0 and 1.

## Outputs

### LCD 2004 display
Backlight is connected to pin 10. Shift register data is connected to A4,
clock is connected to A5.
