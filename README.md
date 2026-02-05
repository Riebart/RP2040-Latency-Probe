# RP2040-Latency-Probe
Uses an RP2040 and a small electrical circuit to measure the glass-to-glass latency and response of any image or light transmission pipeline.

## Circuit and hardawre

### BOM

The only part that's even remotely specific is the phototransistor.

|Part|Count|Notes|
|-|-|-|
|1MOhm resistor|1|Tighter tolerance is better, but not critical as this is part of the voltage divider for sensing|
|1kOhm resistor|1|Also not specific, just to limit current out of the GPIO to the BJT base. More will make the LED dimmer|
|S9018 BJT Transistor|1|Also not sensitive to details, but this powers the LED from the power rails, so needs to be able to handle 20-50mA|
|5mm white LED|1|At least 10k lumen rating, the higher the better|
|TEPT5700|1|Phototransistor. This is the one I had on hand, but you'll want one that is _at least_ as sensitive as this|
|RP2040 board|1|I used a knockoff RP2040 Zero baord from AliExpress, but any RP2040 board will probably do. If the board you're using doesn't tie the AGND to GND internally, you'll need to do that yourself.|

## Circuit

<Coming>

## Assembly

## Usage

- Flash micropython to the RP2040
- Load the code into the RP2040
- Sticky-tack the receiver to the goggles glass, don't disassemble the goggles.
- Plase the LED pointed close to and directly at the camera, tape it down if you need to.
- Turn everything on
- Run the script using your favourite RP2040/Arduino IDE/method (I use Thonny).
- It'll run for a few seconds, and flash the LED. When it's done, it'll print a report to stdout of wherever is running the script (like the Thonny console) so make sure you can receive that stdout from the process.
- 
