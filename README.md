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

```netlist
* Netlist created with www.circuit-diagram.org
* 
V1 0 0
V2 1 0
D3 2 3
'phototransnpn2 emit:4 col:2 t=NPNPhototransistor2
'transnpn col:0 base:3 emit:5 t=NPN
R4 1 4 1000000
R5 5 6 2000
'microcontroller #19:4 #25:1 #34:0 #9:2 #18:6 header=Raspberry Pi Pico sz=40 p0=GP0 p1=VBUS p2=GP1 p3=VSYS p4=GND p5=GND p6=GP2 p7=3V3_EN p8=GP3 p9=3V3_OUT p10=GP4 p11=ADC_VREF p12=GP5 p13=GP28 p14=GND p15=GND p16=GP6 p17=GP27 p18=GP7 p19=GP26 p20=GP8 p21=RUN p22=GP9 p23=GP22 p24=GND p25=GND p26=GP10 p27=GP21 p28=GP11 p29=GP20 p30=GP12 p31=GP19 p32=GP13 p33=GP18 p34=GND p35=GND p36=GP14 p37=GP17 p38=GP15 p39=GP16
```

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
