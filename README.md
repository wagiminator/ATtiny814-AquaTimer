# AquaTimer - Programmable Timer for Aquariums based on ATtiny414/814/1614
AquaTimer is a programmable timer for 12V devices such as lighting, solenoid valves or pumps not only for aquariums. It has three switchable channels for currents up to 2A each and up to 5A in total. Connected lighting can be dimmed if desired and slowly faded in and out to simulate sunrises and sunsets. The internal RTC of the ATtiny is used as a clockwork in conjunction with a 32.768kHz crystal. A backup battery keeps the clock running even if the external power supply is interrupted. Settings are made using three buttons and the OLED display.

- Design Files (EasyEDA): https://easyeda.com/wagiminator/attiny814-aquacontroller

![pic1.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny814-AquaTimer/main/documentation/AquaTimer_pic1.jpg)

# Compiling and Uploading the Firmware
## If using the Arduino IDE
- Open your Arduino IDE.
- Make sure you have installed [megaTinyCore](https://github.com/SpenceKonde/megaTinyCore).
- Go to **Tools -> Board -> megaTinyCore** and select **ATtiny1614/1604/814/804/414/404/214/204**.
- Go to **Tools** and choose the following board options:
  - **Chip:**           ATtiny1614 or ATtiny814 or ATtiny414
  - **Clock:**          5 MHz internal
  - Leave the rest at the default settings.
- Connect your programmer to your PC and to the UPDI header on the board.
- Go to **Tools -> Programmer** and select your UPDI programmer.
- Go to **Tools -> Burn Bootloader** to burn the fuses.
- Open the sketch and click **Upload**.

## If using the makefile (Linux/Mac)
- Connect your [programmer](https://github.com/wagiminator/AVR-Programmer) (jtag2updi or SerialUPDI) to your PC and to the UPDI header on the board.
- Make sure you have installed the latest [avr-gcc toolchain](http://maxembedded.com/2015/06/setting-up-avr-gcc-toolchain-on-linux-and-mac-os-x/).
- Open a terminal.
- Navigate to the folder with the makefile and the sketch.
- Run `DEVICE=attiny814 PROGRMR=serialupdi PORT=/dev/ttyUSB0 make install` to compile, burn the fuses and upload the firmware (change DEVICE, PROGRMR and PORT accordingly).

The device time is automatically set to the current time (compilation time) when the firmware is uploaded. Install the CR1220, CR1225 or LIR1220 (recommended) buffer battery before disconnecting the device.

# Operating Instructions
1. Connect the devices to be controlled to the AquaTimer using the screw terminals. Pay attention to the correct polarity!
2. Connect the AquaTimer to a 12V power supply via the DC barrel connector.
3. Press the "SET" button to get to the main menu. Adjust the values according to your wishes.

# References, Links and Notes
1. [ATtiny814 Datasheet](https://ww1.microchip.com/downloads/en/DeviceDoc/ATtiny417-814-816-817-DataSheet-DS40002288A.pdf)

![pic2.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny814-AquaTimer/main/documentation/AquaTimer_pic2.jpg)
![pic3.png](https://raw.githubusercontent.com/wagiminator/ATtiny814-AquaTimer/main/documentation/AquaTimer_pic3.png)
![pic4.jpg](https://raw.githubusercontent.com/wagiminator/ATtiny814-AquaTimer/main/documentation/AquaTimer_pic4.jpg)

# License
![license.png](https://i.creativecommons.org/l/by-sa/3.0/88x31.png)

This work is licensed under Creative Commons Attribution-ShareAlike 3.0 Unported License. 
(http://creativecommons.org/licenses/by-sa/3.0/)
