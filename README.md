# GiantGecko_I2CTest
Testing Ground for Using I2C on a EFM32 Giant Gecko Board by Silicon Labs.
This code will eventually evolve into the physical layer of Ex-Alta2's Network Stack and beyond, but not on this repo. 

The code has been modified from it's original form given on by the Silabs
Application Note:
https://www.silabs.com/support/resources.ct-application-notes.ct-example-code.p-microcontrollers_32-bit-mcus

See "AN0011: I2C Master and Slave Operation"

Specifically the GPIO pins and in setupI2C(void) have been changed to work with
the expansion header on a Giant Gecko, I2C Interface 1.

To configure, we used pins PC5 (I2C1_SCL) and PC4 (IC21_SDA)
https://www.silabs.com/documents/public/user-guides/efm32gg-stk3700-ug.pdf

Connected together between two boards. Both of these wires were pulled-up by
a 2.7kOhm resistor connected to the 3.3V pin of ONE of the boards.
An oscilloscope was used for initial testing purposes to see if the
clock and data lines were being pulled low. These pins were grounded to the
GND pins on the same board that provides 3.3V.

WHATEVER YOU DO, DO NOT PULL-UP THE PINS USING THE 5V LINE ON THE BOARD.
