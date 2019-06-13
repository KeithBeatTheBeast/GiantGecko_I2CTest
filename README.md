# GiantGecko_I2CTest

_Work has ceased on this repo and is unlikely to be continued as the organization in question that would be using it has chosen opted to switch microcontroller families. Clone and use at your own risk._

cspI2C_EFM32_M3.h

Created on: Jun 29th, 2018
Author: kgmills

The code was originally based off of and is heavily modified from the Silicon Labs Application Note:
https://www.silabs.com/support/resources.ct-application-notes.ct-example-code.p-microcontrollers_32-bit-mcus

See "AN0011: I2C Master and Slave Operation"
Also see "AN0013: Direct Memory Access" as this I2C Physical Layer Driver is dependent on the EFM32's
DMA controller for Tx/Rx operation - cspDMA_EFM32_M3 is a dependency

This is an I2C Driver for the EFM32 Giant Gecko using the ARM Cortex-M3 microprocessor.
It requires the use of FreeRTOS (developed with V7.4.2) to work.

As it is meant to be one interface for the physical layer of a CubeSat Network Stack, only
two modes of operation were implemented:
- MASTER TRANSMITTER
-SLAVE RECEIVER

The driver makes heavy use of the DMA controller within the EFM32 as well as a shared memory
FreeRTOS protocol I developed when it is not integrated with a network stack e.g. CSP
These non-Silabs dependency files are
- cspDMA_EFM32_M3.c/h
- SharedMemory.c/h

MIND THE TODO TAGS I HAVE PLACED THROUGHOUT THE CODE FOR COMMENTS
ON SWITCHING BETWEEN HAVING THE DRIVER AS A STANDALONE AND INTEGRATING IT WITH
CSP WHICH PROVIDES ACCESS TO BUFFERS.

Pull-up Resistors used:
330[Ohms] for 1Mbit/s testing and 2.7[kOhms] otherwise
A minimum pull-up resistor impedance of ~172[Ohms] is required at all speeds.

MTU Constraints:
Due to the nature of the DMA controller on the EFM32 with the Cortex-M3, the maximum transmission
length for a frame being sent by this driver is 1024 bytes.

Performance: With the DMA integrated, this driver was tested and checked with a clock controlled
variable of a 400kHz clock. Measuring the time it takes a transmission to complete on an oscilloscope,
the time taken for a 20 byte (1 byte address + 19 byte frame) message is roughly 500[uS].
This entails a data rate of roughly 320kbit/s or 80% of maximum potential throughput.

A different driver can be developed from this one for the
Giant Gecko with the Cortex-M4, with the double-buffered
I2C Tx/Rx and LDMA. By my estimation, it's MTU will be roughly 4096 bytes.
 
To Accomplish this upgrade, you would need to do the following:

- Modify cspDMA_EFM32.c/h to work with the LDMA on the M4.

- When the DMA IRQ goes off for Rx, you need to modify the equation for
calculating the address of the CTRL register because the hardware is different

- Likewise, when taking the # of transmissions left from the CTRL register,
the equation used will need to be modified and account for the fact that the
n_minus_1 (or equivalent) field is 11 bits long on the M4, where it is only
10 bits long on the M3 (this doubles the MTU from 1024 to 2048)

- To exceed an MTU of 2048 bytes, the DMA needs to be configured to
send 2 bytes (16 bits) at a time rather than 1 byte (8 bits) as it does now.

- Therefore, you will need to figure out a scheme for sending only the right
data and not +/- 1 byte of data. To do this, I propose:
-- When the size of the packet is odd, before invoking the DMA, place
the I2C Slave address and the first byte of packet data into the 16-bit Tx buffer.
Then invoke the DMA, with an initial start address being the 2nd byte of data.
-- When the size of the packet is even, load the address into the 16-bit buffer,
leave 1/2 of the Tx buffer empty, then invoke the DMA on the base address of data.

CURRENT ISSUES:
- The module when running with randomly encounter CLTO (SCL Low Timeout) interrupts and
locks itself up. Despite my best efforts to code it to cancel the current transmission and
reset itself to run again, I cannot get it to recover from this lockup.
If I disable CLTO interrupts, an ARBLOST (Arbitration Lost) interrupt is eventually thrown
and the same problem occurs.
When this happens I am forced to reset/re-flash the board.

- As the EFM32GG Dev board with the M3 can only route I2C SCL/SDA to one location
each, this driver only works when all "handle" input args are set to '1' and only
through the I2C1 module.
