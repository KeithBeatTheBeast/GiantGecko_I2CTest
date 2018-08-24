/*****************************************************************************
 * cspI2C_EFM32_M3.h
 *
 *  Created on: Jun 29th, 2018
 *  Author: kgmills
 *
 *  The code was originally based off of and is heavily modified from the Silicon Labs Application Note:
 *  https://www.silabs.com/support/resources.ct-application-notes.ct-example-code.p-microcontrollers_32-bit-mcus
 *
 *  See "AN0011: I2C Master and Slave Operation"
 *  Also see "AN0013: Direct Memory Access" as this I2C Physical Layer Driver is dependent on the EFM32's
 *  DMA controller for Tx/Rx operation - cspDMA_EFM32_M3 is a dependency
 *
 *  This is an I2C Driver for the EFM32 Giant Gecko using the ARM Cortex-M3 microprocessor.
 *  It requires the use of FreeRTOS (developed with V7.4.2) to work.
 *
 *  As it is meant to be one interface for the physical layer of a CubeSat Network Stack, only
 *  two modes of operation were implemented:
 *	-MASTER TRANSMITTER
 *	-SLAVE RECEIVER
 *
 *	The driver makes heavy use of the DMA controller within the EFM32 as well as a shared memory
 *	FreeRTOS protocol I developed when it is not integrated with a network stack e.g. CSP
 *	These non-Silabs dependency files are
 *	- cspDMA_EFM32_M3.c/h
 *	- SharedMemory.c/h
 *
 *	MIND THE TODO TAGS I HAVE PLACED THROUGHOUT THE CODE FOR COMMENTS
 *	ON SWITCHING BETWEEN HAVING THE DRIVER AS A STANDALONE AND INTEGRATING IT WITH
 *	CSP WHICH PROVIDES ACCESS TO BUFFERS.
 *
 *	Pull-up Resistors used:
 *	330 Ohms and 2.7 kOhms
 *
 *	MTU Constraints:
 *	Due to the nature of the DMA controller on the EFM32 with the Cortex-M3, the maximum transmission
 *	length for a frame being sent by this driver is 1024 bytes.
 *
 *	Performance: With the DMA integrated, this driver was tested and checked with a clock controlled
 *	variable of a 400kHz clock. Measuring the time it takes a transmission to complete on an oscilloscope,
 *	the time taken for a 20 byte (1 byte address + 19 byte frame) message is roughly 500 Microseconds.
 *  This entails a data rate of roughly 320kbit/s or 80% of maximum throughput.
 *
 *  A different driver can be developed from this one for the
 *	Giant Gecko with the Cortex-M4, with the double-buffered
 *	I2C Tx/Rx and LDMA. By my estimation, it's MTU will be roughly 4096 bytes.
 *
 *	To Accomplish this upgrade, you would need to do the following:
 *
 *	- When the DMA IRQ goes off for Rx, you need to modify the equation for
 *	calculating the address of the CTRL register
 *
 *	- Likewise, when taking the # of transmissions left from the CTRL register,
 *	the equation used will need to be modified and account for the fact that the
 *	n_minus_1 (or equivalent) field is 11 bits long on the M4, where it is only
 *	10 bits long on the M3
 *
 *	- To exceed an MTU of 2048 bytes, the DMA needs to be configured to
 *	send 2 bytes (16 bits) at a time rather than 1 byte (8 bits) as it does now.
 *
 *	- Therefore, you will need to figure out a scheme for sending only the right
 *	data and not +/- 1 byte of data. To do this, I propose:
 *	-- When the size of the packet is odd, before invoking the DMA, place
 *	the I2C Slave address and the first byte of packet data into the 16-bit Tx buffer.
 *	Then invoke the DMA, with an initial start address being the 2nd byte of data.
 *	-- When the size of the packet is even, load the address into the 16-bit buffer,
 *	leave 1/2 of the Tx buffer empty, then invoke the DMA on the base address of data.
 *
 *	CURRENT ISSUES:
 *	- The module doesn't like the transmit at its maximum MTU for long.
 *	If I give it a 1024byte frame it will transmit once or twice and then get locked up.
 *
 *	- Likewise, when the module locks itself up, even reports the error, it does not recover
 *	I cannot get it to properly reset and restart itself for additional transfers I have to reset
 *	the development board or re-flash it.
 *
 *	- As the EFM32GG Dev board with the M3 can only route I2C SCL/SDA to one location
 *	each, this driver only works when all "handle" input args are set to '1' and only
 *	through the I2C1 module.
 *
 ******************************************************************************/

#ifndef CSPI2C_EFM32_M3_H_
#define CSPI2C_EFM32_M3_H_

/*
 * C Libraries
 */
#include <stdbool.h>
#include <string.h>

/*
 * Chip Specific Includes
 * Oscillators, GPIO pins, DMA and I2C
 */
#include "em_i2c.h"
#include "em_cmu.h"
#include "em_gpio.h"

/*
 * FreeRTOS Includes
 */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

/*
 * CSP Errors
 */
#include "csp_error.h"
/*
 * Dependencies developed by me.
 */
#include "cspDMA_EFM32_M3.h"
#include "SharedMemory.h"

/* Defines*/
#define CSP_SADDR_MASK					0x7F // Slave Address Mask
#define I2C_WRITE					    0xFE // Should always be Master Trans/Slave Rec
#define STANDARD_CLK					100  // 100kbit/s clock
#define FAST_CLK						400	 // 400kbit/s clock
#define I2C_INT_PRIO_LEVEL				6    // Interrupt priority level
#define TX_INDEX_INIT					-1   // Tx index start value, is pre-incremented in code
#define TX_SEM_TO_MULTIPLIER			10   // Multiplied by portTICK_PERIOD_MS to determine timeout period.
#define CSP_I2C_HEADER_LEN				10   // The size of the CSP I2C Frame Header in bytes.
#define I2C_MTU							1014 // Hardcoded for the EFM32 with the M3 as the DMA can only do 1024 transfers per invocation.
#define NUM_SH_MEM_BUFS					3    // Number of shared memory buffers
#define I2C0_Ports						gpioPortC
#define I2C0_SDA						5 // I2C0 ports are a placeholder since the EFM32 has nothing for it
#define I2C0_SCL						4
#define I2C1_Ports						gpioPortC
#define I2C1_SCL						5
#define I2C1_SDA						4
#define I2C_ROUTE_LOC					0

/* Error codes for transmissionError field below */
#define NO_TRANS_ERR					0x00
#define NACK_ERR						0x01
#define BITO_ERR						0x02
#define CLTO_ERR						0x04
#define BUSERR_ERR						0x08
#define ARBLOST_ERR						0x10
#define TIMEOUT_ERR						0x20
#define E_QUEUE_ERR						0x40
#define F_QUEUE_ERR						0x80
#define ABORT_BUSHOLD					0x100

/*
 * ISR Interrupt Enable Lines
 * What conditions in the register
 * https://www.silabs.com/documents/public/reference-manuals/EFM32GG-RM.pdf p.g. 446
 * Will cause the interrupt to fire.
 */
#define i2c_IEN_flags (I2C_IEN_ADDR | \
					  I2C_IEN_SSTOP | \
					  I2C_IEN_RSTART | \
					  I2C_IEN_ARBLOST | \
					  I2C_IEN_NACK | \
					  I2C_IEN_MSTOP | \
					  I2C_IEN_BUSERR | \
					  I2C_IEN_BITO | \
					  I2C_IEN_BUSHOLD | \
					  I2C_IEN_CLTO)

/*
 * ISR Interrupt Flag Clear
 * When we do a mass clear of the Interrupt flags what goes
 */
#define i2c_IFC_flags (I2C_IFC_ADDR | \
					  I2C_IFC_SSTOP | \
					  I2C_IFC_RSTART | \
					  I2C_IFC_BUSHOLD | \
					  I2C_IFC_ARBLOST | \
					  I2C_IFC_NACK | \
					  I2C_IFC_ACK | \
					  I2C_IFC_MSTOP | \
					  I2C_IFC_TXC | \
					  I2C_IFC_BITO | \
					  I2C_IFC_CLTO | \
					  I2C_IFC_START | \
					  I2C_IFC_BUSERR)

/*
 * Initial values for the CTRL register of the I2C Module
 */
#define csp_I2C_ctrl (I2C_CTRL_SLAVE | \
					  I2C_CTRL_AUTOSN | \
					  I2C_CTRL_BITO_160PCC | \
					  I2C_CTRL_GIBITO | \
					  I2C_CTRL_CLTO_1024PPC)

typedef struct __attribute__((packed)) i2c_frame_s {
    uint8_t padding;
    uint8_t retries;
    uint32_t reserved;
    uint8_t dest;
    uint8_t len_rx;
    uint16_t len;
    uint8_t data[I2C_MTU];
} i2c_frame_t;


/*******************************************************************
 * Non-static (e.g. interact with external code) go here!
 ******************************************************************/
/********************************************************************
 * @brief Function called when DMA transfer is complete.
 * Execution depends on whether Tx or Rx completed.
 *
 * THIS FUNCTION IS CALLED IN AN ISR CONTEXT BY THE DMA'S IRQ
 *
 * For details on how to recover the number of bytes that has been received,
 * please visit the Silicon Labs Reference Manual for the
 * EFM32 Giant Gecko with the ARM Cortex-M3 processor.
 * https://www.silabs.com/documents/public/reference-manuals/EFM32GG-RM.pdf
 *
 * Chapter 8 - DMA; Pages 64-66 on the n_minus_1 data field
 *******************************************************************/
void i2cTransferComplete(unsigned int channel, bool primary, void *user);

/************************************************************
 * @brief Function CSP calls to send.
 * @param handle I2C Module to send from, on a dev board this can only be 1 and right now it doesn't do anything.
 * @param *frame Address to the CSP I2C Frame to be sent.
 * @param timeout Will not wait forever.
 * @return error code
 ***********************************************************/
int i2c_send(int handle, i2c_frame_t *frame, uint16_t timeout);

/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 * YOU MUST INITALIZE THE DMA USING cspDMA_Init(...) FIRST!
 *****************************************************************************/
int csp_i2c_init(uint8_t opt_addr, int handle, int speed);

/*****************************************************************************
 * @brief I2C Interrupt Handler.
 *        The interrupt table is in assembly startup file startup_efm32.s
 *****************************************************************************/
void I2C1_IRQHandler(void);

#endif /* CSPI2C_EFM32_M3_H_ */
