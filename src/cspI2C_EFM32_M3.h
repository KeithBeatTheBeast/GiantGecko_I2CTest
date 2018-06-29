/*
 * cspI2C_EFM32_M3.h
 *
 *  Created on: Jun 4, 2018
 *  Author: kgmills
 *  For use in the AlbertaSat Ex-Alta 2 Network Stack as a Physical Layer I2C Driver
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
 *
 *	MASTER TRANSMITTER/SLAVE RECEIVER
 *
 *	The driver makes heavy use of the DMA controller within the EFM32 as well as a shared memory
 *	FreeRTOS protocol I developed. These non-Silabs dependency files are
 *
 *	cspDMA_EFM32_M3.c/h
 *	SharedMemory.c/h
 *
 *	MTU Constraints:
 *	Due to the nature of the DMA controller on the EFM32 with the Cortex-M3, the maximum transmission
 *	length for a frame being sent by this driver is 1024 bytes.
 *
 *	Performance: With the DMA integrated, this driver was tested and checked with a clock controlled
 *	variable of 400kHz clock. Measuring the time it takes a transmission to complete on an oscilloscope,
 *	the time taken for a 20 byte (1 byte address + 19 byte frame) message is roughly 500 Microseconds.
 *  This entails a datarate of roughly 320kbit/s or 80% of maximum throughput.
 *
 *  A different driver can be (and will be) developed from this one for the
 *	Giant Gecko with the Cortex-M4, with the double-buffered
 *	I2C Tx/Rx and LDMA. By my estimation, it's MTU will be roughly 4096 bytes.
 *
 */

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
 * Dependencies developed by me.
 */
#include "cspDMA_EFM32_M3.h"
#include "SharedMemory.h"
#include "makePrintfWork.h" // TODO remove from release version

/* Defines*/
#define I2C_ADDRESS                     0xE2 // Slave Address of Device
#define I2C_TASKPRIORITY				2    // Task priority
#define I2C_WRITE					    0xFE // Should always be Master Trans/Slave Rec
#define I2C_READ					    0xFF // Future Potential Functionality
#define I2C_INT_PRIO_LEVEL				6    // Interrupt priority level
#define TX_INDEX_INIT					-1   // Tx index start value, is pre-incremented in code
#define TX_SEM_TO_MULTIPLIER			10   // Multiplied by portTICK_PERIOD_MS to determine timeout period.
#define MAX_FRAME_SIZE					1024 // Hardcoded for the EFM32 with the M3 as the DMA can only do 1024 transfers per invocation.
#define NUM_SH_MEM_BUFS					3    // Number of shared memory buffers

// Error Flag for Tx
static volatile uint16_t transmissionError;

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

// Rx Buffer - Constantly changes but needed for ISR.
uint8_t *i2c_Rx;

// Rx buffer index/first transmission flag.
static volatile bool i2c_RxInProgress, firstRx;

// FreeRTOS handles
static SemaphoreHandle_t busySem; // Tx semaphore
static QueueHandle_t 	 rxDataQueue, rxIndexQueue; // Rx Queue for data and index

// Shared memory handle
static SharedMem_t		 i2cSharedMem;

// The statically allocated shared memory
//uint8_t staticSharedMemBufs[NUM_SH_MEM_BUFS][MAX_FRAME_SIZE];

#endif /* CSPI2C_EFM32_M3_H_ */
