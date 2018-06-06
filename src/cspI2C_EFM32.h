/*
 * cspI2C.h
 *
 *  Created on: Jun 4, 2018
 *  Author: kgmills
 *  For use in the AlbertaSat Ex-Alta 2 Network Stack
 *
 *  Purpose of this file:
 *  To store the location of constants, register locations, etc
 *  For the I2C Driver for an EFM32 Giant Gecko's I2C Driver.
 *
 *  The code has been modified from it's original form given on by the Silabs
 *  Application Note:
 *  https://www.silabs.com/support/resources.ct-application-notes.ct-example-code.p-microcontrollers_32-bit-mcus
 *
 *  See "AN0011: I2C Master and Slave Operation"
 *
 *  This file is a storage location for the register locations/data structure
 *  stored in em_i2c.h so that they can be modified as needed for devices that may have the same architecure
 *  but different register locations.
 *
 *  It also includes my own files.
 *
 */

#ifndef CSPI2C_EFM32_H_
#define CSPI2C_EFM32_H_

// Includes for starting the chip, i2c, oscillators and gpio pins
#include <stdbool.h>
#include "em_i2c.h" // TODO intent of this file is to remove this dependency.
#include "em_cmu.h"
#include "em_gpio.h"

// FreeRTOS includes
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "semphr.h"
#include "task.h"
#include "queue.h"

#include "SharedMemory.h"

// Files I use to make printf(...) work on the EFM32 Giant Gecko using Simplicity Studio.
#include "makePrintfWork.h" // TODO remove from final version

/* Defines*/
#define I2C_ADDRESS                     0xE2 // TODO memory address table and functions.
#define I2C_RXBUF_LEN                   256  // Buffer size TODO variable and can change.
#define I2C_TASKPRIORITY				2    // TODO adjust as needed
#define I2C_WRITE					    0xFE // Should always be Master Trans/Slave Rec
#define I2C_READ					    0xFF // Future Potential Functionality
#define I2C_INT_PRIO_LEVEL				5    // Interrupt priority level
#define TX_INDEX_INIT					-1   // Tx index start value, is pre-incremented in code
#define TX_SEM_TO_MULTIPLIER			20   // Multiplied by portTICK_PERIOD_MS to determine timeout period.
#define TX_DELAY_MULTIPLIER				1    // Multiplied by portTICK_PERIOD_MS to determine wait time after transfer complete
#define RX_INDEX_INIT					0	 // Rx Index start value/reset value
#define ADD_BYTE_LIMIT					-1   // Used to determine whether or not we're at the end of Tx
#define MAX_FRAME_SIZE					256  // Adjustable, assume worst case scenario

/* State Register Values Checked in ISR */
#define MASTER_TRANS_ADDR_ACK			0x97 // Master Sent ADDR+W, ACK Received
#define MASTER_TRANS_ADDR_NACK			0x9F // Master Sent ADDR+W, NACK Received
#define MASTER_TRANS_DATA_ACK			0xD7 // Master Sent DATA, ACK Received
#define MASTER_TRANS_DATA_NACK			0xDF // Master Sent DATA, NACK Received
#define SLAVE_RECIV_ADDR_ACK			0x71 // Slave has received ADDR+W
#define SLAVE_RECIV_DATA_ACK			0xB1 // Slave has received DATA

/*
 * ISR Interrupt Enable Lines
 * What conditions in the register
 * https://www.silabs.com/documents/public/reference-manuals/EFM32GG-RM.pdf p.g. 446
 * Will cause the interrupt to fire.
 */
#define i2c_IEN_flags (I2C_IEN_ADDR | \
					  I2C_IEN_RXDATAV | \
					  I2C_IEN_SSTOP | \
					  I2C_IEN_BUSHOLD | \
					  I2C_IEN_RSTART | \
					  I2C_IEN_ARBLOST | \
					  I2C_IEN_NACK | \
					  I2C_IEN_ACK | \
					  I2C_IEN_MSTOP | \
					  I2C_IEN_CLTO | \
					  I2C_IEN_BUSERR | \
					  I2C_IEN_BITO)

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
 * @brief Structure for Tx
 * One exists in global memory for the driver, as a static volatile.
 */
typedef struct {
  uint8_t addr;    // Address, see https://www.i2c-bus.org/addressing/ WE ONLY USE 7-BIT ADDRESSSING
  uint8_t rwBit;   // Read (0) or write (1)
  uint8_t *txData; // Pointer to the data.
  uint8_t len;     // In the code it should usually be sizeof(data) and at least 1
  int16_t txIndex; // Tx Array Index
} cspI2CTransfer_t;

/* Transmission Structure */
volatile cspI2CTransfer_t i2c_Tx;

// Pointer
uint8_t *i2c_Rx;

// Rx buffer index
static int16_t i2c_rxBufferIndex;

// FreeRTOS handles
static SemaphoreHandle_t busySem;
static QueueHandle_t 	 rxQueue;

static SharedMem_t		 i2cSharedMem;

#endif /* CSPI2C_EFM32_H_ */
