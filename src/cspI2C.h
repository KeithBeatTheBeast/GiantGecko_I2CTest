/*
 * cspI2C.h
 *
 *  Created on: Jun 4, 2018
 *  Author: kgmills
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

#ifndef CSPI2C_H_
#define CSPI2C_H_

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

// Files I use to make printf(...) work on the EFM32 Giant Gecko using Simplicity Studio.
#include "makePrintfWork.h" // TODO remove from final version

/* Defines*/
#define I2C_ADDRESS                     0xE2 // TODO memory address table and functions.
#define I2C_RXBUFFER_SIZE               256
#define I2C_TaskPriority				2

// Structure for transfers - a re-vamped version of the I2C_TransferSeq_TypeDef
typedef struct {
  uint8_t addr;    // Address, see https://www.i2c-bus.org/addressing/ WE ONLY USE 7-BIT ADDRESSSING
  uint8_t rwBit;   // Read (0) or write (1)
  uint8_t *data;   // Pointer to the data.
  uint8_t len;     // In the code it should usually be sizeof(data) and at least 1
  int16_t txIndex; // Tx Array Index
} cspI2CTransfer_t;

#endif /* CSPI2C_H_ */
