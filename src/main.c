/*****************************************************************************
 * @file i2c_master_slave.c
 * @brief I2C Demo Application
 * @author Silicon Labs
 * @version 1.06
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 *
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/

/*******************************************************************************
 * Initially edited by Kgmills on 5/11/18
 *
 * Initial Code for using the I2C Interface for an Silabs EFM32 Giant Gecko board for
 * testing purposes. Just sends messages back and forth, capitalizing some characters
 * Simple, but will be used to develop a functional physical layer for our project.
 *
 * The code has been modified from it's original form given on by the Silabs
 * Application Note:
 * https://www.silabs.com/support/resources.ct-application-notes.ct-example-code.p-microcontrollers_32-bit-mcus
 *
 * See "AN0011: I2C Master and Slave Operation"
 *
 * Specifically the GPIO pins and in setupI2C(void) have been changed to work with
 * the expansion header on a Giant Gecko, I2C Interface 1.
 *
 * To configure, we used pins PC5 (I2C1_SCL) and PC4 (IC21_SDA)
 * https://www.silabs.com/documents/public/user-guides/efm32gg-stk3700-ug.pdf
 *
 * Connected together between two boards. Both of these wires were pulled-up by
 * a 2.7kOhm resistor connected to the 3.3V pin of ONE of the boards.
 *
 * 6/1/18: The 2.7kOhm resistors have been exchanged for a pair of 330 Ohm Resistors
 * This enables the proper usage and testing with Fast+ Mode (1Mbit/s)
 * An oscilloscope was used for initial testing purposes to see if the
 * clock and data lines were being pulled low. These pins were grounded to the
 * GND pins on the same board that provides 3.3V.
 *
 **************************************************************************
 * WHATEVER YOU DO, DO NOT PULL-UP THE PINS USING THE 5V LINE ON THE BOARD.
 **************************************************************************
 *
 ******************************************************************************/

// Main Header file - This file will eventually be renamed cspI2C.c
#include "cspI2C_EFM32.h"

// Boolean I use for debugging to determine whether or not I want a stream of printfs.
#define printfEnable					false // TODO remove from final version

// Buffers++
int16_t i2c_rxBufferIndex;

/**************************************************************************//**
 * @brief  Setup I2C
 *****************************************************************************/
void setupI2C(void) {

	/*
	* Changing the priority of I2C1 IRQ.
	* It must be numerically equal to or greater than configMAX_SYSCALL_INTERRUPT_PRIORITY
	* defined in FreeRTOSConfig.h
	* Currently, that is set to 5.
	*/
	NVIC_SetPriority(I2C1_IRQn, I2C_INT_PRIO_LEVEL);

	// Using default settings
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

	/* Using PC4 (SDA) and PC5 (SCL) */
	GPIO_PinModeSet(gpioPortC, 4, gpioModeWiredAndPullUpFilter, 1);
	GPIO_PinModeSet(gpioPortC, 5, gpioModeWiredAndPullUpFilter, 1);

	/* Enable pins at location 1 */
	I2C1->ROUTE = I2C_ROUTE_SDAPEN |
			I2C_ROUTE_SCLPEN |
			(0 << _I2C_ROUTE_LOCATION_SHIFT);

	/* Initializing the I2C */
	I2C_Init(I2C1, &i2cInit);

	/* Setting up to enable slave mode */
	I2C1->SADDR = I2C_ADDRESS;
	I2C1->CTRL |= I2C_CTRL_SLAVE | \
			I2C_CTRL_AUTOACK | \
			I2C_CTRL_AUTOSN | \
		  	I2C_CTRL_BITO_160PCC | \
		  	I2C_CTRL_GIBITO;

	// Set Rx index to zero.
	i2c_rxBufferIndex = RX_INDEX_INIT;

	// Enable interrupts
	I2C_IntClear(I2C1, i2c_IFC_flags);
	I2C_IntEnable(I2C1, i2c_IEN_flags);
	NVIC_EnableIRQ(I2C1_IRQn);

	// We're starting/restarting the board, so it assume the bus is busy
	// We need to either have a clock-high (BITO) timeout, or issue an abort
	if (I2C1->STATE & I2C_STATE_BUSY) {
		I2C1->CMD = I2C_CMD_ABORT;
	}
}

/*
 * @brief Reset I2C as best as we can.
 */
void resetI2C() {

	I2C1->CMD |= I2C_CMD_CLEARTX;
	I2C1->CMD |= I2C_CMD_START | I2C_CMD_STOP;
	I2C1->CMD |= I2C_CMD_CLEARPC;
	I2C1->CMD |= I2C_CMD_ABORT;

	vTaskDelay(portTICK_PERIOD_MS * 1);

	I2C_IntClear(I2C1, i2c_IFC_flags);
	I2C_IntEnable(I2C1, i2c_IEN_flags);
	NVIC_EnableIRQ(I2C1_IRQn);
}

/**************************************************************************//**
 * @brief  Transmitting I2C data. Will busy-wait until the transfer is complete.
 *****************************************************************************/
static void vI2CTransferTask(void *txQueueHandle) { // TODO pass in queue handle and semaphore handle
	while (1) {

		// TODO pend queue
		/* Initializing I2C transfer */
		i2c_Tx.addr    = I2C_ADDRESS;           // TODO get address from LUT
		i2c_Tx.rwBit   = I2C_WRITE;        // TODO hardcode to write.
		i2c_Tx.txData  = (uint8_t*)&"let go of my gecko!"; // TODO data from queue
		i2c_Tx.len     = sizeof(i2c_Tx.txData);
		i2c_Tx.txIndex = TX_INDEX_INIT;                    // Reset index to -1 always

		// Load address. TODO format data from queue
		I2C1->TXDATA = i2c_Tx.addr & i2c_Tx.rwBit;

		// Issue start condition
		I2C1->CMD |= I2C_CMD_START;

		// Pend the semaphore. This semaphore is initialized by main and given by the ISR
		// The reason why the semaphore is here is because the function
		// will eventually become a task where at the top, we pend a queue.
		if (xSemaphoreTake(busySem, portTICK_PERIOD_MS * TX_SEM_TO_MULTIPLIER) == pdTRUE) {
			if (printfEnable) {puts("Semaphore Taken");}
		}

		else {
			if (printfEnable) {puts("Semaphore Timeout");} // TODO send error to upper layer
			resetI2C();
		}

		// TODO replace with semaphore when Brendan helps with NVIC priorities
		vTaskDelay(portTICK_PERIOD_MS * TX_DELAY_MULTIPLIER);
		if (printfEnable) {puts("Tx Looping");}
	}
}

static void vI2CReceiveTask(void *rxQueueHandle) {

	char *buf;
	while(1) {
		xQueueReceive(rxQueue, buf, portMAX_DELAY);
		printf("%s\n", buf); // TODO send to upper layer
	}
}

/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void) {

	/* Enabling clock to the I2C*/
	CMU_ClockEnable(cmuClock_I2C1, true);
  
	// Use this to enable printfs.
	SWO_SetupForPrint();

	// Create the semaphore and report on it.
	busySem = xSemaphoreCreateBinary(); // TODO replace puts with error statements to initalizer
	if (busySem == NULL) { puts("Creation of Busy Semaphore Failed!"); }
	else { puts("Creation of Busy Semaphore Successful!");}

	// Create the rx queue and report on it
	rxQueue = xQueueCreate(10, sizeof(char *));
	if (rxQueue == NULL) { puts("Creation of Rx Queue Failed!"); } // TODO replace with error statements to init
	else { puts("Creation of Rx Queue Successful");}

	/* Setting up i2c */
	setupI2C();

	// Create I2C Tasks
	xTaskCreate(vI2CTransferTask, (const char *) "I2C1_Tx", configMINIMAL_STACK_SIZE + 10, NULL, I2C_TaskPriority, NULL);
	xTaskCreate(vI2CReceiveTask, (const char *) "I2C1_Rx", configMINIMAL_STACK_SIZE, NULL, I2C_TaskPriority, NULL);

	// Start Scheduler TODO externalize to another API
	vTaskStartScheduler();

	// Should never get here
	return 0;
}

/*
 * @brief If there is more data to add to the Tx buffer, that data is added.
 * Function for adding new byte to TX buffer in the case of TXC and TXBL IF conditions
 * First it increments the index of the tx buffer.
 * If the pointer is equal to the size of the buffer, we've reached the end.
 * Send TRUE, in which case the IRQ calling this function will send a STOP condition
 * If the pointer is less than the size of the buffer, we've still got more data
 * Add the next byte to the buffer, return false.
 */
static inline bool addNewByteToTxBuffer() {

	if (++i2c_Tx.txIndex > i2c_Tx.len + ADD_BYTE_LIMIT) {
		return true;
	}

	else if (I2C1->IF & I2C_IF_TXBL) {
		I2C1->TXDATA = i2c_Tx.txData[i2c_Tx.txIndex];
		I2C_IntClear(I2C1, I2C_IFC_TXC);
		if (printfEnable) {printf("Char at location %d: %c\n", i2c_Tx.txIndex, i2c_Tx.txData[i2c_Tx.txIndex]);}
	}
	return false;
}

/*
 * @brief Checks conditions that the BUSHOLD if-else looks for
 * These are the states in the I2C State register that will enter there.
 */
static inline bool checkBusHoldStates(int state){
	return ((state & MASTER_TRANS_ADDR_ACK) | \
			(state & MASTER_TRANS_ADDR_NACK) | \
			(state & MASTER_TRANS_DATA_ACK) | \
			(state & MASTER_TRANS_DATA_NACK) | \
			(state & SLAVE_RECIV_ADDR_ACK) | \
			(state & SLAVE_RECIV_DATA_ACK));
}

/**************************************************************************//**
 * @brief I2C Interrupt Handler.
 *        The interrupt table is in assembly startup file startup_efm32.s
 *****************************************************************************/
void I2C1_IRQHandler(void) {
   
  int flags = I2C1->IF;
  int state = I2C1->STATE;

  /*
   * BITO - Bus Idle Timeout
   * Goes off when SCL has been high for a period specified in I2C_CTRL
   * Assumes bus is idle, and master operations can begin.
   * Reset Rx index
   */
  if (flags & I2C_IF_BITO) {
	  i2c_rxBufferIndex = RX_INDEX_INIT;
	  I2C_IntClear(I2C1, i2c_IFC_flags);
	  xSemaphoreGiveFromISR(busySem, NULL);
	  if (printfEnable) {puts("BITO Timeout");}
  }

  /*
   * Arbitration lost
   */
//  else if (flags & (I2C_IF_BUSERR | I2C_IF_ARBLOST)){
//	  i2c_rxBufferIndex = RX_INDEX_INIT;
//	  NVIC_DisableIRQ(I2C1_IRQn);
//	  I2C_IntDisable(I2C1, i2c_IEN_flags);
//	  I2C_IntClear(I2C1, i2c_IFC_flags);
//	  return;
//  }
  /*
   * Master has transmitted stop condition
   * Transmission is officially over
   * Report success to upper layer
   * Release semaphore
   */
  else if (flags & I2C_IF_MSTOP) {
	  I2C_IntClear(I2C1, i2c_IFC_flags);
	  xSemaphoreGiveFromISR(busySem, NULL); //TODO remove comment
	  if (printfEnable) {puts("Master Stop Detected");}
  }

  /*
   * Conditions that may, but are not guaranteed to, cause a BUSHOLD condition
   * Usually normal operating conditions but take too long
   * Tx/Rx transfer stuff
   */
  else if (checkBusHoldStates(state) || (flags & I2C_IF_BUSHOLD)) {

	  if (printfEnable & (flags & I2C_IF_BUSHOLD)) { puts("BUSHOLD"); printf("State: %x\n", state); }

	  /*
	   * Master Transmitter BUSHOLD Conditionals
	   * Table 16.5, Page 426-427 of the EFM32GG Reference Manual
	   * This part of the sub-conditionals handles states
	   * 0x97, 0x9F, 0xD7 and 0xDF
	   * State 0x57 (Start Sent, but no Address in TX) is NOT
	   * handled here - because the address is loaded into the TX buffer
	   * In the performI2C function before the start condition is requested.
	   */

	  /*
	   * Master Transmitter:
	   * ADDR+W (0x9F) or
	   * DATA (0xDF)
	   * has been sent and a NACK has been received
	   * TODO completed version should cut contact (stop) and send error message to upper layer
	   * I2C_CTRL_AUTOSN sends STOP, we'll need to implement message to upper layer
	   */
	  if (state == MASTER_TRANS_ADDR_NACK || state == MASTER_TRANS_DATA_NACK) {
		  xSemaphoreGiveFromISR(busySem, NULL); //TODO Send error to upper layer
		  I2C_IntClear(I2C1, i2c_IFC_flags);
	  }

	  /*
	   * Master Transmitter:
	   * ADDR+W (0x97) or
	   * DATA (0xD7)
	   * has been sent and an ACK has been received
	   * Since it's an ACK, we send another byte, unless at the end of message
	   * Then we'll send a stop condition
	   */
	  else if (state == MASTER_TRANS_DATA_ACK || state == MASTER_TRANS_ADDR_ACK) {

		  if (addNewByteToTxBuffer()) { // If the function returns true, we're done transmitting
			  I2C1->CMD |= I2C_CMD_STOP; // So send a stop
			  if (printfEnable) {puts("ACK Received after data, stop issued"); }
		  }

		  else {
			  if (printfEnable) {puts("ACK Received after data, no stop issued");}
		  }
		  I2C_IntClear(I2C1, I2C_IFC_ACK);
	  }

	  /*
	   * Slave Receiver BUSHOLD Conditionals
	   * See Table 16.10, page 433 of the EFM32GG Reference Manual
	   * All States that would trigger a BUSHOLD are accounted for.
	   */

	  /*
	   * Slave Receiver:
	   * Start Condition on the line has been detected
	   * Automatic Address Matching has determined a Master is trying to talk to this device
	   * 0x71 is the state
	   * Basically the same code from Silicon Labs
	   * Read the Rx buffer
	   */
	  else if (state == SLAVE_RECIV_ADDR_ACK) {
	      I2C1->RXDATA;

	      I2C_IntClear(I2C1, I2C_IFC_ADDR);
	      if (printfEnable) {puts("Address match non-repeat start");}
	  }

	  /*
	   * Slave Receiver:
	   * The Master has sent data (0xB1)
	   * Load it into the Rx buffer and increment pointer
	   * RXDATA IF is cleared when the buffer is read.
	   */
	  else if (state == SLAVE_RECIV_DATA_ACK) {
	      /* Data received */
	      rxData[i2c_rxBufferIndex++] = I2C1->RXDATA;
	      if (printfEnable) {puts("Data received");}
	  }

	  if (flags & I2C_IF_BUSHOLD) {
		  I2C_IntClear(I2C1, I2C_IFC_BUSHOLD);
	  }
  }

  /*
   * Address Match but not catching on the BUSHOLD subconditional
   * Read from RXDATAV, if necessary, and clear flag.
   */
  else if (flags & I2C_IF_ADDR) {
      I2C1->RXDATA;
      I2C_IntClear(I2C1, I2C_IFC_ADDR);
      if (printfEnable) {puts("Address match outside of BUSHOLD subconditionals");}
  }

  /*
   * Stop condition detected - this flag is raised regardless
   * of whether or not the Master was speaking to us
   * CHECK and see if we were spoken to
   * By looking at the index of the Rx buffer
   */
  else if (flags & I2C_IF_SSTOP) {
	  if (printfEnable) {puts("Stop condition detected");}
	  if (i2c_rxBufferIndex != RX_INDEX_INIT) {
		  if (xQueueSendFromISR(rxQueue, rxData, NULL) == errQUEUE_FULL) {
			 puts("Error, Queue is full");
		  }
		  i2c_rxBufferIndex = RX_INDEX_INIT;
	  }
      I2C_IntClear(I2C1, i2c_IFC_flags);
  }

  /*
   * Repeated Start - This is also a valid way to end transmission
   * Do the same as SSTOP
   */
  else if (flags & I2C_IF_RSTART) {
	  if (printfEnable) {puts("Repeated condition detected");}
	  if (i2c_rxBufferIndex != 0) {
		  if (xQueueSendFromISR(rxQueue, rxData, NULL) == errQUEUE_FULL) {
			 puts("Error, Queue is full");
		  }
		  i2c_rxBufferIndex = RX_INDEX_INIT;
	  }
      I2C_IntClear(I2C1, I2C_IFC_RSTART);
  }

  // Put BUSERR, ARBLOST, CLTO, here.
  // Wait for timeout on semaphore
  else {
	  i2c_rxBufferIndex = RX_INDEX_INIT;
	  NVIC_DisableIRQ(I2C1_IRQn);
	  I2C_IntDisable(I2C1, i2c_IEN_flags);
	  I2C_IntClear(I2C1, i2c_IFC_flags);
  }
}
