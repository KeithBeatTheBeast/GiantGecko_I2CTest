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
 ******************************************************************************
 *
 * 6/1/18: The 2.7kOhm resistors have been exchanged for a pair of 330 Ohm Resistors
 * This enables the proper usage and testing with Fast+ Mode (1Mbit/s)
 * An oscilloscope was used for initial testing purposes to see if the
 * clock and data lines were being pulled low. These pins were grounded to the
 * GND pins on the same board that provides 3.3V.
 *
 ******************************************************************************
 *
 * 6/12/18: The scope of this branch has changed to now utilizing the DMA
 * controller for Tx and possibly Rx usage for the I2C module.
 * Most of the work is based on the Silicon Labs Application Note for using the
 * DMA controller, AN0013
 * https://www.silabs.com/documents/public/application-notes/AN0013.pdf
 * Specifically the code for using the DMA with I2C as a master tx.
 *
 ******************************************************************************/

// Main Header file - This file will eventually be renamed cspI2C.c
#include "cspI2C_EFM32.h"

#include "testSharedMem.h"
#include "EFM32_throwI2CErrors.h"

// Boolean I use for debugging to determine whether or not I want a stream of printfs.
#define printfEnable					false // TODO remove from final version

// Buffers++
uint8_t tempTxBuf0[] = "let go of my gecko?";
uint8_t tempTxBuf1[] = "LET_GO_OF_MY_GECKO!";
uint8_t tempRxBuf[20];

/********************************************************************
 * @brief Function called when DMA transfer is complete.
 * Enables interrupts when doing a DMA read.
 *******************************************************************/
void i2cTransferComplete(unsigned int channel, bool primary, void *user) {

	/* Ignore unused parameters */
	(void) primary;
	(void) user;

	// If we're on the right channel, and the DMA has dumped all data to the TXDATA
	// buffer, then allow the I2C module to send a STOP once it is out of data.
	// This setting is cleared in the MSTOP condition of the I2C IRQ.
	if (channel == DMA_CHANNEL_I2C_TX) {
		I2C1->CTRL |= I2C_CTRL_AUTOSE;
	}
}

/******************************************************************************
 * @brief	Setup DMA Controller
 *****************************************************************************/
void setupDMA() {

	/* Initialization Struct, and the Tx Structs */
	DMA_CfgChannel_TypeDef  txChannelConfig;
	DMA_CfgDescr_TypeDef	txDescriptorConfig;

	cspDMA_Init(CSP_HPROT);

	/* Setup call-back function */
	dmaCB.cbFunc  = i2cTransferComplete;
	dmaCB.userPtr = NULL;

	/* Setting up TX channel */
	txChannelConfig.highPri   = true;
	txChannelConfig.enableInt = true;
	txChannelConfig.select    = DMAREQ_I2C1_TXBL;
	txChannelConfig.cb        = &dmaCB;
	DMA_CfgChannel(DMA_CHANNEL_I2C_TX, &txChannelConfig);

	/* Setting up TX channel descriptor */
	txDescriptorConfig.dstInc  = dmaDataIncNone;
	txDescriptorConfig.srcInc  = dmaDataInc1;
	txDescriptorConfig.size    = dmaDataSize1;
	txDescriptorConfig.arbRate = dmaArbitrate1;
	txDescriptorConfig.hprot   = 0;
	DMA_CfgDescr(DMA_CHANNEL_I2C_TX, true, &txDescriptorConfig);
}

/**************************************************************************//**
 * @brief  Setup I2C
 *****************************************************************************/
void setupI2C() {

	/* Enabling clock to the I2C*/
	CMU_ClockEnable(cmuClock_I2C1, true);

	/*
	* Changing the priority of I2C1 IRQ.
	* It must be numerically equal to or greater than configMAX_SYSCALL_INTERRUPT_PRIORITY
	* defined in FreeRTOSConfig.h
	* Currently, that is set to 5.
	*/
	NVIC_SetPriority(I2C1_IRQn, I2C_INT_PRIO_LEVEL);

	I2C_Init_TypeDef i2cInit = { \
			true, \
			true, \
			0, \
			I2C_FREQ_FAST_MAX, \
			i2cClockHLRAsymetric \
		};

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
			I2C_CTRL_AUTOSN | \
			I2C_CTRL_AUTOACK | \
		  	I2C_CTRL_BITO_160PCC | \
			I2C_CTRL_CLTO_1024PPC | \
		  	I2C_CTRL_GIBITO;

	// Removed: I2C_CTRL_AUTOACK |
	I2C1->SADDRMASK |= 0x7F;

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

/**************************************************************************//**
 * @brief  Transmitting I2C data. Will busy-wait until the transfer is complete.
 *****************************************************************************/
static void vI2CTransferTask(void *txQueueHandle) { // TODO pass in queue handle and semaphore handle

	uint32_t c = 0;
	while (1) {

		// TODO pend queue
		/* Initializing I2C transfer */
		i2c_Tx.addr    = I2C_ADDRESS;           // TODO get address from LUT
		i2c_Tx.rwBit   = I2C_WRITE;				// TODO hardcode to write.
		if (c++ % 2 == 0) {i2c_Tx.txData  = tempTxBuf0;} // TODO temp, remove.
		else {i2c_Tx.txData  = tempTxBuf1;}
		i2c_Tx.len     = 20;         			// TODO need to somehow get size of memory
		i2c_Tx.txIndex = TX_INDEX_INIT;         // Reset index to -1 always
		i2c_Tx.transmissionError = NO_TRANS_ERR;// Set error flag to zero.

		// Load address. TODO format data from queue
		I2C1->TXDATA = i2c_Tx.addr & i2c_Tx.rwBit;

		DMA_ActivateBasic(DMA_CHANNEL_I2C_TX,
				true,
				false,
				(void*)&(I2C1->TXDATA),
				(void*)i2c_Tx.txData,
				i2c_Tx.len - 1);

		// Issue start condition
		I2C1->CMD |= I2C_CMD_START;
		//I2C_IntEnable(I2C1, I2C_IF_TXBL);

		// Pend the semaphore. This semaphore is initialized by main and given by the ISR
		// The reason why the semaphore is here is because the function
		// will eventually become a task where at the top, we pend a queue.
		if (xSemaphoreTake(busySem, portTICK_PERIOD_MS * TX_SEM_TO_MULTIPLIER) != pdTRUE) {
			I2C1->CMD = I2C_CMD_ABORT;
			i2c_Tx.transmissionError |= TIMEOUT_ERR;
		}

		// Error happened. TODO send to upper layer
		if (i2c_Tx.transmissionError > 0) {
			if (i2c_Tx.transmissionError > 1) {printf("Error: %x, IF: %x\n", i2c_Tx.transmissionError, I2C1->IF);}
			vTaskDelay(portTICK_PERIOD_MS * 0.5);
		}
		vTaskDelay(portTICK_PERIOD_MS * 0.25);
	}
}

static void vI2CReceiveTask(void *rxQueueHandle) {

	uint8_t *data;
	int16_t index;

	while(1) {
		xQueueReceive(rxDataQueue, &data, portMAX_DELAY);
		xQueueReceive(rxIndexQueue, &index, portMAX_DELAY);
		printf("DATA SIZE: %d; DATA: %s\n", index, data); // TODO send to upper layer
		xSharedMemPut(i2cSharedMem, data);
	}
}

/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void) {
  
	// Use this to enable printfs.
	SWO_SetupForPrint();
	// Tests TODO more tests
//	runAllSharedMemTests();

	// Create the semaphore and report on it.
	busySem = xSemaphoreCreateBinary(); // TODO replace puts with error statements to initializer
	if (busySem == NULL) { puts("Creation of Busy Semaphore Failed!"); }
	else { puts("Creation of Busy Semaphore Successful!");}

	// Create the rx queue and report on it
	rxDataQueue = xQueueCreate(20, sizeof(uint8_t *));
	if (rxDataQueue == NULL) { puts("Creation of Rx Data Queue Failed!"); } // TODO replace with error statements to init
	else { puts("Creation of Rx Data Queue Successful");}

	rxIndexQueue = xQueueCreate(20, sizeof(int16_t));
	if (rxIndexQueue == NULL) { puts("Creation of Rx Index Queue Failed!"); } // TODO replace with error statements to init
	else { puts("Creation of Rx Index Queue Successful");}

	//i2cSharedMem = xSharedMemoryCreate(sizeof(uint8_t) * MAX_FRAME_SIZE, 10);
	//if (i2cSharedMem == NULL) {puts("Creation of Shared Memory Failed!"); }
	//else {puts("Creation of Shared Memory Successful");}

	/* Setting up DMA Controller */
	setupDMA();

	/* Setting up i2c */
	setupI2C();

	// Create I2C Tasks
	xTaskCreate(vI2CTransferTask, (const char *) "I2C1_Tx", configMINIMAL_STACK_SIZE, NULL, I2C_TASKPRIORITY, NULL);
	xTaskCreate(vI2CReceiveTask, (const char *) "I2C1_Rx", configMINIMAL_STACK_SIZE, NULL, I2C_TASKPRIORITY, NULL);
	//xTaskCreate(vThrowI2CErrors, (const char *) "Throw Exceptions", configMINIMAL_STACK_SIZE, NULL, I2C_TASKPRIORITY, NULL);

	// Start Scheduler TODO externalize to another API
	vTaskStartScheduler();

	// Should never get here
	return 0;
}

static inline bool checkFlags(int flag) {
	return (flag & (I2C_IF_BUSHOLD | \
			I2C_IF_NACK | \
			I2C_IF_ADDR | \
			I2C_IF_RXDATAV));
}

/**************************************************************************//**
 * @brief I2C Interrupt Handler.
 *        The interrupt table is in assembly startup file startup_efm32.s
 *****************************************************************************/
void I2C1_IRQHandler(void) {
   
  int flags = I2C1->IF;

  /*
   * Conditions that may, but are not guaranteed to, cause a BUSHOLD condition
   * Usually normal operating conditions but take too long
   * Tx/Rx transfer stuff
   */
  if (checkFlags(flags)) {

	  if (printfEnable & (flags & I2C_IF_BUSHOLD)) { puts("BUSHOLD");}

	  /*
	   * Master Transmitter:
	   * ADDR+W (0x9F) or
	   * DATA (0xDF)
	   * has been sent and a NACK has been received
	   * Do not allow TXBL to trigger the ISR, report "something bad happened".
	   */
	  if (flags & I2C_IF_NACK) {
		  i2c_Tx.transmissionError |= NACK_ERR;
		  I2C_IntClear(I2C1, i2c_IFC_flags);
		  xSemaphoreGiveFromISR(busySem, NULL);
	  }

	  /*
	   * Slave Receiver BUSHOLD Conditionals
	   * See Table 16.10, page 433 of the EFM32GG Reference Manual
	   * All States that would trigger a BUSHOLD are accounted for.
	   *
	   * Slave Receiver:
	   * Start Condition on the line has been detected
	   * Automatic Address Matching has determined a Master is trying to talk to this device
	   * 0x71 is the state
	   * Basically the same code from Silicon Labs
	   * Read the Rx buffer
	   */
	  else if (flags & I2C_IF_ADDR ) {
	      I2C1->RXDATA;

	 //     i2c_Rx = pSharedMemGetFromISR(i2cSharedMem, NULL);

	      // We were unable to allocate memory from the shared memory system
	      // Report error to task, issue an abort and a NACK
	   //   if (i2c_Rx == pdFALSE) {
	    //	  i2c_Tx.transmissionError |= E_QUEUE_ERR;
	    //	  I2C1->CMD |= I2C_CMD_ABORT | I2C_CMD_NACK;
	     // }

	      // We WERE able to retrieve a memory pointer from the shared system
	      // Send an ACK to the master, and enable auto-acks on the remaining.
	     // else {
	  //    I2C1->CMD |= I2C_CMD_ACK;
	   //   I2C1->CTRL |= I2C_CTRL_AUTOACK;
	    //  }

		  I2C_IntClear(I2C1, I2C_IFC_ADDR);
	      //if (printfEnable) {puts("Address match non-repeat start");}
	  }

	  /*
	   * Slave Receiver:
	   * The Master has sent data (0xB1)
	   * Load it into the Rx buffer and increment pointer
	   * RXDATA IF is cleared when the buffer is read.
	   */
	  else if (flags & I2C_IF_RXDATAV) {
		  //i2c_Rx[i2c_rxBufferIndex++] = I2C1->RXDATA;
		  tempRxBuf[i2c_rxBufferIndex++] = I2C1->RXDATA;
	      if (printfEnable) {puts("Data received");}
	  }

	  if (flags & I2C_IF_BUSHOLD) {
	  	I2C_IntClear(I2C1, I2C_IFC_BUSHOLD);

	  	// Double check for Bushold and if there is one, abort.
	  	if (I2C1->IF & I2C_IF_BUSHOLD) {
	  		I2C1->CMD |= I2C_CMD_ABORT;
	  		DMA->CHENS &= ~DMA_ENABLE_I2C_TX;
	  		i2c_Tx.transmissionError |= ABORT_BUSHOLD;
	  		// xSemaphoreGiveFromISR(busySem, NULL);
	  	}
	  }
  }

  /*
   * Master has transmitted stop condition
   * Transmission is officially over
   * Report success to upper layer
   * Release semaphore
   */
  if (flags & I2C_IF_MSTOP) {
	  I2C_IntClear(I2C1, i2c_IFC_flags);
	  flags &= ~I2C_IF_SSTOP;
	  I2C1->CTRL &= ~I2C_CTRL_AUTOSE;
	  xSemaphoreGiveFromISR(busySem, NULL);
	  if (printfEnable) {puts("Master Stop Detected");}
  }

  /*
   * Stop condition detected - this flag is raised regardless
   * of whether or not the Master was speaking to us
   * CHECK and see if we were spoken to
   * By looking at the index of the Rx buffer
   * Also repeated start since it is a valid way to end a transmission
   */
  if (flags & (I2C_IF_SSTOP | I2C_IF_RSTART)) {
	  if (i2c_rxBufferIndex != RX_INDEX_INIT) {

		  // We're done the transmission, so disable auto-acks.
		//  I2C1->CTRL &= ~I2C_CTRL_AUTOACK;
//		  if (xQueueSendFromISR(rxDataQueue, &i2c_Rx, NULL) == errQUEUE_FULL) {
//			 i2c_Tx.transmissionError |= F_QUEUE_ERR;
//			 xSharedMemPut(i2cSharedMem, i2c_Rx);
//		  }
//
//		  if (xQueueSendFromISR(rxIndexQueue, &i2c_rxBufferIndex, NULL) == errQUEUE_FULL) {
//			  i2c_Tx.transmissionError |= F_QUEUE_ERR;
//		  }

		 // printf("%s\n", tempRxBuf);
		  i2c_rxBufferIndex = RX_INDEX_INIT;
	  }
      I2C_IntClear(I2C1, I2C_IFC_SSTOP | I2C_IFC_RSTART);
  }

  // Put BUSERR, ARBLOST, CLTO, here.
  if (flags & (I2C_IF_ARBLOST | I2C_IF_BUSERR | I2C_IF_CLTO | I2C_IF_BITO)) {
	  if (flags & I2C_IF_BITO) {
		  i2c_Tx.transmissionError |= BITO_ERR;
	  }
	  if (flags & I2C_IF_ARBLOST) {
		  i2c_Tx.transmissionError |= ARBLOST_ERR;
	  }
	  if (flags & I2C_IF_BUSERR) {
		  i2c_Tx.transmissionError |= BUSERR_ERR;
	  }
	  if (flags & I2C_IF_CLTO) {
		  i2c_Tx.transmissionError |= CLTO_ERR;
	  }

	  I2C1->CTRL &= ~I2C_CTRL_AUTOACK;
	  I2C_IntClear(I2C1, I2C_IFC_ARBLOST | I2C_IFC_BUSERR | I2C_IFC_CLTO | I2C_IFC_BITO);
	  I2C1->CMD = I2C_CMD_ABORT;
	  i2c_rxBufferIndex = RX_INDEX_INIT;
	  //xSharedMemPutFromISR(i2cSharedMem, i2c_Rx, NULL);
	  xSemaphoreGiveFromISR(busySem, NULL);
  }
}
