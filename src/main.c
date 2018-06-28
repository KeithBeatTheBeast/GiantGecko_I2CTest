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
 * controller for Tx and Rx usage.
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

// Buffers++
uint8_t tempTxBuf0[] = "let go of my gecko?";
uint8_t tempTxBuf1[] = "LET_GO_OF_MY_GECKO!";

/********************************************************************
 * @brief Inline function for calculating the address of the
 * CTRL register of the RX Channel's Descriptor
 */
static inline int16_t *getRxDMACtrlAddr() {
	return DMA->CTRLBASE + (DMA_CHANNEL_I2C_RX * CHANNEL_MULT_OFFSET) + CTRL_ADD_OFFSET;
}

/********************************************************************
 * @brief Function called when DMA transfer is complete.
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

	// RX has completed
	// Calculate the number of bytes that were received in a transmission
	// And send that number to the receiver task for processing to the next layer.
	// The I2C IRQ will send the data buffer.
	else if (channel == DMA_CHANNEL_I2C_RX) {

		/* VERY IMPORTANT THIS IS HOW YOU GET RX DATA SIZE!!!" */
		uint16_t count = MAX_FRAME_SIZE - ((*getRxDMACtrlAddr() & TRANS_REMAIN_MASK) >> TRANS_REMAIN_SHIFT) -1 ;

		// I literally put this here to prevent a size misalignment on the first transfer.
		if (firstRx) {
			firstRx = false;
			count--;
		}

		if (xQueueSendFromISR(rxIndexQueue, &count, NULL) != pdTRUE) {
			transmissionError |= F_QUEUE_ERR;
			puts("Index Queue Send Error");
		}
	}
}

/******************************************************************************
 * @brief	Setup DMA Controller
 *****************************************************************************/
void setupDMA() {

	/* Initialization Struct, and the Tx Structs */
	DMA_CfgChannel_TypeDef  txChannelConfig;
	DMA_CfgDescr_TypeDef	txDescriptorConfig;

	DMA_CfgChannel_TypeDef  rxChannelConfig;
	DMA_CfgDescr_TypeDef	rxDescriptorConfig;

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

	/* Setting up RX channel */
	rxChannelConfig.highPri    = true;
	rxChannelConfig.enableInt  = true;
	rxChannelConfig.select     = DMAREQ_I2C1_RXDATAV;
	rxChannelConfig.cb 		   = &dmaCB;
	DMA_CfgChannel(DMA_CHANNEL_I2C_RX, &rxChannelConfig);

	/* Setting up RX channel descriptor */
	rxDescriptorConfig.dstInc  = dmaDataInc1;
	rxDescriptorConfig.srcInc  = dmaDataIncNone;
	rxDescriptorConfig.size    = dmaDataSize1;
	rxDescriptorConfig.arbRate = dmaArbitrate1;
	rxDescriptorConfig.hprot   = 0;
	DMA_CfgDescr(DMA_CHANNEL_I2C_RX, true, &rxDescriptorConfig);

	/*
	* Changing the priority of DMA IRQ to use FreeRTOS functions.
	* It must be numerically equal to or greater than configMAX_SYSCALL_INTERRUPT_PRIORITY
	* defined in FreeRTOSConfig.h
	* Currently, that is set to 5.
	* I make the DMA have a higher priority than the I2C interrupt.
	* That, originally, is how it worked.
	*/
	NVIC_SetPriority(DMA_IRQn, I2C_INT_PRIO_LEVEL - 1);
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
	* The I2C priority level goes to 6, and the DMA 5.
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
		  	I2C_CTRL_BITO_160PCC | \
			I2C_CTRL_GIBITO | \
			I2C_CTRL_CLTO_1024PPC; // Removed autoack


	// Only accept transmissions if it is directly talking to me.
	I2C1->SADDRMASK |= 0x7F;

	// Set Rx index to zero.
	i2c_RxInProgress = false;

	// We do an operation on the first Rx.
	firstRx = true;

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

		uint8_t *txData;
		// TODO pend queue
		if (c++ % 2 == 0) {txData  = tempTxBuf0;} // TODO temp, remove.
		else {txData  = tempTxBuf1;}
		transmissionError = NO_TRANS_ERR; // Set error flag to zero.

		I2C1->CMD |= I2C_CMD_CLEARTX;

		// Load address. TODO format data from queue
		I2C1->TXDATA = I2C_ADDRESS & I2C_WRITE;

		DMA_ActivateBasic(DMA_CHANNEL_I2C_TX,
				true,
				false,
				(void*)&(I2C1->TXDATA),
				(void*)txData,
				20 - 1); // TODO get from upper layer.

		// Issue start condition
		I2C1->CMD |= I2C_CMD_START;

		// Pend the semaphore. This semaphore is initialized by main and given by the ISR
		// The reason why the semaphore is here is because the function
		// will eventually become a task where at the top, we pend a queue.
		if (xSemaphoreTake(busySem, portTICK_PERIOD_MS * TX_SEM_TO_MULTIPLIER) != pdTRUE) {
			I2C1->CMD = I2C_CMD_ABORT;
			transmissionError |= TIMEOUT_ERR;
		}

		// Error happened. TODO send to upper layer
		if (transmissionError > 0) {
			if (transmissionError > 1) {printf("Error: %x, IF: %x\n", transmissionError, I2C1->IF);}
			I2C1->CMD |= I2C_CMD_ABORT;
			vTaskDelay(portTICK_PERIOD_MS * 0.5);
		}
		//vTaskDelay(portTICK_PERIOD_MS);
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

	i2cSharedMem = xSharedMemoryCreate(sizeof(uint8_t) * MAX_FRAME_SIZE, NUM_SH_MEM_BUFS);
	if (i2cSharedMem == NULL) {puts("Creation of Shared Memory Failed!"); }
	else {puts("Creation of Shared Memory Successful");}

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
			I2C_IF_ADDR));
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

	  /*
	   * Slave Receiver Conditionals
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
	  if (flags & I2C_IF_ADDR ) {

		  I2C1->CMD = I2C_CMD_ACK;

		  // Get a chunk of memory. TODO for now we assume a block is available.
		  i2c_Rx = pSharedMemGetFromISR(i2cSharedMem, NULL);

		  if (i2c_Rx != pdFALSE) {
			  // Setup DMA Transfer
			  i2c_Rx[0] = I2C1->RXDATA;
			  I2C1->CTRL |= I2C_CTRL_AUTOACK;
			  i2c_RxInProgress = true;
			  DMA_ActivateBasic(DMA_CHANNEL_I2C_RX,
					  true,
					  false,
					  (void*)i2c_Rx + 1,
					  (void*)&(I2C1->RXDATA),
					  MAX_FRAME_SIZE - 1);
		  }

		  else {
			  I2C1->CMD = I2C_CMD_NACK | I2C_CMD_ABORT;
		  }
		  I2C_IntClear(I2C1, I2C_IFC_ADDR | I2C_IFC_BUSHOLD);
	  }

	  /*
	   * Master Transmitter:
	   * NACK Received.
	   * I2C_CTRL_AUTOSN flags ensures we cut transmission on a NACK but we need to report the error.
	   */
	  if (flags & I2C_IF_NACK) {
		  transmissionError |= NACK_ERR;
		  I2C_IntClear(I2C1, I2C_IFC_NACK);
		  I2C1->CMD |= I2C_CMD_ABORT;
		  xSemaphoreGiveFromISR(busySem, NULL);
	  }

	  // Double check for Bushold and if there is one, abort.
	  if (I2C1->IF & I2C_IF_BUSHOLD) {

		  // Data Transmitted and ACK received. DMA was not fast enough to trigger stop.
		  if (I2C1->STATE & I2C_STATE_MASTER) {
			  I2C1->CMD |= I2C_CMD_STOP;
		  }

		  else {
			  I2C1->CMD |= I2C_CMD_ABORT;
			  DMA->CHENS &= ~DMA_ENABLE_I2C_TX;
			  transmissionError |= ABORT_BUSHOLD;
			  xSemaphoreGiveFromISR(busySem, NULL);
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
	  I2C_IntClear(I2C1, i2c_IFC_flags); // TODO Is this clear a potential source of error?
	  flags &= ~I2C_IF_SSTOP;
	  I2C1->CTRL &= ~I2C_CTRL_AUTOSE;
	  xSemaphoreGiveFromISR(busySem, NULL);
  }

  /*
   * Stop Condition (or Repeated Start) detected
   * This flag is raised regardless
   * of whether or not the Master was speaking to us
   * CHECK and see if we were spoken to
   * By looking at a boolean set when the I2C_IF_ADDR flag was raised.
   */
  if (flags & (I2C_IF_SSTOP | I2C_IF_RSTART)) {
	  if (i2c_RxInProgress) {

		  // TODO something about AUTO ACKS
		  // Send Data to Rx task for processing. DMA IRQ handles calculating and
		  // posting the actual size of bytes RX'd to a separate queue.
		  if (xQueueSendFromISR(rxDataQueue, &i2c_Rx, NULL) != pdTRUE) {
			 transmissionError |= F_QUEUE_ERR;
			 puts("Data Queue Insert Error");
			 xSharedMemPut(i2cSharedMem, i2c_Rx);
		  }

		  // Tell the DMA to stop receiving bytes.
		  DMA->IFS = DMA_COMPLETE_I2C_RX;
		  i2c_RxInProgress = false;
	  }
	  I2C1->CTRL &= ~I2C_CTRL_AUTOACK;
      I2C_IntClear(I2C1, I2C_IFC_SSTOP | I2C_IFC_RSTART);
  }

  // Put ARBLOST, BITO, BUSERR, CLTO, here.
  // TODO this section is a source of error.
  // It worked without the DMA but now it causes both devices to simply spam start conditions
  // And never latch onto each other.
  if (flags & (I2C_IF_ARBLOST | I2C_IF_BUSERR | I2C_IF_CLTO | I2C_IF_BITO)) {
	  if (flags & I2C_IF_BITO) {
		  transmissionError |= BITO_ERR;
	  }
	  if (flags & I2C_IF_ARBLOST) {
		  transmissionError |= ARBLOST_ERR;
	  }
	  if (flags & I2C_IF_BUSERR) {
		  transmissionError |= BUSERR_ERR;
	  }
	  if (flags & I2C_IF_CLTO) {
		  transmissionError |= CLTO_ERR;
	  }

	  // Tell the DMA to stop.
	  if (i2c_RxInProgress) {
		  DMA->IFS = DMA_COMPLETE_I2C_RX;
	  }

	  else {
		  DMA->IFS = DMA_COMPLETE_I2C_TX;
	  }

	  I2C1->CTRL &= ~I2C_CTRL_AUTOACK;
	  I2C_IntClear(I2C1, I2C_IFC_ARBLOST | I2C_IFC_BUSERR | I2C_IFC_CLTO | I2C_IFC_BITO);
	  I2C1->CMD = I2C_CMD_ABORT;
	  i2c_RxInProgress = false;
	  xSharedMemPutFromISR(i2cSharedMem, i2c_Rx, NULL);
	  xSemaphoreGiveFromISR(busySem, NULL);
  }
}
