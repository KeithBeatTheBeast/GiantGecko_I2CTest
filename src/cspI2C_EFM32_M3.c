/*****************************************************************************
 * cspI2C_EFM32_M3.h
 *
 *  Created on: Jun 29th, 2018
 *  Author: kgmills
 *
 *  Please see the header of the paired cspI2C_EFM32_M3.h file for details
 *
 ******************************************************************************/

#include "cspI2C_EFM32_M3.h"

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
		I2CRegs->CTRL |= I2C_CTRL_AUTOSE;
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

/**************************************************************************//**
 * @brief  Setup I2C
 *****************************************************************************/
void cspI2C_Init() {

	printf("%x\n", I2C1);
	I2CRegs = I2C1;
	printf("%x\n", I2CRegs);
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
	I2CRegs->ROUTE = I2C_ROUTE_SDAPEN |
			I2C_ROUTE_SCLPEN |
			(0 << _I2C_ROUTE_LOCATION_SHIFT);

	/* Initializing the I2C */
	I2C_Init(I2CRegs, &i2cInit);

	/* Setting up to enable slave mode */
	I2CRegs->SADDR = I2C_ADDRESS;
	I2CRegs->CTRL |= I2C_CTRL_SLAVE | \
			I2C_CTRL_AUTOSN | \
		  	I2C_CTRL_BITO_160PCC | \
			I2C_CTRL_GIBITO | \
			I2C_CTRL_CLTO_1024PPC;


	// Only accept transmissions if it is directly talking to me.
	I2CRegs->SADDRMASK |= 0x7F;

	// Set Rx index to zero.
	i2c_RxInProgress = false;

	// We do an operation on the first Rx.
	firstRx = true;

	// Enable interrupts
	I2C_IntClear(I2CRegs, i2c_IFC_flags);
	I2C_IntEnable(I2CRegs, i2c_IEN_flags);
	NVIC_EnableIRQ(I2C1_IRQn);

	/* We now setup the DMA components for I2C */

	/* Initialization Struct, and the Tx Structs */
	DMA_CfgChannel_TypeDef  txChannelConfig;
	DMA_CfgDescr_TypeDef	txDescriptorConfig;

	DMA_CfgChannel_TypeDef  rxChannelConfig;
	DMA_CfgDescr_TypeDef	rxDescriptorConfig;

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

	// We're starting/restarting the board, so it assume the bus is busy
	// We need to either have a clock-high (BITO) timeout, or issue an abort
	if (I2CRegs->STATE & I2C_STATE_BUSY) {
		I2CRegs->CMD = I2C_CMD_ABORT;
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

		I2CRegs->CMD |= I2C_CMD_CLEARTX;

		// Load address. TODO format data from queue
		I2CRegs->TXDATA = I2C_ADDRESS & I2C_WRITE;

		DMA_ActivateBasic(DMA_CHANNEL_I2C_TX,
				true,
				false,
				(void*)&(I2CRegs->TXDATA),
				(void*)txData,
				20 - 1); // TODO get from upper layer.

		// Issue start condition
		I2CRegs->CMD |= I2C_CMD_START;

		// Pend the semaphore. This semaphore is initialized by main and given by the ISR
		// The reason why the semaphore is here is because the function
		// will eventually become a task where at the top, we pend a queue.
		if (xSemaphoreTake(busySem, portTICK_PERIOD_MS * TX_SEM_TO_MULTIPLIER) != pdTRUE) {
			I2CRegs->CMD = I2C_CMD_ABORT;
			transmissionError |= TIMEOUT_ERR;
		}

		// Error happened. TODO send to upper layer
		if (transmissionError > 0) {
			if (transmissionError > 1) {printf("Error: %x, IF: %x\n", transmissionError, I2CRegs->IF);}
			I2CRegs->CMD |= I2C_CMD_ABORT;
			vTaskDelay(portTICK_PERIOD_MS * 0.5);
		}
	}
}

static void vI2CReceiveTask(void *handle) {

	uint8_t *newRxBuf, *cspBuf;
	int16_t index;

	while(1) {
		xQueueReceive(rxDataQueue, &newRxBuf, portMAX_DELAY);
		xQueueReceive(rxIndexQueue, &index, portMAX_DELAY);

		cspBuf = pvPortMalloc(sizeof(uint8_t) * index);
		memcpy(cspBuf, newRxBuf, index);

		xSharedMemPut(i2cSharedMem, newRxBuf);

		printf("DATA SIZE: %d; DATA: %s\n", index, cspBuf); // TODO send to upper layer
		vPortFree(cspBuf); // TODO this will be the responsibility of the upper layer to get rid of.
	}
}

/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
void i2cTempmain(void) {

	// Create the semaphore and report on it.
	busySem = xSemaphoreCreateBinary(); // TODO replace puts with error statements to initializer
	if (busySem == NULL) { puts("Creation of Busy Semaphore Failed!"); }
	else { puts("Creation of Busy Semaphore Successful!");}

	// Create the rx queue and report on it
	rxDataQueue = xQueueCreate(NUM_SH_MEM_BUFS, sizeof(uint8_t *));
	if (rxDataQueue == NULL) { puts("Creation of Rx Data Queue Failed!"); } // TODO replace with error statements to init
	else { puts("Creation of Rx Data Queue Successful");}

	rxIndexQueue = xQueueCreate(NUM_SH_MEM_BUFS, sizeof(int16_t));
	if (rxIndexQueue == NULL) { puts("Creation of Rx Index Queue Failed!"); } // TODO replace with error statements to init
	else { puts("Creation of Rx Index Queue Successful");}

	i2cSharedMem = xSharedMemoryCreate(sizeof(uint8_t) * MAX_FRAME_SIZE, NUM_SH_MEM_BUFS);
	//i2cSharedMem = xSharedMemoryCreateStatic(staticSharedMemBufs, NUM_SH_MEM_BUFS); TODO fix ASAP
	if (i2cSharedMem == NULL) {puts("Creation of Shared Memory Failed!"); }
	else {puts("Creation of Shared Memory Successful");}

	/* Setting up DMA Controller */
	cspDMA_Init(CSP_HPROT);

	/* Setting up i2c */
	cspI2C_Init();

	// Create I2C Tasks
	xTaskCreate(vI2CTransferTask, (const char *) "I2CRegs_Tx", configMINIMAL_STACK_SIZE, NULL, I2C_TASKPRIORITY, NULL);
	xTaskCreate(vI2CReceiveTask, (const char *) "I2CRegs_Rx", configMINIMAL_STACK_SIZE, NULL, I2C_TASKPRIORITY, NULL);

	// Start Scheduler TODO externalize to another API
	vTaskStartScheduler();
}

/*****************************************************************************
 * @brief I2C Interrupt Handler.
 *        The interrupt table is in assembly startup file startup_efm32.s
 *****************************************************************************/
void I2C1_IRQHandler() {
   
  int flags = I2CRegs->IF;

  /*
   * Conditions that may, but are not guaranteed to, cause a BUSHOLD condition
   * Usually normal operating conditions but take too long
   * Tx/Rx transfer stuff
   */
  if (flags & (I2C_IF_BUSHOLD | I2C_IF_NACK | I2C_IF_ADDR)) {

	  /*
	   * Slave Receiver:
	   * Start Condition on the line has been detected
	   * Automatic Address Matching has determined a Master is trying to talk to this device
	   */
	  if (flags & I2C_IF_ADDR ) {

		  // Ack the address or else it will be interpreted as a NACK.
		  I2CRegs->CMD = I2C_CMD_ACK;

		  // Pend the shared memory for a buffer pointer.
		  i2c_Rx = pSharedMemGetFromISR(i2cSharedMem, NULL);

		  // The buffer pend returned true, so we can accept the transmission.
		  // Get the first byte, set up for AUTO-ACKs so the DMA handles all incoming
		  // And activate the DMA. Done.
		  if (i2c_Rx != pdFALSE) {
			  // Setup DMA Transfer
			  i2c_Rx[0] = I2CRegs->RXDATA;
			  I2CRegs->CTRL |= I2C_CTRL_AUTOACK;
			  i2c_RxInProgress = true;
			  DMA_ActivateBasic(DMA_CHANNEL_I2C_RX,
					  true,
					  false,
					  (void*)i2c_Rx + 1,
					  (void*)&(I2CRegs->RXDATA),
					  MAX_FRAME_SIZE - 1);
		  }

		  // No pointers are availible so we cannot accept the data.
		  // Send a NACK and abort the transmission.
		  else {
			  I2CRegs->CMD = I2C_CMD_NACK | I2C_CMD_ABORT;
			  transmissionError |= E_QUEUE_ERR;
		  }

		  // In all cases the Interrupt flag must be cleared.
		  I2C_IntClear(I2CRegs, I2C_IFC_ADDR | I2C_IFC_BUSHOLD);
	  }

	  /*
	   * Master Transmitter:
	   * NACK Received.
	   * I2C_CTRL_AUTOSN flags ensures we cut transmission on a NACK but we need to report the error.
	   */
	  if (flags & I2C_IF_NACK) {
		  transmissionError |= NACK_ERR;
		  I2C_IntClear(I2CRegs, I2C_IFC_NACK);
		  I2CRegs->CMD |= I2C_CMD_ABORT;
		  xSemaphoreGiveFromISR(busySem, NULL);
	  }

	  /*
	   * Sometimes we encounter BUSHOLD
	   * Most of the time it was because the DMA IRQ was not fast enough
	   * to tell the I2C module to sent it's stop condition when TXBL and TXC are fully
	   * emptied so the bus locks up.
	   */
	  if (I2CRegs->IF & I2C_IF_BUSHOLD) {

		  // Data Transmitted and ACK received. DMA was not fast enough to trigger stop.
		  if (I2CRegs->STATE & I2C_STATE_MASTER) {
			  I2CRegs->CMD |= I2C_CMD_STOP;
		  }

		  else {
			  I2CRegs->CMD |= I2C_CMD_ABORT;
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
	  I2C_IntClear(I2CRegs, i2c_IFC_flags);
	  flags &= ~I2C_IF_SSTOP;
	  I2CRegs->CTRL &= ~I2C_CTRL_AUTOSE;
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

		  // Send Data to Rx task for processing. DMA IRQ handles calculating and
		  // posting the actual size of bytes RX'd to a separate queue.
		  if (xQueueSendFromISR(rxDataQueue, &i2c_Rx, NULL) != pdTRUE) {
			 transmissionError |= F_QUEUE_ERR;
			 xSharedMemPut(i2cSharedMem, i2c_Rx);
		  }

		  // Tell the DMA to stop receiving bytes.
		  DMA->IFS = DMA_COMPLETE_I2C_RX;
		  i2c_RxInProgress = false;
	  }
	  I2CRegs->CTRL &= ~I2C_CTRL_AUTOACK;
      I2C_IntClear(I2CRegs, I2C_IFC_SSTOP | I2C_IFC_RSTART);
  }

  /*
   * All Potential Error Codes/Timeouts
   * Put ARBLOST, BITO, BUSERR, CLTO, here.
   * Raise error flag for Tx task.
   * Cancel Auto-acks, abort transmission, return from ISR.
   * */
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

	  I2CRegs->CTRL &= ~I2C_CTRL_AUTOACK;
	  I2C_IntClear(I2CRegs, I2C_IFC_ARBLOST | I2C_IFC_BUSERR | I2C_IFC_CLTO | I2C_IFC_BITO);
	  I2CRegs->CMD = I2C_CMD_ABORT;
	  i2c_RxInProgress = false;
	  xSharedMemPutFromISR(i2cSharedMem, i2c_Rx, NULL);
	  xSemaphoreGiveFromISR(busySem, NULL);
  }
}