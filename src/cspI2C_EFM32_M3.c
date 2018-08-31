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
 *	330[Ohms] for 1Mbit/s testing and 2.7[kOhms] otherwise
 *	A minimum pull-up resistor impedance of ~172[Ohms] is required at all speeds.
 *
 *	MTU Constraints:
 *	Due to the nature of the DMA controller on the EFM32 with the Cortex-M3, the maximum transmission
 *	length for a frame being sent by this driver is 1024 bytes.
 *
 *	Performance: With the DMA integrated, this driver was tested and checked with a clock controlled
 *	variable of a 400kHz clock. Measuring the time it takes a transmission to complete on an oscilloscope,
 *	the time taken for a 20 byte (1 byte address + 19 byte frame) message is roughly 500[uS].
 *  This entails a data rate of roughly 320kbit/s or 80% of maximum potential throughput.
 *
 *  A different driver can be developed from this one for the
 *	Giant Gecko with the Cortex-M4, with the double-buffered
 *	I2C Tx/Rx and LDMA. By my estimation, it's MTU will be roughly 4096 bytes.
 *
 *	To Accomplish this upgrade, you would need to do the following:
 *
 *	- Modify cspDMA_EFM32.c/h to work with the LDMA on the M4.
 *
 *	- When the DMA IRQ goes off for Rx, you need to modify the equation for
 *	calculating the address of the CTRL register because the hardware is different
 *
 *	- Likewise, when taking the # of transmissions left from the CTRL register,
 *	the equation used will need to be modified and account for the fact that the
 *	n_minus_1 (or equivalent) field is 11 bits long on the M4, where it is only
 *	10 bits long on the M3 (this doubles the MTU from 1024 to 2048)
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
 *	- The module when running with randomly encounter CLTO (SCL Low Timeout) interrupts and
 *	locks itself up. Despite my best efforts to code it to cancel the current transmission and
 *	reset itself to run again, I cannot get it to recover from this lockup.
 *	If I disable CLTO interrupts, an ARBLOST (Arbitration Lost) interrupt is eventually thrown
 *	and the same problem occurs.
 *	When this happens I am forced to reset/re-flash the board.
 *
 *	- As the EFM32GG Dev board with the M3 can only route I2C SCL/SDA to one location
 *	each, this driver only works when all "handle" input args are set to '1' and only
 *	through the I2C1 module.
 *
 ******************************************************************************/

#include "cspI2C_EFM32_M3.h"

/***********
 * GLOBALS
 **********/
// Error Flag for Tx
static volatile uint16_t transmissionError;

// Pointer to structure where the I2C Registers are found. Assigned at runtime depending on module
static I2C_TypeDef *I2CRegs;

// Rx Buffer - Constantly changes but needed for ISR.
static uint8_t *i2c_Rx;

// Rx buffer index/first transmission flag.
static volatile bool i2c_RxInProgress, firstRx;

// FreeRTOS handles
static SemaphoreHandle_t busySem, waitSem; // Tx semaphores

// Shared memory handle
static SharedMem_t		 i2cSharedMem;

/********************************************************************
 * @brief Inline function for calculating the address of the
 * CTRL register of the RX Channel's Descriptor
 *
 * MATH: ADDRESS = (Channel Number * Size of a channel) + Offset to CTRL Register.
 *
 * THIS WILL NOT WORK FOR THE CORTEX-M4'S DMA.
 *******************************************************************/
static inline int16_t *getRxDMACtrlAddr() {
	return DMA->CTRLBASE + (DMA_CHANNEL_I2C_RX * CHANNEL_MULT_OFFSET) + CTRL_ADD_OFFSET;
}

/********************************************************************
 * @brief Function called when DMA transfer is complete.
 * Only executes when Tx is done. Rx is handled in I2C ISR
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

	// I2C Transmission Channel, and the DMA has dumped all data to the TXDATA
	// buffer, then allow the I2C module to send a STOP once it is out of data.
	// This setting is cleared in the MSTOP condition of the I2C IRQ.
	if (channel == DMA_CHANNEL_I2C_TX) {
		I2CRegs->CTRL |= I2C_CTRL_AUTOSE;
	}
}

/************************************************************
 * @brief Function CSP calls to send.
 * @param handle I2C Module to send from, on a dev board this can only be 1 and right now it doesn't do anything.
 * @param *frame Address to the CSP I2C Frame to be sent.
 * @param timeout Will not wait forever.
 * @return error code
 ***********************************************************/
int i2c_send(int handle, i2c_frame_t *frame, uint16_t timeout) {

	// Wait for the driver to be free or timeout.
	if (xSemaphoreTake(waitSem, timeout) != pdTRUE) {
		return CSP_ERR_TIMEDOUT;
	}

	for (uint8_t i = 0; i < frame->retries; i++) {

		transmissionError = NO_TRANS_ERR; // Set error flag to zero. This can be modified by the ISR and used for debugging purposes.
		I2CRegs->CMD |= I2C_CMD_CLEARTX;  // Clear the Tx Buffer and Tx Shift Register

		I2CRegs->TXDATA = frame->dest & I2C_WRITE; // Load slave address with WRITE BIT.

		// Invoke the DMA for the transfer. It will load a new byte into
		// the Tx register when the Tx ISR flag goes up.
		// I assume the header is not bad.
		DMA_ActivateBasic(DMA_CHANNEL_I2C_TX, // DMA Channel
				true,						  // Primary Channel
				false,						  // Do not use burst
				&(I2CRegs->TXDATA),           // Destination Address
				frame,						  // Source Address
				CSP_I2C_HEADER_LEN + frame->len - 1); // Length of Transfer in Bytes

		while(i2c_RxInProgress) { // TODO This may be the issue, a race condition between Tx and Rx.
			vTaskDelay(100);
		}

		I2C_IntDisable(I2CRegs, I2C_IEN_CLTO | I2C_IEN_BITO); // Disable timeouts - the task semaphore wait will act as one.
		I2C_IntClear(I2CRegs, I2C_IFC_CLTO | I2C_IEN_BITO); // Clear those ISR flags if they existed.
		I2CRegs->CMD |= I2C_CMD_START; // Issue the start condition

		// Wait for the transfer to complete according to the timeout.
		if (xSemaphoreTake(busySem, timeout) != pdTRUE) {
			transmissionError |= TIMEOUT_ERR;
		}

		// A driver-layer error has occurred. It may have been a timeout, or arbitration was lost, etc
		if (transmissionError != NO_TRANS_ERR) {
			// Right now in all cases, an abort command is issued, and we wait.
			printf("0x%x\n", transmissionError);
			if (transmissionError & NACK_ERR) {
				I2CRegs->CMD |= I2C_CMD_ABORT;
				vTaskDelay(portTICK_PERIOD_MS);
			}

			else { // TODO this is placed for debugging purposes.
				// Instead of giving back the CSP error code, the driver-level error code is returned.
				// Remove this layer of conditionals in the final version and have it always abort and
				// vTaskDelay to calm down.
				xSemaphoreGive(waitSem);
				return transmissionError;
			}

		}

		// Transmission was successful.
		// Release the module, return no error.
		else {
			xSemaphoreGive(waitSem);
			return CSP_ERR_NONE;
		}
	}

	// If we reached this point, we ran out of attempts to send the frame.
	// Driver layer errors occurred. Release the semaphore and report that.
	xSemaphoreGive(waitSem);

	// TODO comment/uncomment depending on whether its standalone tests or with CSP
	return transmissionError; // If standalone
	//return CSP_ERR_DRIVER;  // If integrated with CSP
}

/******************************************************************************
 * @brief Setup the DMA channels for I2C
 * All functions called here are void, so we only return void.
 *
 * @param TX Tx DMA Channel
 * @param RX Rx DMA Channel
 *****************************************************************************/
static void i2cDMA_ChannelInit(int TxChan, int TxReg, int RxChan, int RxReg) {

	/* Initialization Struct, and the Tx Structs */
	DMA_CfgChannel_TypeDef  txChannelConfig;
	DMA_CfgDescr_TypeDef	txDescriptorConfig;

	DMA_CfgChannel_TypeDef  rxChannelConfig;
	DMA_CfgDescr_TypeDef	rxDescriptorConfig;

	/* Setup call-back function */
	dmaCB.cbFunc  = i2cTransferComplete; // Function that is called in the ISR context.
	dmaCB.userPtr = NULL;                // We're not passing that function any parameters.

	/* Setting up TX channel */
	txChannelConfig.highPri   = true;             // High Priority when arbitration.
	txChannelConfig.enableInt = true;             // Yes, when the transfer is complete, the DMA IRQ will fire.
	txChannelConfig.select    = TxReg; // Transfer when the modules I2C TX Buffer is empty. // TODO This will need to change for the M4.
	txChannelConfig.cb        = &dmaCB;           // Information on what callback function is tied to this channel.
	DMA_CfgChannel(TxChan, &txChannelConfig);

	/* Setting up TX channel descriptor */
	txDescriptorConfig.dstInc  = dmaDataIncNone; // Do not increment the destination address after each transfer as its a hardware register.
	txDescriptorConfig.srcInc  = dmaDataInc1;    // Increment source register by 1 byte after each transfer.
	txDescriptorConfig.size    = dmaDataSize1;   // The size of the data to be moved is 1 byte.
	txDescriptorConfig.arbRate = dmaArbitrate1;  // Arbitrate after each transfer.
	txDescriptorConfig.hprot   = 0;              // Protection is not an issue.
	DMA_CfgDescr(TxChan, true, &txDescriptorConfig);

	/* Setting up RX channel
	 * Same as setting up the Tx Channel with Rx equivalents. */
	rxChannelConfig.highPri    = true;
	rxChannelConfig.enableInt  = false;
	rxChannelConfig.select     = RxReg;
	rxChannelConfig.cb 		   = NULL;
	DMA_CfgChannel(RxChan, &rxChannelConfig);

	/* Setting up RX channel descriptor
	 * Same as setting up Tx Channel but mirrored increments. */
	rxDescriptorConfig.dstInc  = dmaDataInc1;
	rxDescriptorConfig.srcInc  = dmaDataIncNone;
	rxDescriptorConfig.size    = dmaDataSize1;
	rxDescriptorConfig.arbRate = dmaArbitrate1;
	rxDescriptorConfig.hprot   = 0;
	DMA_CfgDescr(RxChan, true, &rxDescriptorConfig);
}

/*****************************************************************************
 * @brief Deletes FreeRTOS objects
 ****************************************************************************/
static inline void i2c_FreeRTOS_Structs_Del() {
	vSemaphoreDelete(busySem);
	vSemaphoreDelete(waitSem);
}

/******************************************************************************
 * @brief Initializes the Tx Semaphore, Rx Data and Index queues, and shared memory
 *****************************************************************************/
static int8_t i2c_FreeRTOS_Structs_Init() {

	busySem = xSemaphoreCreateBinary(); // The driver is busy with someone else's frame.
	waitSem = xSemaphoreCreateBinary(); // The driver is attempting to transmit my frame.

	// TODO Comment this out when working with CSP.
	i2cSharedMem = xSharedMemoryCreate(sizeof(uint8_t) * I2C_MTU, NUM_SH_MEM_BUFS);

	// If the initialization of one failed, the driver cannot work. Delete all of them!
	if (busySem == NULL || waitSem == NULL || xSemaphoreGive(waitSem) != pdTRUE) {
		i2c_FreeRTOS_Structs_Del();
		return CSP_ERR_NOMEM;
	}

	// TODO comment out block when working with CSP.
	if (i2cSharedMem == NULL) {
		i2c_FreeRTOS_Structs_Del();
		return CSP_ERR_NOMEM;
	}

	// TODO Pick depending on CSP or standalone.
	i2c_Rx = pSharedMemGet(i2cSharedMem); // Standalone
	//i2c_Rx = csp_buffer_get(I2C_MTU + CSP_I2C_HEADER_LEN); // CSP

	return CSP_ERR_NONE;
}

/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 * YOU MUST INITALIZE THE DMA USING cspDMA_Init(...) FIRST!
 *****************************************************************************/
int csp_i2c_init(uint8_t opt_addr, int handle, int speed) {

	// Initialize all FreeRTOS structures required for the driver to run.
	int rtosErr = i2c_FreeRTOS_Structs_Init();
	// If initialization failed for WHATEVER reason, e.g. this is extensible code for multiple
	// bad error conditions, return said error condition.
	if (rtosErr != CSP_ERR_NONE) { return rtosErr;}

	// Decision block for which I2C module to use.
	// First we decide which registers
	// Then enable the clock for the appropriate line.
	// Then decide which IRQ to enable TODO hard coded for 1 on EFM32 with M3
	// Then enable pins
	// Then init DMA channels
	enum IRQn I2C_IRQ;
	if (handle == 0){
		I2CRegs = I2C0;
		CMU_ClockEnable(cmuClock_I2C0, true);
		I2C_IRQ = I2C0_IRQn;

		GPIO_PinModeSet(I2C0_Ports, I2C0_SDA, gpioModeWiredAndPullUpFilter, 1);
		GPIO_PinModeSet(I2C0_Ports, I2C0_SCL, gpioModeWiredAndPullUpFilter, 1);

		i2cDMA_ChannelInit(DMA_CHANNEL_I2C_TX, DMAREQ_I2C0_TXBL, DMA_CHANNEL_I2C_RX, DMAREQ_I2C0_RXDATAV);
	}

	else if (handle == 1) {
		I2CRegs = I2C1;
		CMU_ClockEnable(cmuClock_I2C1, true);
		I2C_IRQ = I2C1_IRQn;

		GPIO_PinModeSet(I2C1_Ports, I2C1_SDA, gpioModeWiredAndPullUpFilter, 1);
		GPIO_PinModeSet(I2C1_Ports, I2C1_SCL, gpioModeWiredAndPullUpFilter, 1);

		i2cDMA_ChannelInit(DMA_CHANNEL_I2C_TX, DMAREQ_I2C1_TXBL, DMA_CHANNEL_I2C_RX, DMAREQ_I2C1_RXDATAV);
	}

	// Add more channels if you want.
	else {
		i2c_FreeRTOS_Structs_Del();
		return CSP_ERR_INVAL;
	}

	/*
	* Changing the priority of I2C1 IRQ.
	* It must be numerically equal to or greater than configMAX_SYSCALL_INTERRUPT_PRIORITY
	* defined in FreeRTOSConfig.h
	* Currently, that is set to 5.
	* The I2C priority level goes to 6, and the DMA 5.
	* If you want I2C to take priority above the DMA
	* YOU MUST move the transfer calc code to the I2C IRQ first.
	*/
	NVIC_SetPriority(I2C_IRQ, I2C_INT_PRIO_LEVEL);

	// Driver works for 100kbit/s
	if (speed == STANDARD_CLK) {
		I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
		I2C_Init(I2CRegs, &i2cInit);
	}

	// Driver works for 400kbit/s
	else if (speed == FAST_CLK) {
		I2C_Init_TypeDef i2cInit = { \
			true, \
			true, \
			0, \
			I2C_FREQ_FAST_MAX, \
			i2cClockHLRAsymetric \
		};
		I2C_Init(I2CRegs, &i2cInit);
	}

	// Driver doesn't work at 1Mbit/s because pickles.
	else {
		i2c_FreeRTOS_Structs_Del();
		return CSP_ERR_INVAL;
	}

	/* Enable pins at location 1 */
	I2CRegs->ROUTE = I2C_ROUTE_SDAPEN |
			I2C_ROUTE_SCLPEN |
			(I2C_ROUTE_LOC << _I2C_ROUTE_LOCATION_SHIFT);

	/* Setting up to enable slave mode */
	I2CRegs->SADDR = opt_addr;
	I2CRegs->CTRL |= csp_I2C_ctrl;
	I2CRegs->SADDRMASK |= CSP_SADDR_MASK;

	// Set Rx index to zero.
	i2c_RxInProgress = false;

	// We do an operation on the first Rx.
	firstRx = true;

	// Enable interrupts
	I2C_IntClear(I2CRegs, i2c_IFC_flags);
	I2C_IntEnable(I2CRegs, i2c_IEN_flags);
	I2CRegs->CTRL |= I2C_CTRL_AUTOACK;
	NVIC_EnableIRQ(I2C_IRQ);

	// We're starting/restarting the board, so it assume the bus is busy
	// We need to either have a clock-high (BITO) timeout, or issue an abort
	if (I2CRegs->STATE & I2C_STATE_BUSY) {
		I2CRegs->CMD = I2C_CMD_ABORT;
	}

	return CSP_ERR_NONE; // Everything went fine. Good to go!
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

		  // If the previous pend failed the buffer variable will be set to NULL
		  // Since it is not, we have a pointer to use
		  if (i2c_Rx != NULL) {

			  // Setup DMA Transfer
			  i2c_Rx[0] = I2CRegs->RXDATA; // Get the first byte of data.
			  //I2CRegs->CTRL |= I2C_CTRL_AUTOACK; // Enable automatic acknowledgements.
			  i2c_RxInProgress = true; // Raise progress flag.
			  DMA_ActivateBasic(DMA_CHANNEL_I2C_RX,      // Rx Channel
					  true, 						     // Primary channel
					  false,                             // No burst.
					  (void*)i2c_Rx + 2,                 // Destination Register
					  (void*)&(I2CRegs->RXDATA),         // Source register
					  I2C_MTU + CSP_I2C_HEADER_LEN - 1); // Bytes to Rx
			  i2c_Rx[1] = I2CRegs->RXDATA;               // 2nd byte arrived before DMA is ready.
		  }

		  // No pointers are available so we cannot accept the data.
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
   * Cease auto stop, clear ALL flags since We. Don't. Care.
   * Report success to upper layer.
   */
  if (flags & I2C_IF_MSTOP) {
	  I2C_IntClear(I2CRegs, i2c_IFC_flags);
	  I2CRegs->CTRL &= ~I2C_CTRL_AUTOSE;
	  I2C_IntEnable(I2CRegs, I2C_IEN_CLTO | I2C_IEN_BITO);
	  xSemaphoreGiveFromISR(busySem, NULL);
	  return;
  }

  /*
   * Stop Condition (or Repeated Start) detected
   * This flag is raised regardless
   * of whether or not the Master was speaking to us
   * CHECK and see if we were spoken to
   * By looking at a boolean set when the I2C_IF_ADDR flag was raised.
   */
  if (flags & (I2C_IF_SSTOP | I2C_IF_RSTART)) {
	  // We were Rx'ing stuff, so we had a buffer, etc.
	  if (i2c_RxInProgress) {

		  i2c_RxInProgress = false; // We are no longer in Rx.

		  // Cast the Rx buffer as a CSP I2C Frame.
		  i2c_frame_t *cspBuf = (i2c_frame_t*)i2c_Rx;

		  // VERY IMPORTANT THIS IS HOW YOU GET RX DATA SIZE!!!
		  // The goal of this line is to isolate the n_minus_1 field, and shift it.
		  // Then, we take the maximum # of bytes possible, subtract the previously calculated number and add 1.
		  cspBuf->len = I2C_MTU + CSP_I2C_HEADER_LEN
				  - ((*getRxDMACtrlAddr() & TRANS_REMAIN_MASK) >> TRANS_REMAIN_SHIFT) + 1 ;

		  // I literally put this here to prevent a size misalignment on the first transfer.
		  if (firstRx) {
			  firstRx = false;
			  cspBuf->len = cspBuf->len - 2;
		  }

		  printf("Padding: %x, Retries: %x, Reserved: %d, Dest: %x, Len_rx: %x, Len: %d, \n Data: %s\n", \
				  cspBuf->padding, cspBuf->retries, cspBuf->reserved, cspBuf->dest, cspBuf->len_rx, cspBuf->len, cspBuf->data);

		  // TODO this changes depending on standalone  or CSP
		  xSharedMemPutFromISR(i2cSharedMem, i2c_Rx, NULL); // Standalone
		  //csp_i2c_rx(cspBuf, void * pxTaskWoken)  // CSP
		  i2c_Rx = NULL;
	  }

	  // The buffer pointer was set to pdFalse/NULL either by
	  // a) A previous line in the SSTOP/RSTART ISR after a previous frame had been received.
	  // b) A previous execution of this conditional that failed before.
	  if (i2c_Rx == NULL) {

		  // Now, pend a NEW MEMORY block for the next time we need it. TODO Standalone vs CSP
		  i2c_Rx = pSharedMemGetFromISR(i2cSharedMem, NULL); // Standalone
		  //i2c_Rx = csp_buffer_get_isr(I2C_MTU + CSP_I2C_HEADER_LEN); // CSP

		  if (i2c_Rx == NULL) { I2CRegs->CTRL &= ~I2C_CTRL_AUTOACK; } // We did not get the block, so disable auto-ack.
		  else { I2CRegs->CTRL |= I2C_CTRL_AUTOACK; } // We have a block ready, so setup auto-acks.
	  }

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
	  // Receive it's item but do nothing with it.
	  if (!i2c_RxInProgress) {
		  DMA->IFS = DMA_COMPLETE_I2C_TX;
	  }

	  I2C_IntClear(I2CRegs, i2c_IFC_flags);
	  I2CRegs->CMD = I2C_CMD_ABORT;
	  i2c_RxInProgress = false;
	  xSemaphoreGiveFromISR(busySem, NULL);
  }
}
