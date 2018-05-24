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
 * Edited by Kgmills on 5/11/18
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
 * An oscilloscope was used for initial testing purposes to see if the
 * clock and data lines were being pulled low. These pins were grounded to the
 * GND pins on the same board that provides 3.3V.
 *
 **************************************************************************
 * WHATEVER YOU DO, DO NOT PULL-UP THE PINS USING THE 5V LINE ON THE BOARD.
 **************************************************************************
 *
 ******************************************************************************/

#include <stdbool.h>
#include <ctype.h> // Added to uppercase message
#include "em_device.h"
#include "em_chip.h"
#include "em_i2c.h"
#include "em_rtc.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"

// Files I use to make printf(...) work on the EFM32 Giant Gecko using Simplicity Studio.
#include "makePrintfWork.h"

/* Defines*/
#define CORE_FREQUENCY              14000000
#define RTC_MIN_TIMEOUT                32000 
#define I2C_ADDRESS                     0xE2
#define I2C_RXBUFFER_SIZE                 10

// Buffers++
uint8_t i2c_txBuffer[] = "let go of my gecko!"; // Modified message.
uint8_t i2c_txBufferSize = sizeof(i2c_txBuffer) - 1;
uint8_t i2c_rxBuffer[I2C_RXBUFFER_SIZE * 2];
uint8_t i2c_rxBufferIndex;

int txIndex = -1;

// Transmission flags
volatile bool i2c_rxInProgress;
volatile bool i2c_startTx;

/* Transmission and Receiving Structure */
I2C_TransferSeq_TypeDef i2cTransfer;

// Temp variables for controlling ISR triggers.
// Adjust as needed for debugging and driver development

// This should be used as the 2nd argument to I2C_IntClear
// Different variable as RXDATAV and TXBL do not exist for the EFM32GG's IFC/IFS registers. Set to zero.
#define i2c_IFC_flags (I2C_IFC_ADDR | \
					  I2C_IFC_SSTOP | \
					  I2C_IFC_RSTART | \
					  I2C_IFC_BUSHOLD | \
					  I2C_IFC_ARBLOST | \
					  I2C_IFC_NACK | \
					  I2C_IFC_ACK | \
					  I2C_IFC_MSTOP | \
					  I2C_IFC_BUSERR | \
					  I2C_IFC_TXC)
//					  I2C_IFC_CLTO | \
//					  I2C_IFC_BITO | \
//					  I2C_IFC_RXUF | \
//					  I2C_IFC_TXOF | \
//					  I2C_IFC_BUSHOLD | \
//					  I2C_IFC_BUSERR| \
//					  I2C_IFC_ARBLOST | \
//					  I2C_IFC_MSTOP | \
//					  I2C_IFC_NACK | \
//					  I2C_IFC_ACK | \
//					  I2C_IFC_TXC | \
//					  I2C_IFC_RSTART | \
//					  I2C_IFC_START \
// Should be used as the 2nd argument for I2C_IntEnable/I2C_IntDisable
#define i2c_IEN_flags (I2C_IEN_ADDR | \
					  I2C_IEN_RXDATAV | \
					  I2C_IEN_SSTOP | \
					  I2C_IEN_BUSHOLD | \
					  I2C_IEN_RSTART | \
					  I2C_IEN_ARBLOST | \
					  I2C_IEN_NACK | \
					  I2C_IEN_ACK | \
					  I2C_IEN_MSTOP | \
					  I2C_IEN_BUSERR | \
					  I2C_IEN_TXC)
//					  I2C_IEN_CLTO | \
//					  I2C_IEN_BITO | \
//					  I2C_IEN_RXUF | \
//					  I2C_IEN_TXOF | \
//					  I2C_IEN_BUSERR| \
//					  I2C_IEN_ARBLOST | \
//					  I2C_IEN_MSTOP | \
//					  I2C_IEN_NACK | \
//					  I2C_IEN_ACK | \
//					  I2C_IEN_RSTART | \
//					  I2C_IEN_START \

/**************************************************************************//**
 * @brief  Starting oscillators and enabling clocks
 *****************************************************************************/
void setupOscillators(void)
{
  /* Enabling clock to the I2C, GPIO, LE */
  CMU_ClockEnable(cmuClock_I2C1, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_CORELE, true);
  
  // Enabling USART0 (see errata)
  CMU_ClockEnable(cmuClock_USART0, true);
  
  /* Starting LFXO and waiting until it is stable */
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

  /* Routing the LFXO clock to the RTC */
  CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_RTC, true);
}

/**************************************************************************//**
 * @brief  enables I2C slave interrupts
 *****************************************************************************/
void enableI2C1Interrupt(void){

  I2C_IntClear(I2C1, i2c_IFC_flags);
  I2C_IntEnable(I2C1, i2c_IEN_flags);
  NVIC_EnableIRQ(I2C1_IRQn);
}

/**************************************************************************//**
 * @brief  Setup I2C
 *****************************************************************************/
void setupI2C(void)
{
  // Using default settings
  I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;

  /* Using PD6 (SDA) and PD7 (SCL) */
  GPIO_PinModeSet(gpioPortC, 4, gpioModeWiredAndPullUpFilter, 1);
  GPIO_PinModeSet(gpioPortC, 5, gpioModeWiredAndPullUpFilter, 1);

  /* Enable pins at location 1 */
  I2C1->ROUTE = I2C_ROUTE_SDAPEN |
                I2C_ROUTE_SCLPEN |
                (0 << _I2C_ROUTE_LOCATION_SHIFT);

  /* Initializing the I2C */
  I2C_Init(I2C1, &i2cInit);
  
  /* Setting the status flags and index */
  i2c_rxInProgress = false;
  i2c_startTx = true;
  i2c_rxBufferIndex = 0;

  /* Initializing I2C transfer */
  i2cTransfer.addr          = I2C_ADDRESS;
  i2cTransfer.flags         = I2C_FLAG_WRITE;
  i2cTransfer.buf[0].data   = i2c_txBuffer;
  i2cTransfer.buf[0].len    = sizeof(i2c_txBuffer);
  i2cTransfer.buf[1].data   = i2c_rxBuffer;
  i2cTransfer.buf[1].len    = I2C_RXBUFFER_SIZE;

  /* Setting up to enable slave mode */
  I2C1->SADDR = I2C_ADDRESS;
  I2C1->CTRL |= I2C_CTRL_SLAVE | I2C_CTRL_AUTOACK;// | I2C_CTRL_AUTOSN | I2C_CTRL_AUTOSE; //TODO this sends STOP when NACK received
  enableI2C1Interrupt();
}

/**************************************************************************//**
 * @brief  Transmitting I2C data. Will busy-wait until the transfer is complete.
 *****************************************************************************/
void performI2CTransfer(void) {
  // I2C_TransferSeq_TypeDef used to be here.
  // Made a global.

  
  if (I2C1->STATE & I2C_STATE_BUSY) {
	  I2C1->CMD |= I2C_CMD_ABORT; //TODO correct for the fact that we're designing for multiple masters.
  }

  // Reset Index
  txIndex = -1;

  // Load address, assuming write bit.
  I2C1->TXDATA = i2cTransfer.addr | 0x0000; // Flag write was 0x0001....
  //I2C1->TXDATA = i2cTransfer.buf[0].data[txIndex]; // This was originally below.

  // Issue start condition
  I2C1->CMD |= I2C_CMD_START;



  return;
}

/**************************************************************************//**
 * @brief  Setup RTC
 *****************************************************************************/
void setupRTC(void)
{  
  /* Variable for calculating the RTC timeout */
  uint16_t rtcTimeout;    
  RTC_Init_TypeDef rtcInit = RTC_INIT_DEFAULT;
  
  /* Getting an individual timeout value based upon the UNIQUE_0 field in the
  Device Information table */
  rtcTimeout = (*(uint32_t*)0x0FE081F0);  
  /* Setting a minimum timeout value */
  if(rtcTimeout < RTC_MIN_TIMEOUT){
    rtcTimeout += RTC_MIN_TIMEOUT;
  }
 
  // Setting RTC timeout
  RTC_CompareSet(0, rtcTimeout);

  /* Enabling Interrupt from RTC */
  RTC_IntEnable(RTC_IFC_COMP0);
  NVIC_EnableIRQ(RTC_IRQn);

  /* Enabling the RTC, using  */
  RTC_Init(&rtcInit);
}

/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void) {
  /* Initialize chip */
  CHIP_Init();
  
  /* Configuring clocks in the Clock Management Unit (CMU) */
  setupOscillators(); // TODO: The command for the I2C1 clock is causing RXUF flag to be raised.
  
  /* Setting up i2c */
  setupI2C();
  
  /* Setting up rtc*/
  setupRTC();
  
  // Use this to enable printfs.
  SWO_SetupForPrint();

  int i = 0; // Variable for capitalizing one character at a time.
  while (1)
  {
    if(i2c_rxInProgress){
       /* Receiving data */
    	while(i2c_rxInProgress){;}

        //disableI2cInterrupts();
        //i2c_startTx = true;

       // A crude, but effective way to capitalize one letter at a time and then print to screen.
       i2c_rxBuffer[i] = toupper(i2c_rxBuffer[i]);
       printf("%s\n", i2c_rxBuffer);
       i++;

       // Cycle back at end of buffer. It's the same message each time.
	   if (i > i2c_txBufferSize) {
		   i = 0;
	   }
    }else if (i2c_startTx){
       /* Transmitting data */
       performI2CTransfer();
       /* Transmission complete */
       i2c_startTx = false;
    }
  }
}



/**************************************************************************//**
 * @brief RTC Interrupt Handler, clears the flag.
 *        The interrupt table is in assembly startup file startup_efm32.s
 *****************************************************************************/
void RTC_IRQHandler(void)
{
  /* Clear interrupt source */
  RTC_IntClear(RTC_IFC_COMP0);
  
  /* If RX is not in progress, a new transfer is started*/
  if (!i2c_rxInProgress){
    //disableI2cInterrupts();
    i2c_startTx = true;
  }
}

/*
 * Function for adding new byte to TX buffer in the case of TXC and TXBL IF conditions
 * First it increments the index of the tx buffer.
 * If the pointer is equal to the size of the buffer, we've reached the end.
 * Send TRUE, in which case the IRQ calling this function will send a STOP condition
 * If the pointer is less than the size of the buffer, we've still got more data
 * Add the next byte to the buffer, return false.
 */
static inline bool addNewByteToTxBuffer() {

	if (++txIndex >= i2cTransfer.buf[0].len) {
		return true;
	}

	else if (I2C1->IF & I2C_IF_TXBL) {
		I2C1->TXDATA = i2cTransfer.buf[0].data[txIndex];
		I2C_IntClear(I2C1, I2C_IFC_TXC);
		printf("Char at location %d: %c\n", txIndex, i2cTransfer.buf[0].data[txIndex]);
	}
	return false;
}

/**************************************************************************//**
 * @brief I2C Interrupt Handler.
 *        The interrupt table is in assembly startup file startup_efm32.s
 *****************************************************************************/
void I2C1_IRQHandler(void) {
   
  int status = I2C1->IF;
  int state = I2C1->STATE;

  if (status & I2C_IF_BUSHOLD) {
	  puts("BUSHOLD");
	  printf("State: %x\n", state);

	  if (state == 0xdf) {
		  puts("Data transmitted, NACK Received");
		  if (addNewByteToTxBuffer()) {
			  I2C1->CMD |= I2C_CMD_STOP;
			  puts("NACK Received after data, stop issued");
		  }

		  else {
			  I2C1->CMD |= I2C_CMD_CONT;
			  puts("NACK Received after data, no stop issued");
		  }

		  I2C_IntClear(I2C1, I2C_IFC_NACK);
	  }

	  else if (state == 0xd7) {
		  puts("Data transmitted, ACK Received");
		  if (addNewByteToTxBuffer()) {
			  I2C1->CMD |= I2C_CMD_STOP;
			  puts("ACK Received after data, stop issued");
		  }

		  else {
			  puts("ACK Received after data, no stop issued");
		  }

		  I2C_IntClear(I2C1, I2C_IFC_ACK);
	  }

	  else if (status & I2C_IF_ADDR) {
	      /* Address Match */
	      /* Indicating that reception is started */
		  i2c_rxInProgress = true;
	      I2C1->RXDATA;

	      I2C_IntClear(I2C1, I2C_IFC_ADDR);
	      puts("Address match non-repeat start");
	  }

	  else if (status & I2C_IF_RXDATAV) {
	      /* Data received */
	      i2c_rxBuffer[i2c_rxBufferIndex] = I2C1->RXDATA;
	      i2c_rxBufferIndex++;
	      puts("Data received");
	  }

	  else if (status & I2C_IF_SSTOP) {
	      /* Stop received, reception is ended */
	      I2C_IntClear(I2C1, I2C_IEN_SSTOP);
	      i2c_rxInProgress = false;
	      i2c_rxBufferIndex = 0;
	      puts("Stop condition detected");
	      printf("%s\n", i2c_rxBuffer);
	  }

	  else if (state == 0xb1) {
		  puts("Data Received");
	      i2c_rxBuffer[i2c_rxBufferIndex] = I2C1->RXDATA;
	      i2c_rxBufferIndex++;

	  }

	  I2C_IntClear(I2C1, I2C_IFC_BUSHOLD);
  }

  else if (status & I2C_IF_ARBLOST) {
	  // TODO handle ARBLOST better in the future
	  I2C_IntClear(I2C1, I2C_IEN_ARBLOST);
	  puts("Arbitration Lost");
  }

  else if (status & I2C_IF_TXC) {
	  I2C_IntClear(I2C1, I2C_IFC_TXC | I2C_IFC_BUSHOLD);

	  if (addNewByteToTxBuffer()) {
		  I2C1->CMD |= I2C_CMD_STOP;
		  puts("TXC Add New Byte has reached end of line");
	  }

	  else {
		  I2C1->CMD |= I2C_CMD_CONT;
	  }
	  puts("TXC else if");
  }

  else if ((status & I2C_IF_ACK) || (status & I2C_IF_NACK)) {
	  if (status & I2C_IF_TXBL) {
		  if (addNewByteToTxBuffer()) {
			  I2C1->CMD |= I2C_CMD_STOP;
			  puts("(N)ACK add new byte returned true");
		  }

		  else {
			  I2C1->CMD |= I2C_CMD_CONT;
			  puts("(N)ACK add new byte returned false");
		  }
	  }

	  I2C_IntClear(I2C1, I2C_IFC_ACK | I2C_IFC_NACK | I2C_IFC_BUSHOLD);
	  puts("(N)ACK");
  }

  else if (status & (I2C_IF_ADDR | I2C_IF_RSTART)) {
  	  /* Repeat Start condition
  	   * Assume auto-address matching
  	   * Reception has started
  	   */

  	  i2c_rxInProgress = true;
  	  I2C1->RXDATA;

  	  I2C_IntClear(I2C1, I2C_IFC_ADDR | I2C_IF_RSTART);
  	  puts("Repeat Start on auto-matching address");
    }
  else if (status & I2C_IF_ADDR) {
      /* Address Match */
      /* Indicating that reception is started */
	  i2c_rxInProgress = true;
      I2C1->RXDATA;

      I2C_IntClear(I2C1, I2C_IFC_ADDR);
      puts("Address match non-repeat start");
  }

  else if (status & I2C_IF_RXDATAV) {
      /* Data received */
      i2c_rxBuffer[i2c_rxBufferIndex] = I2C1->RXDATA;
      i2c_rxBufferIndex++;
      puts("Data received");
  }

  else if (status & I2C_IF_SSTOP) {
      /* Stop received, reception is ended */
      I2C_IntClear(I2C1, I2C_IEN_SSTOP);
      i2c_rxInProgress = false;
      i2c_rxBufferIndex = 0;
      puts("Stop condition detected");
      printf("%s\n", i2c_rxBuffer);
  }
}
