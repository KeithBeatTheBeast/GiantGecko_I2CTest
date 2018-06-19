/*
 * cspDMA_EFM32.h
 *
 *  Created on: Jun 19, 2018
 *      Author: kgmills
 *
 *      File contains DMA configuration information
 *      for all used peripherals.
 *      Such as defining number of channels, the channels used
 *      by each peripheral, and also that pesky memory managment stuff.
 *
 *      See Silicon Labs Application Note #0013 for more information
 *      https://www.silabs.com/documents/public/application-notes/AN0013.pdf
 *
 */

#ifndef CSPDMA_EFM32_H_
#define CSPDMA_EFM32_H_

#include "em_dma.h"

/*
 * NUM_DMA_CHANNELS - Max 12 primary, 12 alternate (which I'm not touching)
 * 	Currently: 1, for I2C Tx
 * DESCRIPTOR_SIZE - Size of a descriptor block needed to be allocated.
 * ADDR_SPACE_SPARE_COEF - The address for the descriptor blocks must be aligned
 * 	It's base needs to be a multiple of its size.
 * 	Faculty Advisor, has told me to allocate 1.5x the space
 * 	rather than use a function like memalign
 * controlBlock - Array used. Will be aligned on init.
 */
#define NUM_DMA_CHANNELS				1
#define DESCRIPTOR_SIZE					16
#define ADDR_SPACE_SPARE_COEF			2
#define CSP_HPROT						0

/*
 * I2C DMA CONSTANTS
 * DMA_CHANNEL_I2C_TX is the channel used, between 0 and 11
 * DMA_ENABLE_I2C_TX is the ENABLE/DISABLE bit for the DMA->CHENS register.
 * 	It is a wrapper for another variable, which should be of the form
 * 	DMA_CHENS_CH#ENS where # is the channel number
 * dmaCB is the callback structure.
 */
#define DMA_CHANNEL_I2C_TX 				0
#define DMA_ENABLE_I2C_TX  				DMA_CHENS_CH0ENS
static DMA_CB_TypeDef 					dmaCB;

/***************************************************************************//**
 * @brief
 *   Initializes DMA controller. Copied from SiLabs code from em_dma.c and modified so that it can work
 *   For a variable number of active DMA channels instead of assuming all are active.
 *
 * @details
 *   This function will reset and prepare the DMA controller for use. Although
 *   it may be used several times, it is normally only used during system
 *   init. If reused during normal operation, notice that any ongoing DMA
 *   transfers will be aborted. When completed, the DMA controller is in
 *   an enabled state.
 *
 * @note
 *   Must be invoked before using the DMA controller.
 *
 * @param[in] init
 *   Pointer to a structure containing DMA init information.
 ******************************************************************************/
void cspDMA_Init(uint8_t hprot) {

	uint16_t align = NUM_DMA_CHANNELS * DESCRIPTOR_SIZE;
	uintptr_t mask = ~(uintptr_t)(align - 1);
	void *cbrAddr = pvPortMalloc(NUM_DMA_CHANNELS * DESCRIPTOR_SIZE * ADDR_SPACE_SPARE_COEF);

	/* Set pointer to control block, notice that this ptr must have been */
	/* properly aligned, according to requirements defined in the reference */
	/* manual. */
	DMA->CTRLBASE = (uint32_t)(((uintptr_t)cbrAddr + align - 1) & mask);

	/* Make sure DMA clock is enabled prior to accessing DMA module */
	CMU_ClockEnable(cmuClock_DMA, true);

	/* Make sure DMA controller is set to a known reset state */
	DMA_Reset();

	/* Clear/enable DMA interrupts */
	NVIC_ClearPendingIRQ(DMA_IRQn);
	NVIC_EnableIRQ(DMA_IRQn);

	/* Enable bus error interrupt */
	DMA->IEN = DMA_IEN_ERR;

		/* Configure and enable the DMA controller */
	DMA->CONFIG = (hprot << _DMA_CONFIG_CHPROT_SHIFT)
                | DMA_CONFIG_EN;
}

#endif /* CSPDMA_EFM32_H_ */
