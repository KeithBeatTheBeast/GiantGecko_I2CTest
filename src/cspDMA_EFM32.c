/*
 * cspDMA_EFM32.c
 *
 *  Created on: Jun 20, 2018
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

#include "cspDMA_EFM32.h"

// Descriptor memory space do not do anything
//uint8_t cbrAddr[NUM_DMA_CHANNELS * DESCRIPTOR_SIZE * ADDR_SPACE_SPARE_COEF];

void cspDMA_Init(uint8_t hprot) {
	/*
	 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	 * THIS IS A VERY IMPORTANT CODE BLOCK - BREAK IT AND THE DMA. WILL. NOT. WORK.
	 * The base address for the DMA controller's descriptors MUST be a multiple of the total size
	 * of the memory allocated.
	 * There are some ways to achieve this end, one way is to use a specialized malloc, memalign
	 * https://linux.die.net/man/3/memalign
	 * However, it is obsolete. Also our faculty advisor said this is a no-no.
	 *
	 * Another way is to allocate more than you need, but enough so that you can be guaranteed to find an appropriate address
	 * https://stackoverflow.com/questions/227897/how-to-allocate-aligned-memory-only-using-the-standard-library
	 * I will let the most popular answer explain how it works.
	 *
	 */
	uint16_t align = NUM_DMA_CHANNELS * DESCRIPTOR_SIZE;
	uintptr_t mask = ~(uintptr_t)(align - 1);
	void *cbrAddr = pvPortMalloc(NUM_DMA_CHANNELS * DESCRIPTOR_SIZE * ADDR_SPACE_SPARE_COEF);

	/* Set pointer to control block, notice that this ptr must have been */
	/* properly aligned, according to requirements defined in the reference */
	/* manual. */
	DMA->CTRLBASE = (uint32_t)(((uintptr_t)cbrAddr + align - 1) & mask);

	/* END OF VERY IMPORTANT CODE BLOCK THE REST IS COPY/PASTA FROM SILICON LABS! */

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

