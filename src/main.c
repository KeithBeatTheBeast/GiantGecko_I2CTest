/*
 * main.c
 *
 *  Created on: Jun 29, 2018
 *      Author: keith
 */

#include "cspI2C_EFM32_M3.h"
#include "makePrintfWork.h"
int main(void) {
	// Use this to enable printfs.
	SWO_SetupForPrint();

	/* Setting up DMA Controller */
	cspDMA_Init(CSP_HPROT);

	csp_i2c_init(0, 0, 0);
	vTaskStartScheduler();
}
