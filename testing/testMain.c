/*
 * main.c
 *
 *  Created on: Jun 29, 2018
 *      Author: keith
 *
 *      THIS IS TEST CODE DO NOT ADD IMPORT IT INTO CSP AS PART OF THE DRIVER.
 */

#include "../src/cspI2C_EFM32_M3.h"
#include "makePrintfWork.h"

static void vI2CTransferTask(void *nothing) {

	i2c_frame_t *theFrame = pvPortMalloc(sizeof(i2c_frame_t));
	memcpy(theFrame->data, "TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting ", I2C_MTU);
	theFrame->dest = 0xE2;
	theFrame->len = 502;
	theFrame->len_rx = 0x40;
	theFrame->padding = 0x21;
	theFrame->reserved = 554;
	theFrame->retries = 0x23;

	int k = 0;
	int e = 0;
	while (1) {
		e = i2c_send(1, theFrame, portTICK_PERIOD_MS * 4);
		if (e == CSP_ERR_NONE) {
			k++;
		}
		else {
			printf("Frame Size: %d Error: %x Number of Runs: %d\n", theFrame->len + CSP_I2C_HEADER_LEN, e, k);
		}
	}
}

int main(void) {
	// Use this to enable printfs.
	SWO_SetupForPrint();

	/* Setting up DMA Controller */
	cspDMA_Init(CSP_HPROT);

	if (csp_i2c_init(0xE2, 1, 400) == CSP_ERR_NONE) {

		xTaskCreate(vI2CTransferTask, (const char *) "I2CRegs_Tx", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
		vTaskStartScheduler();
	}
	else {
		puts("There was an error in csp_i2c_init! Scheduler NOT started.");
	}
}
