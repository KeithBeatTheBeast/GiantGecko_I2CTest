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
	theFrame->len = 1;
	theFrame->len_rx = 5;
	theFrame->padding = 1;
	theFrame->reserved = 554;
	theFrame->retries = 5;

	int k = 0;
	int r = 0;
	while (1) {
		r = i2c_send(1, theFrame, portTICK_PERIOD_MS * 4);
		if (r == CSP_ERR_NONE) {
			printf("Size of frame: %d, Number runs: %d\n", theFrame->len + CSP_I2C_HEADER_LEN, ++k);
		}
		//vTaskDelay(portTICK_PERIOD_MS * 1);
	}
}

int main(void) {
	// Use this to enable printfs.
	SWO_SetupForPrint();

	/* Setting up DMA Controller */
	cspDMA_Init(CSP_HPROT);

	csp_i2c_init(0xE2, 1, 400);

	xTaskCreate(vI2CTransferTask, (const char *) "I2CRegs_Tx", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	vTaskStartScheduler();
}
