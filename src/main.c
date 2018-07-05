/*
 * main.c
 *
 *  Created on: Jun 29, 2018
 *      Author: keith
 *
 *      THIS IS TEST CODE DO NOT ADD IMPORT IT INTO CSP AS PART OF THE DRIVER.
 */

#include "cspI2C_EFM32_M3.h"
#include "makePrintfWork.h"

static void vI2CTransferTask(void *nothing) {

	i2c_frame_t *theFrame = pvPortMalloc(sizeof(i2c_frame_t));
	memcpy(theFrame->data, "TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting TESTINGtesting ", I2C_MTU);
	theFrame->dest = 0xE2;
	theFrame->len = 990;
	theFrame->len_rx = 5;
	theFrame->padding = 1;
	theFrame->reserved = 554;
	theFrame->retries = 5;

	while (1) {
		theFrame->len %= 1014;
		if (theFrame->len == 0) {theFrame->len = 50;}
		i2c_send(1, theFrame, portTICK_PERIOD_MS * 4);
		vTaskDelay(portTICK_PERIOD_MS * 1);
		theFrame->len++;
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
