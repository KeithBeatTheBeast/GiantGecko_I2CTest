/*
 * EFM32_throwI2CErrors.h
 *
 *  Created on: Jun 7, 2018
 *      Author: keith
 *
 *      Makes a task with a higher priority than the Tx and Rx tasks which, on a delay and
 *      random number generator, throw Interrupt Error Flags at the I2C Module
 */

#ifndef EFM32_THROWI2CERRORS_H_
#define EFM32_THROWI2CERRORS_H_

#include "../src/cspI2C_EFM32_M3.h"

void vThrowI2CErrors(void *iDontCare) {

	int delayMult = 3;
	vTaskDelay(portTICK_PERIOD_MS * 3);
	puts("Tremble before me, I2C Driver. I am going to throw error flags!");

	while (1) {

		puts("THROW ARBLOST");
		I2C1->IFS |= I2C_IFS_ARBLOST;
		vTaskDelay(portTICK_PERIOD_MS * delayMult);

		puts("THROW BUSERR");
		I2C1->IFS |= I2C_IFS_BUSERR;
		vTaskDelay(portTICK_PERIOD_MS * delayMult);

		puts("THROW CLTO");
		I2C1->IFS |= I2C_IFS_CLTO;
		vTaskDelay(portTICK_PERIOD_MS * delayMult);

		puts("THROW BITO");
		I2C1->IFS |= I2C_IFS_BITO;
		vTaskDelay(portTICK_PERIOD_MS * delayMult);
	}
}


#endif /* EFM32_THROWI2CERRORS_H_ */
