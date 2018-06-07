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

#include "cspI2C_EFM32.h"

void vThrowI2CErrors(void *iDontCare) {

	vTaskDelay(portTICK_PERIOD_MS * 3);
	puts("Tremble before me, I2C Driver. I am going to throw error flags!");

	while (1) {
		//int myRand = rand();
		//if (myRand % 3 == 0) {
			// Throw ARBLOST
		puts("THROW ARBLOST");
		I2C1->IFS |= I2C_IFS_ARBLOST;
		vTaskDelay(portTICK_PERIOD_MS * 4);
		//}

		//else if (myRand % 7 == 0) {
			// Throw BUSERR
			//puts("THROW BUSERR");
			//I2C1->IFS |= I2C_IFS_BUSERR;
			//vTaskDelay(portTICK_PERIOD_MS * 10);
		//}

//		else if (myRand % 11 == 0) {
//			// Throw CLTO
//			puts("THROW CLTO");
//			I2C1->IFS |= I2C_IFS_CLTO;
//			vTaskDelay(portTICK_PERIOD_MS * 10);
//		}
//
//		else if (myRand % 5 == 0) {
//			// Throw BITO
//			puts("THROW BITO");
//			I2C1->IFS |= I2C_IFS_BITO;
//			vTaskDelay(portTICK_PERIOD_MS * 10);
//		}
//
//		else {
//			// Short delay
//			vTaskDelay(portTICK_PERIOD_MS * 5);
//		}
	}
}


#endif /* EFM32_THROWI2CERRORS_H_ */
