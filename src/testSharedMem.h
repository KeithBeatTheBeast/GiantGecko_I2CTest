/*
 * testSharedMem.h
 *
 *  Created on: Jun 6, 2018
 *      Author: kgmills
 *
 *      Simple code to test SharedMemory.c/h
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include "SharedMemory.h"

bool testReadAndWriteSH() {

	const char* test_string = "test!";
	SharedMem_t testMemBase = xSharedMemoryCreate(sizeof(uint8_t) * 5, 1);

	for (int i = 0; i < 50; i++) {
		uint8_t *testMemCell = (uint8_t*)pSharedMemGet(testMemBase);
		strcpy(testMemCell, test_string);
		testMemCell[i%5] = toupper(testMemCell[i%5]);
		printf("%s\n", testMemCell);
		if (xSharedMemPut(testMemBase, (void*)testMemCell) == pdFALSE) {
			return false;
		}
	}
	return true;
}


void runAllSharedMemTests() {

	if (testReadAndWriteSH()) {puts("Test Read And Write Passed");}
	else {puts("Test Read And Write Failed");}
}
