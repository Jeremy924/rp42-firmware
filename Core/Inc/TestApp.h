/*
 * TestApp.h
 *
 *  Created on: May 13, 2025
 *      Author: Jerem
 */

#ifndef INC_TESTAPP_H_
#define INC_TESTAPP_H_

#include "FSUtils.h"

/*
 * 	const char* filename;
	unsigned int flags;
	uint32_t handle;
 */
int test_app_main() {
	struct FileOpenParams open_file_params = {
			"0:/test.txt",
			 FA_WRITE | FA_CREATE_ALWAYS,
			0
	};

	systemCallData.args = (void*) &open_file_params;
	systemCallData.command = 0x100;
	__asm("SVC #0");
	if (systemCallData.result != FR_OK) {
		printf("failed to open file\r\n");

		return -1;
	}

	BYTE* wBuf = "test";

	struct FileRWParams rw_file_params = {
			open_file_params.handle,
			0,
			wBuf,
			strlen(wBuf)
	};
	systemCallData.args = (void*) &rw_file_params;
	systemCallData.command = 0x111;
	__asm("SVC #0");
	if (systemCallData.result != FR_OK) {
		printf("failed to write");
		return -3;
	}

	systemCallData.args = (void*) open_file_params.handle;
	systemCallData.command = 0x101;
	__asm("SVC #0");

	if (systemCallData.result != FR_OK) {
		printf("failed to close file\r\n");
		return -2;
	}



	open_file_params.flags = FA_READ;

	systemCallData.args = (void*) &open_file_params;
	systemCallData.command = 0x100;
	__asm("SVC #0");
	if (systemCallData.result != FR_OK) {
		printf("failed to open file\r\n");

		return -1;
	}

	BYTE* buf[256];
	rw_file_params.buf = buf;
	rw_file_params.len = 256;

	systemCallData.args = (void*) &rw_file_params;
	systemCallData.command = 0x110;
	__asm("SVC #0");
	if (systemCallData.result != FR_OK) {
		printf("failed to read");
		return -3;
	}

	systemCallData.args = (void*) open_file_params.handle;
	systemCallData.command = 0x101;
	__asm("SVC #0");

	if (systemCallData.result != FR_OK) {
		printf("failed to close file\r\n");
		return -2;
	}



	return 0;
}

#endif /* INC_TESTAPP_H_ */
