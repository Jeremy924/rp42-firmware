/*
 * FatFSTest.h
 *
 *  Created on: Apr 26, 2025
 *      Author: Jerem
 */

#ifndef INC_FATFSTEST_H_
#define INC_FATFSTEST_H_

#include "ff.h"         // FatFs core definitions
#include "main.h"       // For HAL_Delay, printf (or your logging equivalent)
#include <string.h>     // For strlen, memcmp
#include <stdio.h>      // For printf (adapt if using a different logging method)

// --- Test Configuration ---
#define TEST_FILENAME "test3.txt"
#define TEST_STRING   "Hello world"
#define READ_BUFFER_SIZE 256 // Should be large enough for TEST_STRING + null terminator

/**
 * @brief Runs a basic test sequence for FatFs on the QSPI flash drive.
 * @retval 0 on success, -1 on failure.
 */
int run_fatfs_qspi_test(void)
{
	FATFS QSPI_FatFs;
	FRESULT fr;
	FIL TestFile;
	UINT bytesWritten;
	UINT bytesRead;

	const char* drive_path = "0:";

	fr = f_mount(&QSPI_FatFs, TEST_FILENAME, 1);

	if (fr == FR_NO_FILESYSTEM) {
		BYTE work_buffer[512];

		fr = f_mkfs(drive_path, FM_ANY, 0, work_buffer, 512);

		// failed to create file system
		if (fr != FR_OK) {
			return -1;
		}

		fr = f_mount(NULL, drive_path, 1);
	}

	if (fr != FR_OK) return -1;

	fr = f_open(&TestFile, TEST_FILENAME, FA_CREATE_ALWAYS | FA_WRITE);

	if (fr != FR_OK) {
		f_mount(&QSPI_FatFs, drive_path, 0);
		return -1;
	}

	fr = f_write(&TestFile, TEST_STRING, strlen(TEST_STRING), &bytesWritten);
	if (fr != FR_OK) {
		f_close(&TestFile);
		f_mount(NULL, drive_path, 0);

		return -1;
	}

	if (bytesWritten != strlen(TEST_STRING)) {
		return -1;
	}

	fr = f_close(&TestFile);
	if (fr != FR_OK) {
		f_mount(NULL, drive_path, 0);
		return -1;
	}

	fr = f_open(&TestFile, TEST_FILENAME, FA_READ);
	if (fr != FR_OK) {
		f_mount(NULL, drive_path, 0);
		return -1;
	}

	BYTE read_buffer[256];
	memset(read_buffer, 0, sizeof(read_buffer));
	fr = f_read(&TestFile, read_buffer, READ_BUFFER_SIZE - 1, &bytesRead);
	if (fr != FR_OK) {
		f_close(&TestFile);
		f_mount(NULL, drive_path, 0);
		return -1;
	}

	fr = f_close(&TestFile);
	f_mount(NULL, drive_path, 0);

	if (fr != FR_OK) {
		return -1;
	}


	if (bytesRead != strlen(TEST_STRING)) {
		return -1;
	}

	if (memcmp(TEST_STRING, read_buffer, strlen(TEST_STRING)) != 0) {
		return -1;
	}

	return 0;
}


#endif /* INC_FATFSTEST_H_ */
