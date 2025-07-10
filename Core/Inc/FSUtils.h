/*
 * FatFSTest.h
 *
 *  Created on: Apr 26, 2025
 *      Author: Jerem
 */

#ifndef INC_FSUTILS_H_
#define INC_FSUTILS_H_

#include "ff.h"         // FatFs core definitions
#include "main.h"       // For HAL_Delay, printf (or your logging equivalent)
#include <string.h>     // For strlen, memcmp
#include <stdio.h>      // For printf (adapt if using a different logging method)

// --- Test Configuration ---
#define TEST_FILENAME "test3.txt"
#define TEST_STRING   "Hello world"
#define READ_BUFFER_SIZE 256 // Should be large enough for TEST_STRING + null terminator

#define LogMessage printf

struct FileOpenParams {
	const char* filename;
	unsigned int flags;
	uint32_t handle;
};

struct FileRWParams {
	uint32_t handle;
	uint32_t bytesRW;
	char* buf;
	uint32_t len;
};

struct FileSeekParams {
	uint32_t handle;
	uint32_t offset;
	uint32_t dir;
};

struct FileRenameParams {
	const char* old_path;
	const char* new_path;
};

struct FileStat {
	const char* path;
	struct stat* stat;
};

struct FileHandleStat {
	int handle;
	struct stat* stat;
};

struct TextUIParams {
	char* text;
	uint8_t* page;
	uint8_t* col;
};

struct NextFileParams {
	char** file_name;
	uint8_t attributes;
};

struct FListParams {
	char* folder_name;
	char*** file_list;
};

struct FileSelectParams {
	char* result;
	char* starting_location;
	char* file_type;
	unsigned int size_of_result;
};

struct IconUIParams {
	uint8_t* bitmap;
	uint8_t width_bits;
	uint8_t height_bytes;
	uint8_t* page;
	uint8_t* col;
};

/***
 * @brief Lists the contents of a directory.
 * @param path: Full path to the directory to list (e.g., "0:/", "0:/myfolder").
 * @retval FR_OK if successful, or an error code otherwise.
 */
FRESULT print_directory_contents(const char *path);

/**
 * @brief Runs a basic test sequence for FatFs on the QSPI flash drive.
 * @retval 0 on success, -1 on failure.
 */
int run_fatfs_qspi_test(int code);


#endif /* INC_FSUTILS_H_ */
