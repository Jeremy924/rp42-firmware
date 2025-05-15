/*
 * FatFSTest.h
 *
 *  Created on: Apr 26, 2025
 *      Author: Jerem
 */


#include "ff.h"         // FatFs core definitions
#include "main.h"       // For HAL_Delay, printf (or your logging equivalent)
#include <string.h>     // For strlen, memcmp
#include <stdio.h>      // For printf (adapt if using a different logging method)

#include "FSUtils.h"

// --- Test Configuration ---
#define TEST_FILENAME "test3.txt"
#define TEST_STRING   "Hello world"
#define READ_BUFFER_SIZE 256 // Should be large enough for TEST_STRING + null terminator

#define LogMessage printf

/***
 * @brief Lists the contents of a directory.
 * @param path: Full path to the directory to list (e.g., "0:/", "0:/myfolder").
 * @retval FR_OK if successful, or an error code otherwise.
 */
FRESULT test_list_directory_contents(const char *path)
{
	FATFS QSPI_FatFs;
    FRESULT fr;         // FatFs function result code
    DIR dir;            // Directory object
    FILINFO fno;        // File information object
    char item_path[256]; // Buffer for constructing full item path for recursion (optional)

	const char* drive_path = "0:";

	fr = f_mount(&QSPI_FatFs, drive_path, 1);

    LogMessage("\r\nListing directory: %s\r\n", path);
    LogMessage("--------------------------------------------------\r\n");
    LogMessage("Type  | Size       | Attributes | Name\r\n");
    LogMessage("--------------------------------------------------\r\n");

    // --- 1. Open the directory ---
    fr = f_opendir(&dir, path);
    if (fr != FR_OK) {
        LogMessage("ERROR: Failed to open directory '%s'. Code: %d\r\n", path, fr);
        return fr;
    }

    // --- 2. Read directory entries ---
    for (;;) {
        fr = f_readdir(&dir, &fno); // Read one entry
        if (fr != FR_OK || fno.fname[0] == 0) {
            break; // Break on error or end of directory
        }

        // Print item type (Directory or File)
        if (fno.fattrib & AM_DIR) {
            LogMessage("DIR   |            | ");
        } else {
            LogMessage("FILE  | %10lu | ", fno.fsize); // Print file size
        }

        // Print attributes (R=ReadOnly, H=Hidden, S=System, A=Archive)
        LogMessage("%c%c%c%c       | ",
                   (fno.fattrib & AM_RDO) ? 'R' : '-',
                   (fno.fattrib & AM_HID) ? 'H' : '-',
                   (fno.fattrib & AM_SYS) ? 'S' : '-',
                   (fno.fattrib & AM_ARC) ? 'A' : '-');

        // Print the filename
        LogMessage("%s\r\n", fno.fname);

        // Optional: Recursive listing for subdirectories
        // Be careful with stack depth if you enable deep recursion
        /*
        if ((fno.fattrib & AM_DIR) && strcmp(fno.fname, ".") != 0 && strcmp(fno.fname, "..") != 0) {
            // Construct path for subdirectory
            if (strlen(path) + 1 + strlen(fno.fname) < sizeof(item_path) -1) { // -1 for null terminator
                sprintf(item_path, "%s/%s", path, fno.fname);
                // Ensure path separator is correct for FatFs ('/' is standard)
                // If path is "0:", avoid "0://subdir"
                if (strcmp(path, drive_path) == 0 && path[strlen(path)-1] == ':') {
                     sprintf(item_path, "%s%s", path, fno.fname); // e.g. "0:subdir"
                } else if (path[strlen(path)-1] == '/') {
                     sprintf(item_path, "%s%s", path, fno.fname); // e.g. "0:/mydir/subdir"
                } else {
                     sprintf(item_path, "%s/%s", path, fno.fname); // e.g. "0:/mydir/subdir"
                }
                list_directory_contents(item_path); // Recursive call
            } else {
                LogMessage("Path too long for recursive listing: %s/%s\r\n", path, fno.fname);
            }
        }
        */
    }

    // --- 3. Close the directory ---
    fr = f_closedir(&dir);
	f_mount(NULL, drive_path, 0);

	if (fr != FR_OK) {
        LogMessage("ERROR: Failed to close directory '%s'. Code: %d\r\n", path, fr);
        // Note: Even if close fails, we've read what we could.
    }

    LogMessage("--------------------------------------------------\r\n");
    if (fr == FR_OK) { // If loop broke due to end of dir, fr might still be FR_OK from last successful readdir
        return FR_OK;
    } else {
        return fr; // Return error from f_readdir if that's what broke the loop
    }
}

int FS_mount() {
	static FATFS QSPI_FatFs;
	FRESULT fr;
	FIL TestFile;
	UINT bytesWritten;
	UINT bytesRead;

	const char* drive_path = "0:";

	fr = f_mount(&QSPI_FatFs, drive_path, 1);

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

	return 0;
}

int FS_umount() {
	const char* drive_path = "0:";

	if (f_mount(NULL, drive_path, 0) != FR_OK) return -1;
	return 0;
}

/**
 * @brief Runs a basic test sequence for FatFs on the QSPI flash drive.
 * @retval 0 on success, -1 on failure.
 */
int run_fatfs_qspi_test(int code)
{
	FATFS QSPI_FatFs;
	FRESULT fr;
	FIL TestFile;
	UINT bytesWritten;
	UINT bytesRead;

	const char* drive_path = "0:";

	fr = f_mount(&QSPI_FatFs, drive_path, 1);

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

	if (code == 4) {
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
