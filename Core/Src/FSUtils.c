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

/**
 * @brief Lists all files in a given folder and returns the count.
 * @param folder: Null-terminated string of the folder path.
 * @param file_list: Pointer to a char** that will be allocated and filled with file names.
 * @return: Number of files found, or -1 on error.
 */
uint32_t FLIST(const char* folder, char*** file_list) {
    FRESULT res;
    DIR dir;
    FILINFO fno;
    uint32_t file_count = 0;
    char** list = NULL;
    uint32_t list_size = 10; // Initial allocation for 10 file names

    // Open the directory
    res = f_opendir(&dir, folder);
    if (res != FR_OK) {
        return -1; // Return -1 to indicate an error
    }

    // Allocate initial memory for the file list
    list = malloc(list_size * sizeof(char*));
    if (list == NULL) {
        f_closedir(&dir);
        return -1; // Memory allocation failed
    }

    while (1) {
        // Read a directory item
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) {
            break; // Break on error or end of directory
        }

        // Ignore directories
        if (fno.fattrib & AM_DIR) {
            continue;
        }

        // Check if we need to reallocate memory for the list
        if (file_count >= list_size) {
            list_size *= 2; // Double the size
            char** new_list = realloc(list, list_size * sizeof(char*));
            if (new_list == NULL) {
                // Free previously allocated memory before returning
                for (uint32_t i = 0; i < file_count; i++) {
                    free(list[i]);
                }
                free(list);
                f_closedir(&dir);
                return -1; // Reallocation failed
            }
            list = new_list;
        }

        // Allocate memory for the file name and copy it
        list[file_count] = malloc(strlen(fno.fname) + 1);
        if (list[file_count] == NULL) {
            // Free all allocated memory
             for (uint32_t i = 0; i < file_count; i++) {
                free(list[i]);
            }
            free(list);
            f_closedir(&dir);
            return -1; // Allocation for file name failed
        }
        strcpy(list[file_count], fno.fname);
        file_count++;
    }

    f_closedir(&dir);

    // Pass the allocated list back to the caller
    *file_list = list;

    return file_count;
}

/***
 * @brief Lists the contents of a directory.
 * @param path: Full path to the directory to list (e.g., "0:/", "0:/myfolder").
 * @retval FR_OK if successful, or an error code otherwise.
 */
FRESULT print_directory_contents(const char *path)
{
	FATFS QSPI_FatFs;
    FRESULT fr;         // FatFs function result code
    DIR dir;            // Directory object
    FILINFO fno;        // File information object
    char item_path[256]; // Buffer for constructing full item path for recursion (optional)

    // --- 1. Open the directory ---
    fr = f_opendir(&dir, path);
    if (fr != FR_OK) {
        fputs("ERROR: Failed to open directory\n", stderr);
        return fr;
    }

    LogMessage("\e[4;37mType  |Size        |Attributes  |Name                     |\e[0m\n");

    // --- 2. Read directory entries ---
    for (;;) {
        fr = f_readdir(&dir, &fno); // Read one entry
        if (fr != FR_OK || fno.fname[0] == 0) {
            break; // Break on error or end of directory
        }

        if ((fno.fattrib & AM_HID != 0) || (fno.fattrib & AM_SYS) != 0) continue;

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

	if (fr != FR_OK) {
        LogMessage("ERROR: Failed to close directory '%s'. Code: %d\r\n", path, fr);
        // Note: Even if close fails, we've read what we could.
    }

    if (fr == FR_OK) { // If loop broke due to end of dir, fr might still be FR_OK from last successful readdir
        return FR_OK;
    } else {
        return fr; // Return error from f_readdir if that's what broke the loop
    }
}

BYTE fatfs_work_buffer[512];
int FS_mount() {
	static FATFS QSPI_FatFs;
	FRESULT fr;
	FIL TestFile;
	UINT bytesWritten;
	UINT bytesRead;

	const char* drive_path = "0:";

	fr = f_mount(&QSPI_FatFs, drive_path, 1);

	if (fr == FR_NO_FILESYSTEM) {
		uint8_t page = 2;
		uint8_t col = 0;
		printText("Filesystem corrupt. See", &page, &col);
		page = 3;
		col = 0;
		printText("Jeremy924.github.io/e000", &page, &col);

		while (1) {
		    HAL_PWREx_EnterSTOP2Mode(PWR_STOPENTRY_WFI);
		}

		fr = f_mkfs(drive_path, FM_ANY, 0, fatfs_work_buffer, sizeof(fatfs_work_buffer));

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
