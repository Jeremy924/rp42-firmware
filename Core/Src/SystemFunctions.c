/*
 * SystemFunctions.c
 *
 *  Created on: Jul 14, 2025
 *      Author: Jerem
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_acm_if.h" // to turn on/off echo
#include "fatfs.h"
#include "SystemFunctions.h"
#include "CommandParser.h"
#include "version.h"
#include "usbd_msc_if.h" // to turn on/off msc
#include "FSUtils.h"
#include "main.h"
#include "quadspi.h"
#include "AppInfo.h"
#include "stdbool.h"

uint8_t hardware_unlocked = 0;

void clear_command_func(char* ) {
	printf("\033[2J\033[H");
	fflush(stdout);
}

void echo_command_func(char* args) {
	if (strcmp(args, "off") == 0) echo_on = 0;
	else echo_on = 1;
}

void bootmode_command_func(char* args) {
	if (args[0] == '\0') {
		printf("bootmode %u\n", systemConfigData.bootmode);
		return;
	}
	if (args[1] != '\0' || args[0] < '0' || args[0] > '9') {
		fputs("bootmode {code}\n", stderr);
		return;
	}

	uint8_t code = (uint8_t) (args[0] - '0');
	systemConfigData.bootmode = code;

	FRESULT result = update_system_config();
	if (result != FR_OK) {
		fputs(fresult_to_string(result), stderr);
		fputs("\n", stderr);
	} else printf("Bootmode saved\n");
}

void dms_command_func(char* args) {
	char* time =  args;
	if (*time == '\0') {
		fputs("dms {millis}\n", stderr);

		return;
	}

	char* end_ptr;
	uint32_t new_debounce_ms = strtol(time, &end_ptr, 10);
	if (new_debounce_ms > 9999) {
		fputs("Out of range\n", stderr);
		return;
	}

	systemConfigData.debounce_ms = new_debounce_ms;

	FRESULT result = update_system_config();
	if (result != FR_OK) fputs(fresult_to_string(result), stderr);
	else printf("Debounce time set to %u milliseconds\n", new_debounce_ms);
}

void hostname_command_func(char* args) {
	char* hostname = args;
	while (*hostname == ' ' && *hostname != '\0') hostname++;
	if (*hostname == '\0') {
		fputs("hostname {name}\n", stderr);
		return;
	}

	if (strlen(hostname) > 14) {
		fputs("Name too long\n", stderr);
		return;
	}

	strcpy(systemConfigData.device_name, hostname);

	FRESULT result = update_system_config();
	if (result != FR_OK) fputs(fresult_to_string(result), stderr);
}

void whoami_command_func(char* args) {
	printf("%s\n", systemConfigData.device_name);
}

void msc_command_func(char* args) {
	if (args[0] == '\0' || args[1] != '\0' || (args[0] != 'y' && args[0] != 'n')) {
		fputs("msc {y/n}\n", stderr);
		return;
	}

	g_msc_is_active = args[0] == 'y';
}

void version_command_func(char* args) {
	char* app_name = args;

	if (strcmp(app_name, "firmware") == 0) {
		printf("Firmware v. %u.%uH%u\n", systemConfigData.fm_major, systemConfigData.fm_minor, systemConfigData.hw_version);
	} else {
		if (args[0] == '\0') {
			fputs("version {app_name}\n", stderr);
			return;
		}
		struct AppInfo info;
		get_app_info(args, &info);

		if (info.app_name[0] == '\0') {
			fputs("App not found\n", stderr);
		} else {
			printf("%u.%u\n", (uint32_t) info.version_major, (uint32_t) info.version_minor);
		}
	}
}

void reset_command_func(char* args) {
	NVIC_SystemReset();
}

void poweroff_command_func(char* args) {
	Powerdown();
}

void mv_command_func(char* args) {
	if (args[0] == '\0') {
		fputs("mv {src} {dest}\n", stderr);
		return;
	}
	char* src = args;
	char* dest = args + 1;
	if (dest[0] != ' ') {
		fputs("mv {src} {dest}\n", stderr);
		return;
	}

	while (dest[0] == ' ' && dest[0] != '\0') dest++;
	if (dest[0] == '\0') {
		fputs("mv {src} {dest}\n", stderr);
		return;
	}

	FRESULT result = f_rename(src, dest);
	fputs(fresult_to_string(result), stderr);
}

void mkdir_command_func(char* args) {
	char* folder_name = args;
	FRESULT result = f_mkdir(folder_name);

	fputs(fresult_to_string(result), stderr);
}

void cat_command_func(char* args) {
	char* file = args;
	FIL f;

	FRESULT result = f_open(&f, file, FA_READ);

	if (result != FR_OK) {
		fputs(fresult_to_string(result), stderr);
		return;
	}

	char read_buf[11];

	unsigned int bytesRead = 10;
	while (bytesRead == 10) {
		result = f_read(&f, read_buf, 10, &bytesRead);
		if (result != FR_OK) {
			fputs(fresult_to_string(result), stderr);
			f_close(&f);
			return;
		}
		read_buf[bytesRead] = '\0';
		printf(read_buf);
	}

	f_close(&f);
	printf("\n");
}

void pwd_command_func(char* args) {
	char* path = (char*) malloc(64);

	FRESULT result = f_getcwd(path, 64);

	fputs(fresult_to_string(result), stderr);
	if (result == FR_OK) printf(path);

	free(path);
	printf("\n");
}

void rm_command_func(char* args) {
	char* path = args;

	FRESULT result = f_unlink(path);
	fputs(fresult_to_string(result), stderr);
}

void touch_command_func(char* args) {
	char* path = args;

	FIL f;
	FRESULT result = f_open(&f, path, FA_CREATE_NEW);

	if (result == FR_OK) result = f_close(&f);

	fputs(fresult_to_string(result), stderr);
}

void gclust_command_func(char* args) {
	char* path = args;

	FIL f;

	FRESULT fr = f_open(&f, path, FA_READ);

	if (fr != FR_OK) {
		fputs(fresult_to_string(fr), stderr);
		return;
	}

	uint32_t bytesRead;
	char buf;
	f_read(&f, &buf, 1, &bytesRead);

	printf("Cluster #: %lu\n", f.sect);

	f_close(&f);
}

void fexpand_command_func(char* args) {
	char* path = args;

	while (*args != ' ' && *args != '\0') args++;

	if (*args == '\0') {
		fputs("fexpand {path} {size}\n", stderr);
		return;
	}

	*args = '\0';
	args++;
	char* end_ptr;
	uint32_t size = strtol(args, &end_ptr, 10);

	if (end_ptr == args) {
		fputs("fexpand {path} {size}\n", stderr);
		return;
	}

	FIL f;
	FRESULT fr;

	fr = f_open(&f, path, FA_CREATE_ALWAYS | FA_WRITE);

	if (fr != FR_OK) {
		fputs(fresult_to_string(fr), stderr);
		return;
	}

	fr = f_expand(&f, size, 1); // 1 means require contiguous
	if (fr != FR_OK) {
		fputs(fresult_to_string(fr), stderr);
	}

    // 3. Set the logical file size to match the allocation
    fr = f_lseek(&f, size);
    if (fr != FR_OK) {
        fputs(fresult_to_string(fr), stderr);
        f_close(&f);
        return;
    }

    fr = f_truncate(&f);
    if (fr != FR_OK) {
        fputs(fresult_to_string(fr), stderr);
        f_close(&f);
        return;
    }

    f_close(&f);
}

void mkfs_command_func(char* args) {
	if (strcmp(args, "yes") != 0) {
		printf("Confirmation required\n");
		return;
	}

	uint8_t page = 0;
	uint8_t col = 0;
	printText("Making FS", &page, &col);
	FRESULT status = f_mkfs("0:", FM_ANY, 0, (uint8_t*) 0x10000000, 32768);

	fputs(fresult_to_string(status), stderr);

	if (status == FR_OK) printf("Created filesystem\n");
}

void rmdir_command_func(char* args) {
	char* folder = args;

	FRESULT result = f_rmdir(folder);
	fputs(fresult_to_string(result), stderr);
}

void lscpu_command_func(char* args) {
	printf("RP-42 H%u\nMCU: STM32L475RCT6\nCPU: Arm M4 Cortex\nMax Clock Speed: 80 MHz\nCurrent Clock Speed: %u MHz\nRAM: 128KB\nFLASH: 256KB\n", systemConfigData.hw_version, 80);
}

void unlock_command_func(char* args) {
	if (strcmp(args, "532326") != 0) {
		fputs("Security code required\n", stderr);
		return;
	}

	hardware_unlocked = 1;
	printf("Hardware unlocked\n");
}

void lock_command_func(char* args) {
	if (hardware_unlocked == 0) {
		fputs("Hardware already locked\n", stderr);
		return;
	}

	hardware_unlocked = 0;
	printf("Hardware locked\n");
}

void ls_command_func(char* args) {
	char* current_directory;

	if (args[0] == '\0') current_directory = ".";
	else current_directory = args;

	print_directory_contents(current_directory);
}

void cd_command_func(char* args) {
	if (args[0] == '\0') {
		fputs("cd {path}\n", stderr);
		return;
	}

	FRESULT result = f_chdir(args);
	fputs(fresult_to_string(result), stderr);
}

void pfirm_command_func(char* args) {
	if (!hardware_unlocked) {
		fputs("Hardware locked\n", stderr);
		return;
	}

	initialize_firmware_install();
}

void _flashrw_get_args(char* args, uint32_t* start_address, uint32_t* length) {
	char* endptr;

	*start_address = strtol(args, &endptr, 10);

	if (endptr == args) {
		fputs("flashr/w {start} {length}\n", stderr);
		return;
	}

	char* len_endptr;
	*length = strtol(endptr, &len_endptr, 10);
	if (endptr == len_endptr) {
		fputs("flashr/w {start} {length}\n", stderr);
		return;
	}
}

void flashr_command_func(char* args) {
	if (!hardware_unlocked) {
		fputs("Hardware locked\n", stderr);
		return;
	}

	uint32_t start;
	uint32_t length;
	_flashrw_get_args(args, &start, &length);

	uint8_t* buf = (uint8_t*) malloc(length);
	if (buf == NULL) {
		fputs("Not enough memory\n", stderr);
		return;
	}

	uint8_t result = CSP_QSPI_Read(buf, start, length);

	if (result != 0) {
		fputs("Failed to read flash\n", stderr);
		free(buf);
		return;
	}

	result = CDC_Transmit(0, buf, length);

	if (result != USBD_OK) {
		Error_Handler();
	}

	free(buf);
}

void flashw_command_func(char* args) {
	if (!hardware_unlocked) {
		fputs("Hardware locked\n", stderr);
		return;
	}

	uint32_t start;
	uint32_t length;
	_flashrw_get_args(args, &start, &length);

	uint8_t* buffer = (uint8_t*) malloc(length);
	if (buffer == NULL) {
		fputs("Not enough memory\n", stderr);
		return;
	}

	memset(buffer, 0, length);

	readfill_stdin(buffer, length);

	uint8_t result = CSP_QSPI_WriteMemory(buffer, start, length);

	if (result != 0) {
		fputs("Failed to write\n", stderr);
		free(buffer);
		return;
	}

	printf("done\n");
	free(buffer);
}

void flashec_command_func(char* args) {
	if (!hardware_unlocked) {
		fputs("Hardware locked\n", stderr);
		return;
	}
	 for (unsigned int start_address = 0; start_address < 0x100000; start_address += 0x10000) {
	 //for (unsigned int start_address = 0; start_address < 0x100000; start_address += 0x10000) {
		 CSP_QSPI_EraseSector(start_address);
		 set_progress(16 * (start_address / 0x10000));
	 }
	 set_progress(0);

	 WriteString("Done\n");
}

void flashef_command_func(char* args) {
	if (!hardware_unlocked) {
		fputs("Hardware locked\n", stderr);
		return;
	}

	 int progress = 0;
	 for (unsigned int start_address = 0x7c0000; start_address < 0x800000; start_address += 0x10000) {
		 CSP_QSPI_EraseSector(start_address);
		 set_progress(progress);
		 progress += 64;
	 }
	 set_progress(0);
	 WriteString("Done\n");
}

void flashstat_command_func(char* args) {
	if (!hardware_unlocked) {
		fputs("Hardware locked\n", stderr);
		return;
	}

	if (args[0] != '\0' && args[1] != '\0') {
		fputs("Bad format\n", stderr);
		return;
	}

	if (args[0] == '\0' || args[0] == '1') {
		printf("%d\n", (int) CSP_QSPI_Read_StatusRegister1());
	}

	if (args[0] == '\0' || args[0] == '2') {
		printf("%d\n", (int) CSP_QSPI_Read_StatusRegister2());
	}

	if (args[0] == '\0' || args[0] == '3') {
		printf("%d\n", (int) CSP_QSPI_Read_StatusRegister3());
	}

	if (args[0] > '3' || (args[0] != '\0' && args[0] < '0')) {
		fputs("Invalid register\n", stderr);
	}
}

void flashcheck_command_func(char* args) {
	if (!hardware_unlocked) {
		fputs("Hardware locked\n", stderr);
		return;
	}

	int32_t checksum = 0;
	CSP_QSPI_EnableMemoryMappedMode();

	uint8_t* address = 0x90000000;
	uint8_t* end = address + 0x100000;

	while (address != end) {
		checksum += *address;
		address++;
	}

	printf("%d\n", checksum);
	HAL_QSPI_Abort(&hqspi);
}

void alert_command_func(char* args) {
	char temp_buf[30];
	memset(temp_buf, 0, 30);
	while (Read(temp_buf, 30) == 0) continue;

	clearSegment(3, 0, 132);
	uint8_t page = 3;
	uint8_t col = 0;

	for (unsigned int i = 0; i < 30; i++) {
		if (temp_buf[i] == '\0' || temp_buf[i] == '\n')  {
			temp_buf[i] = '\0';
			break;
		}
	}

	printText(temp_buf, &page, &col);
}

void progress_command_func(char* args) {
	char* parse_end;
	long value = strtol(args, &parse_end, 10);

	if (*parse_end != '\0') {
		fputs("progress {length}\n", stderr);
		return;
	}

	if (value < 0 || value > 255) fputs("Bad value\n", stderr);
	else set_progress((uint8_t) value);
}

void applist_command_func(char* args) {
	FIL app_list_file;

	FRESULT result = f_open(&app_list_file, "/System/apps.dat", FA_READ);
	if (result != FR_OK) {
		fputs(fresult_to_string(result), stderr);
		return;
	}

	struct AppInfo appInfo;
	uint32_t bytesRead;

	uint8_t firmware_found = 0;
	do {
		result = f_read(&app_list_file, &appInfo, sizeof(struct AppInfo), &bytesRead);

		if (bytesRead != 0 && bytesRead < sizeof(struct AppInfo)) {
			fputs("App list corrupted\n", stderr);
			f_close(&app_list_file);
			return;
		}

		if (strcmp(appInfo.app_name, "firmware") == 0) firmware_found = 1;

		if (bytesRead != 0)
			printf("App Name: %s v. %u.%u\nApp Publisher: %s\nPublish Date: %u/%u/20%u\n\n", appInfo.app_name, appInfo.version_major, appInfo.version_minor, appInfo.app_publisher, appInfo.published_month, appInfo.published_day, appInfo.published_year);
	} while (bytesRead != 0);

	if (!firmware_found) {
		printf("App Name: firmware v. 0.12\nApp Publisher: Jeremy924\nPublish Date: 7/22/25\n\n");
	}

	f_close(&app_list_file);
}

void appinfo_delete_command_func(char* args) {
	if (!hardware_unlocked) {
		fputs("Hardware locked\n", stderr);
		return;
	}

	char app_name_to_delete[16];
	char app_publisher_to_delete[16];

	// --- 1. Parse Input Arguments ---
	// We only need the name and publisher to identify the entry for deletion.
	int result = sscanf(args, "%15s %15s", app_name_to_delete, app_publisher_to_delete);
	if (result != 2) {
		fputs("Usage: appinfo_delete {app_name} {app_publisher}\n", stderr);
		return;
	}

	FIL f_orig, f_temp;
	FRESULT fresult;
	const char* orig_path = "/System/apps.dat";
	const char* temp_path = "/System/apps.tmp";

	// --- 2. Open Original File for Reading ---
	fresult = f_open(&f_orig, orig_path, FA_READ);
	if (fresult != FR_OK) {
		fprintf(stderr, "Error: Cannot open original file '%s'.\n", orig_path);
		return;
	}

	// --- 3. Create Temporary File for Writing ---
	// FA_CREATE_ALWAYS will create a new file or overwrite an existing one.
	fresult = f_open(&f_temp, temp_path, FA_CREATE_ALWAYS | FA_WRITE);
	if (fresult != FR_OK) {
		fprintf(stderr, "Error: Cannot create temporary file '%s'.\n", temp_path);
		f_close(&f_orig); // Clean up the file we already opened
		return;
	}

	struct AppInfo current_app_info;
	UINT bytes_read, bytes_written;
	bool entry_found = false;

	// --- 4. Copy Records, Skipping the One to Delete ---
	while (f_read(&f_orig, &current_app_info, sizeof(struct AppInfo), &bytes_read) == FR_OK && bytes_read == sizeof(struct AppInfo)) {
		// Check if the current record is the one we want to delete
		if (strcmp(current_app_info.app_name, app_name_to_delete) == 0 &&
			strcmp(current_app_info.app_publisher, app_publisher_to_delete) == 0) {

			// Match found! Set the flag and *do not* write this record to the temp file.
			entry_found = true;
			printf("Found entry '%s' by '%s'. Marking for deletion.\n", app_name_to_delete, app_publisher_to_delete);

		} else {
			// This is not the record to delete, so write it to our temporary file.
			fresult = f_write(&f_temp, &current_app_info, sizeof(struct AppInfo), &bytes_written);
			if (fresult != FR_OK || bytes_written != sizeof(struct AppInfo)) {
				fprintf(stderr, "Error writing to temporary file. Aborting.\n");
				// Abort and clean up
				f_close(&f_orig);
				f_close(&f_temp);
				f_unlink(temp_path); // Delete the partially written temp file
				return;
			}
		}
	}

	// --- 5. Close Both Files ---
	f_close(&f_orig);
	f_close(&f_temp);

	// --- 6. Replace Original File with Temp File ---
	if (entry_found) {
		// First, delete the old original file.
		fresult = f_unlink(orig_path);
		if (fresult != FR_OK) {
			fprintf(stderr, "Error deleting original file. Manual cleanup of '%s' may be required.\n", temp_path);
			return;
		}

		// Second, rename the temporary file to become the new original.
		fresult = f_rename(temp_path, orig_path);
		if (fresult != FR_OK) {
			fprintf(stderr, "CRITICAL ERROR: Could not rename '%s' to '%s'. Data may be lost.\n", temp_path, orig_path);
			return;
		}
		printf("Successfully deleted entry and rebuilt the file.\n");
	} else {
		// If no entry was found, we don't need the temp file.
		printf("Entry '%s' by '%s' not found. No changes made.\n", app_name_to_delete, app_publisher_to_delete);
		f_unlink(temp_path); // Just delete the useless temp file.
	}
}

void appinfo_update_command_func(char* args) {
	if (!hardware_unlocked) {
		fputs("Hardware locked\n", stderr);
		return;
	}

	struct AppInfo new_app_info;
	unsigned int version_major;
	unsigned int version_minor;
	unsigned int publish_month;
	unsigned int publish_day;
	unsigned int publish_year;

	int result = sscanf(args, "%15s %u %u %15s %u %u %u",
	                    new_app_info.app_name,
	                    &version_major,
	                    &version_minor,
	                    new_app_info.app_publisher,
	                    &publish_month,
	                    &publish_day,
	                    &publish_year);
	if (result != 7) {
		fputs("appinfo_update {app_name} {version_major} {version_minor} {app_publisher} {publish_month} {publish_day} {publish_year}\n", stderr);
		return;
	}

	new_app_info.version_major   = (uint8_t) version_major;
	new_app_info.version_minor   = (uint8_t) version_minor;
	new_app_info.published_month = (uint8_t) publish_month;
	new_app_info.published_day   = (uint8_t) publish_day;
	new_app_info.published_year  = (uint8_t) publish_year;

    FIL f;
    FRESULT fresult;
    const char* dat_path = "/System/apps.dat";

    // --- 2. Open The File ---
    // We need to both read and write.
    // FA_OPEN_ALWAYS: Opens the file if it exists. If not, it creates a new one.
    // This is perfect for our use case.
    f_open(&f, dat_path, FA_CREATE_NEW);
    f_close(&f);
    fresult = f_open(&f, dat_path, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
    if (fresult != FR_OK) {
        fprintf(stderr, "Error opening file: %d\n", fresult);
        return;
    }

    struct AppInfo current_app_info;
    UINT bytes_read;
    bool entry_updated = false;
    DWORD file_pos = 0; // Keep track of our position in the file

    // --- 3. Search for an Existing Entry ---
    // Read the file one AppInfo struct at a time.
    while (f_read(&f, &current_app_info, sizeof(struct AppInfo), &bytes_read) == FR_OK && bytes_read == sizeof(struct AppInfo)) {

        // Check if the current record from the file matches our new app's name and publisher
        if (strcmp(current_app_info.app_name, new_app_info.app_name) == 0 &&
            strcmp(current_app_info.app_publisher, new_app_info.app_publisher) == 0) {

            // Move the file pointer *back* to the beginning of the record we just read.
            fresult = f_lseek(&f, file_pos);
            if (fresult != FR_OK) {
                fprintf(stderr, "Error seeking in file to update: %d\n", fresult);
                f_close(&f);
                return;
            }

            // Overwrite the old record with the new one.
            UINT bytes_written;
            fresult = f_write(&f, &new_app_info, sizeof(struct AppInfo), &bytes_written);
            if (fresult != FR_OK || bytes_written != sizeof(struct AppInfo)) {
                fprintf(stderr, "Error writing updated record: %d\n", fresult);
                f_close(&f);
                return;
            }

            entry_updated = true;
            break; // Exit the loop since we've found and updated our entry.
        }

        // Update our position keeper
        file_pos = f_tell(&f);
    }

    // --- 4b. No Match Found: Append to End ---
    // If we finished the loop and the 'entry_updated' flag is still false,
    // it means our app wasn't in the file. The file pointer is already at the end.
    if (!entry_updated) {
        UINT bytes_written;
        // Since f_lseek is not needed, we just write the new record.
        fresult = f_write(&f, &new_app_info, sizeof(struct AppInfo), &bytes_written);
        if (fresult != FR_OK || bytes_written != sizeof(struct AppInfo)) {
            fprintf(stderr, "Error appending new record: %d\n", fresult);
        }
    }

    // --- 5. Close the File ---
    // Always remember to close the file to save changes and release resources.
    f_close(&f);
}

const unsigned int TOTAL_FUNCTIONS = 33;
struct CLICommand clear_cmd    = { "clear", clear_command_func };
struct CLICommand echo_cmd     = { "echo", echo_command_func };
struct CLICommand bootmode_cmd = { "bootmode", bootmode_command_func };
struct CLICommand dms_cmd      = { "dms", dms_command_func };
struct CLICommand hostname_cmd = { "hostname", hostname_command_func };
struct CLICommand whoami_cmd   = { "whoami", whoami_command_func };
struct CLICommand msc_cmd      = { "msc", msc_command_func };
struct CLICommand version_cmd  = { "version", version_command_func };
struct CLICommand reset_cmd    = { "reset", reset_command_func };
struct CLICommand poweroff_cmd = { "poweroff", poweroff_command_func };
struct CLICommand mv_cmd       = { "mv", mv_command_func };
struct CLICommand mkdir_cmd    = { "mkdir", mkdir_command_func };
struct CLICommand cat_cmd      = { "cat", cat_command_func };
struct CLICommand pwd_cmd      = { "pwd", pwd_command_func };
struct CLICommand rm_cmd       = { "rm", rm_command_func };
struct CLICommand touch_cmd    = { "touch", touch_command_func };
struct CLICommand fexpand_cmd  = { "fexpand", fexpand_command_func };
struct CLICommand gclust_cmd   = { "gclust", gclust_command_func };
struct CLICommand mkfs_cmd     = { "mkfs", mkfs_command_func };
struct CLICommand rmdir_cmd    = { "rmdir", rmdir_command_func };
struct CLICommand lscpu_cmd    = { "lscpu", lscpu_command_func };
struct CLICommand unlock_cmd   = { "unlock", unlock_command_func };
struct CLICommand lock_cmd     = { "lock", lock_command_func };
struct CLICommand ls_cmd       = { "ls", ls_command_func };
struct CLICommand cd_cmd       = { "cd", cd_command_func };
struct CLICommand flashstat_cmd= { "flashstat", flashstat_command_func };
struct CLICommand flashr_cmd   = { "flashr", flashr_command_func };
struct CLICommand flashw_cmd   = { "flashw", flashw_command_func };
struct CLICommand flashec_cmd  = { "flashec", flashec_command_func };
struct CLICommand flashef_cmd  = { "flashef", flashef_command_func };
struct CLICommand check_cmd    = { "flashcheck", flashcheck_command_func };
struct CLICommand alert_cmd    = { "alert", alert_command_func };
struct CLICommand progress_cmd = { "progress", progress_command_func };
struct CLICommand applist_cmd  = { "applist", applist_command_func };
struct CLICommand appupdate_cmd= { "appinfo_update", appinfo_update_command_func };
struct CLICommand appdelete_cmd= { "appdelete", appinfo_delete_command_func };
struct CLICommand pfirm_cmd    = { "pfirm", pfirm_command_func };

void register_all_sys_functions() {

	hint_total_commands(TOTAL_FUNCTIONS);

	register_cli_command(&clear_cmd);
	register_cli_command(&echo_cmd);
	register_cli_command(&bootmode_cmd);
	register_cli_command(&dms_cmd);
	register_cli_command(&hostname_cmd);
	register_cli_command(&whoami_cmd);
	register_cli_command(&msc_cmd);
	register_cli_command(&version_cmd);
	register_cli_command(&reset_cmd);
	register_cli_command(&poweroff_cmd);
	register_cli_command(&mv_cmd);
	register_cli_command(&mkdir_cmd);
	register_cli_command(&cat_cmd);
	register_cli_command(&pwd_cmd);
	register_cli_command(&rm_cmd);
	register_cli_command(&touch_cmd);
	register_cli_command(&mkfs_cmd);
	register_cli_command(&rmdir_cmd);
	register_cli_command(&lscpu_cmd);
	register_cli_command(&unlock_cmd);
	register_cli_command(&lock_cmd);
	register_cli_command(&ls_cmd);
	register_cli_command(&cd_cmd);
	register_cli_command(&flashstat_cmd);
	register_cli_command(&flashr_cmd);
	register_cli_command(&flashw_cmd);
	register_cli_command(&flashec_cmd);
	register_cli_command(&flashef_cmd);
	register_cli_command(&check_cmd);
	register_cli_command(&alert_cmd);
	register_cli_command(&progress_cmd);
	register_cli_command(&appupdate_cmd);
	register_cli_command(&applist_cmd);
	register_cli_command(&appdelete_cmd);
	register_cli_command(&pfirm_cmd);
	register_cli_command(&fexpand_cmd);
	register_cli_command(&gclust_cmd);
}

FRESULT update_system_config() {
	FIL f;
	FRESULT result = f_open(&f, "/System/sys.dat", FA_WRITE | FA_CREATE_ALWAYS);

	if (result != FR_OK) return result;

	unsigned int bw;
	char* data = (char*) &systemConfigData;
	result = f_write(&f, data, sizeof(SystemConfigData), &bw);

	if (result != FR_OK) return result;

	result = f_close(&f);

	return result;
}


const char* fresult_to_string(FRESULT result) {
	switch (result) {
		case FR_OK:
			return "";
		case FR_DISK_ERR:
		case FR_NOT_READY:
		case FR_NO_FILESYSTEM:
		case FR_MKFS_ABORTED:
		case FR_TIMEOUT:
		case FR_NOT_ENOUGH_CORE:
			return "IO Error\n";
		case FR_INT_ERR:
			return "Bug detected (assertion failed)\n";
		case FR_NO_FILE: // maybe don't combine?
		case FR_NO_PATH:
			return "File not found\n";
		case FR_INVALID_NAME:
			return "Path not formatted correctly\n";
		case FR_DENIED:
		case FR_LOCKED:
			return "Access denied\n";
		case FR_EXIST:
			return "File exists\n";
		case FR_INVALID_PARAMETER:
		case FR_INVALID_OBJECT:
		case FR_NOT_ENABLED:
			return "Internal error\n";
		case FR_WRITE_PROTECTED:
			return "Write protection enabled\n";
		case FR_INVALID_DRIVE:
			return "Invalid drive\n";
		case FR_TOO_MANY_OPEN_FILES:
			return "Too many open files\n";
	}

	return "";
}
