#include "COMInterface.h"
//#include "usbd_cdc_if.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "n25q128a.h"
#include "quadspi.h"
#include "usbd_cdc_acm_if.h"
#include "FSUtils.h"
#include "ProgressBar.h"
#include "usb_device.h"
#include "spi.h"
#include "fatfs.h"
#include "ff.h"
#include "usbd_msc_if.h"
#include "CommandParser.h"
#include "SystemFunctions.h"

uint32_t ReadFill(uint8_t* buf, uint32_t capacity);

// returns 0 if success and non-zero if fail
uint8_t ReadFillCallback(uint8_t* buf, uint32_t length) {
	while (ReadFill(buf, length) == 0) HAL_Delay(10);
	return 0;
}

// returns 0 if success and non-zero if fail
uint8_t ReadCallback(uint8_t* buf, uint32_t capacity) {
	uint32_t length;
	do {
		uint8_t key = system_call(0x0001, 0);
		while (key != 254) {
			if (key == 33) {
				systemConfigData.bootmode = 0;
				update_system_config();
				NVIC_SystemReset();
			}

			key = system_call(0x0001, 0);
		}

		length = Read(buf, capacity);
		if (length == 0) HAL_Delay(10);
	} while (length == 0);

	buf[length - 1] = '\0';

	return 0;
}

void run_console(USBD_HandleTypeDef* hUsbDeviceFS) {

#define MAIN_MENU 0
#define FLASH_MENU 1
#define HAL_MENU 2
#define LCD_MENU 3
#define BAR_UPDATE_FREQ 5
#define START_ERR       WriteString("\033[31m");
#define END_ERR         WriteString("\033[0m\n");


	uint8_t current_menu = MAIN_MENU;

	uint32_t program_address = 0;
	uint8_t* program_buf = malloc(256);
	uint8_t continuous_bar_last = 0;
	unsigned int last_bar_update = 0;
	bool show_continuous_bar = false;

	char command[100] = { 0 };

	struct CLICall call_obj;

	readfill_stdin = ReadFillCallback;
	read_stdin     = ReadCallback;
	register_all_sys_functions();

	while (1) {
		exec_shell();
	}

	while (1) {
		if (hUsbDeviceFS->dev_state == USBD_STATE_CONFIGURED) {
			char* current_directory = command;
			FRESULT success = f_getcwd(command, sizeof(command));

			if (success != FR_OK) {
				fputs("System error\n", stderr);
			}

			printf("\e[0;32m%s:%s%s$\e[0m ", systemConfigData.device_name, current_directory, (current_menu != FLASH_MENU) ? "" : "~frw");
			fflush(stdout);
		}

		int size;
		do {
			size = Read(command, 100);
			if (show_continuous_bar) {
				if (HAL_GetTick() - last_bar_update > BAR_UPDATE_FREQ) {
					continuous_progress_bar(&continuous_bar_last);
					last_bar_update = HAL_GetTick();
				}
			}
		} while (size == 0);
		//command[0] = 'h';
		//command[1] = 'i';

		//unsigned char temp[50] = { 0 };
		//sprintf(temp, "\r\nsize: %d\r\n", size);
		//CDC_Transmit_FS(temp, sizeof(temp));

		command[size - 1] = '\0';

		uint8_t result = parse_command(command, &call_obj);

		if (result == 2) fputs("Command not found\n", stderr);
		if (result == 0) call_obj.command->function(call_obj.arguments);

		continue;

		if (current_menu == MAIN_MENU) {
			if (strcmp("clear", command) == 0) { //done
				printf("\033[2J\033[H");
				fflush(stdout);
			} else if (strlen(command) > 4 && command[0] == 'c' && command[1] == 'd' && command[2] == ' ') {
				char* folder;
				for (folder = command + 3; *folder != '\0'; folder++) {
					if (*folder != ' ') break;
				}

				FRESULT result = f_chdir(folder);
				if (result == FR_DENIED) fputs("Access denied\n", stderr);
				else if (result == FR_DISK_ERR) fputs("IO Error\n", stderr);
				else if (result == FR_NO_PATH) fputs("Folder does not exist\n", stderr);

			} else if (strcmp(command, "poweroff") == 0) { //done
				Powerdown();
			} else if (memcmp(command, "mv", 2) == 0) {//done
				char* src = command + 3;
				while (*src == ' ' && *src != '\0') src++;
				if (*src == '\0') {
					fputs("Bad format\n", stderr);
					continue;
				}

				char* dest = src;
				while (*dest != ' ' && *dest != '\0') dest++;

				if (*dest == '\0') {
					fputs("Bad format\n", stderr);
					continue;
				}

				*dest = '\0';
				dest++;
				while (*dest == ' ' && *dest != '\0') *dest++;
				if (*dest == '\0') {
					fputs("Bad format\n", stderr);
					continue;
				}

				f_rename(src, dest);
			} else if (memcmp(command, "msc", 3) == 0) {//done
				if (strlen(command) != 4) {
					fputs("Command not found\n", stderr);
					continue;
				}

				if (command[3] == 'y') g_msc_is_active = 1;
				else if (command[3] == 'n') g_msc_is_active = 0;
				else {
					fputs("Bad format\n", stderr);
				}
			} else if (memcmp(command, "bootmode", 8) == 0) {//done
				if (strlen(command) < 10 || command[8] != ' ' || command[9] > '9' || command[9] < '0') {
					fputs("Bad boot code\n", stderr);
					continue;
				}

				uint8_t code = (uint8_t) (command[9] - '0');
				systemConfigData.bootmode = code;

				update_system_config();
			}
			else if (memcmp(command, "dms", 3) == 0) {//done
				char* time = command + 4;
				while (*time == ' ' && *time != '\0') time++;
				if (*time == '\0') {
					fputs("Bad format\n", stderr);
					continue;
				}

				char* end_ptr;
				uint32_t new_debounce_ms = strtol(time, &end_ptr, 10);
				if (new_debounce_ms > 9999) {
					fputs("Out of range\n", stderr);
					continue;
				}

				systemConfigData.debounce_ms = new_debounce_ms;

				update_system_config();
				printf("Debounce time set to %d ms\n", new_debounce_ms);
			} else if (memcmp(command, "hostname", 11) == 0) {//done
				char* hostname = command + sizeof("hostname");
				while (*hostname == ' ' && *hostname != '\0') hostname++;
				if (*hostname == '\0') {
					fputs("Bad format\n", stderr);
					continue;
				}

				if (strlen(hostname) > 14) {
					fputs("Name too long\n", stderr);
					continue;
				}

				strcpy(systemConfigData.device_name, hostname);

				FIL f;
				FRESULT result = f_open(&f, "/System/sys.dat", FA_WRITE | FA_CREATE_ALWAYS);

				unsigned int bw;
				char* data = (char*) &systemConfigData;
				result = f_write(&f, data, sizeof(SystemConfigData), &bw);
				f_close(&f);
			}
			else if (memcmp(command, "mkdir", 5) == 0) { // done
				char* folder_name = command + 6;
				while (*folder_name == ' ' && *folder_name != '\0') folder_name++;

				if (*folder_name == '\0') {
					fputs("Bad format\n", stderr);
				} else {
					f_mkdir(folder_name);
				}
			}
			else if (command[0] == 'c' && command[1] == 'a' && command[2] == 't') { //done
				char* file = command + 4;
				while (*file == ' ' && *file != '\0') file++;
				if (*file == '\0') {
					fputs("Bad format\n", stderr);
				} else {
					FIL f;
					FRESULT success = f_open(&f, file, FA_READ);
					if (success != FR_OK) {
						fputs("Could not open file\n", stderr);
						END_ERR;
					} else {
						char buf[11];
						unsigned int bytesRead = sizeof(buf) - 1;
						while (bytesRead == sizeof(buf) - 1) {
							if (f_read(&f, buf, sizeof(buf) - 1, &bytesRead) != FR_OK) {
								fputs("Failed to read", stderr);
							} else {
								buf[bytesRead] = 0;
								printf(buf);
							}

						}
					}

					puts("\n");
				}
			}
			else if (strcmp("reset", command) == 0) { //done
				NVIC_SystemReset();
			} else if (memcmp("pwd", command, 3) == 0) { //done
				f_getcwd(command, 100);
				printf("%s\n", command);
			} else if (memcmp("rm ", command, 3) == 0) {
				char* file_start = command + 3;
				while (*file_start != '\0' && *file_start == ' ') file_start++;
				if (*file_start == '\0') fputs("Bad format\n", stderr);
				else if (f_unlink(file_start) != FR_OK)
					fputs("Failed to delete\n", stderr);
			} else if (memcmp("touch", command, 5) == 0) {
				char* file_start = command + 6;
				while (*file_start != '\0' && *file_start == ' ') file_start++;
				if (*file_start == '\0') fputs("Bad format\n", stderr);
				else {
					FIL fp;
					if (f_open(&fp, file_start, FA_CREATE_NEW) != FR_OK)
						fputs("Failed to create file\n", stderr);
					else if (f_close(&fp) != FR_OK)
						fputs("Failed to create file\n", stderr);
				}
			} else if (memcmp("wf", command, 2) == 0) {
				char* file_start = command + 3;
				while (*file_start != '\0' && *file_start == ' ') file_start++;
				if (*file_start == '\0') fputs("Invalid format\n", stderr);
				else {
					uint32_t length = 0;
					char* length_str = file_start + 1;
					while (*length_str != '\0' && *length_str != ' ') length_str++;
					if (*length_str != '\0') {
						*length_str = '\0';
						length_str++;
					}
					while (*length_str != '\0' && *length_str == ' ') length_str++;
					if (*length_str == '\0') {
						fputs("Bad format\n", stderr);
						continue;
					}

					char* end_ptr;
					length = strtol(length_str, &end_ptr, 10);
					if (*end_ptr != '\0') {
						fputs("Bad format\n", stderr);
						continue;
					}

					if (length > sizeof(command)) {
						fputs("Too big\n", stderr);
						continue;
					}

					FIL fp;
					if (f_open(&fp, file_start, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
						fputs("Failed to create/open file\n", stderr);
					else {
						UINT bytes_written;
						uint32_t bytes_to_write = 0;
						do {
							bytes_to_write = ReadFill(command, length);
							HAL_Delay(10);
						} while (bytes_to_write == 0);
						f_write(&fp, command, bytes_to_write, &bytes_written);
						f_close(&fp);

						printf("Wrote %u bytes\n", bytes_written);
					}
				}
			} else if (memcmp("expand", command, 6) == 0) {
				char file_name[32];
				unsigned int size;

				sscanf(command, "expand %s %u", file_name, &size);

				FIL f;
				if (f_open(&f, file_name, FA_WRITE|FA_CREATE_ALWAYS) != FR_OK)
					fputs("Failed to open file\n", stderr);
				else {
					if (f_expand(&f, size, 0) != FR_OK)
						fputs("Failed to expand file\n", stderr);
					else printf("Expanded %s to %d bytes\n", file_name, size);

					f_close(&f);

				}
			} else if (strcmp("mkfs", command) == 0) { //done
				FRESULT status = f_mkfs("0:", FM_ANY, 0, (uint8_t*) 0x10000000, 32768);

				if (status != FR_OK) fputs("Failed to create file system\n", stderr);
				else printf("Created filesystem\n");
			}
			else if (memcmp("rmdir", command, 5) == 0) { // done
				char* folder_start = command + 6;
				while (*folder_start != '\0' && *folder_start == ' ') folder_start++;
				if (*folder_start == '\0') fputs("Invalid format", stderr);
				else if (f_rmdir(folder_start) != FR_OK)
					fputs("Failed to delete directory\n", stderr);
			}
			else if (strcmp("whoami", command) == 0) { // done
				printf("%s\n", systemConfigData.device_name);
			} else if (strcmp("lscpu", command) == 0) { // done
				printf("RP42 0.0.0.4-b\n\tMCU: STM32L475RCT6\n\tCPU: ARM M4 Cortex\n\tMax Clock Speed: 80MHZ\n\tRAM: 128KB\n\tFLASH: 256KB\n");
			} else if (strcmp("lsblk", command) == 0) {//done
				printf("NAME\tRM\tSIZE\tRO\tTYPE\tMOUNTPOINTS\nsda\t0\t256K\t1\tFLASH\nsdb\t0\t8M\t0\tFLASH\n>");
			} else if (strcmp("frw", command) == 0) {
				printf("\033[32mFlash Read/Write Tool\033[0m\n");
				current_menu = FLASH_MENU;
			} else if (strcmp("echo off", command) == 0) { //done
				echo_on = 0;
				//COM_STATUS &= ~ECHO;
			} else if (strcmp("echo on", command) == 0) {//done
				echo_on = 1;
				//COM_STATUS |= ECHO;
			} else if (strcmp("sudo hal-config", command) == 0) {//done
				current_menu = HAL_MENU;
			//} else if (strcmp("whereami", command) == 0) {
			//	_WriteString("Running from FLASH!!!!", 1);
			} else if (memcmp(command, "ls", 2) == 0) {//done
				char* current_directory;

				if (strlen(command) == 2) {
					current_directory = command;
					FRESULT success = f_getcwd(command, sizeof(command));
				} else {
					current_directory = command + 3;
					while (*current_directory != '\0' && *current_directory == ' ') current_directory++;
				}

				print_directory_contents(current_directory);
			}
			else if (memcmp("version", command, strlen("version")) == 0) {//done
				char* app_name = command + strlen("version");
				if (*app_name != ' ') {
					fputs("Bad format\n", stderr);
					continue;
				}

				while (*app_name != '\0' && *app_name == ' ') app_name++;
				if (*app_name == '\0') {
					fputs("Bad format\n", stderr);
					continue;
				}

				if (strcmp(app_name, "firmware") == 0)
					printf("%u.%u.%u\n", systemConfigData.fm_major, systemConfigData.fm_minor, systemConfigData.hw_version);
				else fputs("App not found\n", stderr);
			}
			else {
				fputs("Command not found\n", stderr);
			}
		} else if (current_menu == FLASH_MENU) {
			if (strcmp(command, "exit") == 0) {//skip
				current_menu = MAIN_MENU;
			} else if(strcmp(command, "status1") == 0) {//done
				uint8_t status = CSP_QSPI_Read_StatusRegister1();

				printf("%d\n", (int) status);
			} else if(strcmp(command, "status2") == 0) {//done
				uint8_t status = CSP_QSPI_Read_StatusRegister2();
				printf("%d\n", (int) status);
			} else if(strcmp(command, "status3") == 0) {//done
				uint8_t status = CSP_QSPI_Read_StatusRegister3();
				printf("%d\n", (int) status);
			}  else if (strcmp(command, "pfirm") == 0) { //done
				initialize_firmware_install();
			} else if (strcmp(command, "test") == 0) { //skip
				CSP_QSPI_EnableMemoryMappedMode();

				uint32_t original[100];
				uint32_t* read = (uint32_t*) 0x90000000;
				uint8_t pass = 1;

				for (int i = 0; i < 100; i++) original[i] = read[i];

				for (int i = 0; i < 100000; i++) {
					if (i % 1000 == 0) {
						printf("% 2d%% complete\r", i / 1000);
					}
					for (int j = 0; j < 100; j++)
						if (original[j] != read[j]) {
							printf("Failed %d\n", i);

							pass = 0;
							break;
						}

					if (pass == 0) break;
				}

				if (pass == 1) printf("Passed");
			} else if (strcmp(command, "PWR ON") == 0) { //skip
				HAL_GPIO_WritePin(PWR_PERPH_GPIO_Port, PWR_PERPH_Pin, SET);
				printf("\n");
			} else if (strcmp(command, "PWR OFF") == 0) { //skip
				HAL_GPIO_WritePin(PWR_PERPH_GPIO_Port, PWR_PERPH_Pin, RESET);
				printf("\n");
			} else if (strcmp(command, "BAR_SHOW") == 0) {
				init_progress_bar();
			} else if (strcmp(command, "SHOW_CONT") == 0) {
				show_continuous_bar = true;
			} else if (strcmp(command, "HIDE_CONT") == 0) {
				show_continuous_bar = false;
			}
			else if (size >= 2 && command[0] == 'B' && command[1] == 'M') {
				if (strcmp(strtok(command, " "), "BM") != 0) {
					fputs("Invalid Format\n", stderr);
					continue;
				}
				char* token = strtok(NULL, " ");
				char* parse_end;
				long value = strtol(token, &parse_end, 10);
				if (*parse_end != '\0') {
					fputs("Invalid Format\n", stderr);
					continue;
				}

				if (value < 0 || value > 255) fputs("Bad value\n", stderr);
				else set_progress((uint8_t) value);
			}
			else if (size >= 2 && command[0] == 'R' && command[1] == 'S') {
				if (strcmp(strtok(command, " "), "RS") != 0) {
					fputs("Invalid format\n", stderr);
					continue;
				}
				char* token = strtok(NULL, " ");
				char* parse_end;
				long read_start = strtol(token, &parse_end, 10);
				if (*parse_end != '\0') {
					fputs("Invalid format\n", stderr);
					continue;
				}

				token = strtok(NULL, " ");
				long read_size = strtol(token, &parse_end, 10);
				if (*parse_end != '\0') {
					fputs("Invalid format\n", stderr);
					continue;
				}

				//uint8_t CSP_QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)

				if (read_start < 0 || read_start >= N25Q128A_FLASH_SIZE) {
					fputs("Invalid start\n", stderr);
					continue;
				}

				if (read_size < 0 || read_start + read_size >= N25Q128A_FLASH_SIZE) {
					fputs("Invalid size\n", stderr);
					continue;
				}

				char* buf = (char*) malloc(read_size);
				if (buf == NULL) {
					fputs("Out of memory\n", stderr);
					continue;
				}

				for (size_t i = 0; i < read_size; i++) buf[i] = 0;

				uint8_t status = CSP_QSPI_Read(buf, read_start, read_size);

				if (status != HAL_OK) {
					printf("%d", (int) status);
				} else {
					for (size_t i = 0; i < read_size; i++) {
						printf("%d: %d\n", (int) (read_start + i), (int) buf[i]);
					}
				}

				free(buf);
			} else if (command[0] == 'R') {
				if (strcmp(strtok(command, " "), "R") != 0) {
					_WriteString("\033[31mInvalid Format\033[0m\n", 1);
					continue;
				}
				char* token = strtok(NULL, " ");
				char* parse_end;
				long read_start = strtol(token, &parse_end, 10);
				if (*parse_end != '\0') {
					_WriteString("\033[31mInvalid Format\033[0m\n", 1);
					continue;
				}

				token = strtok(NULL, " ");
				long read_size = strtol(token, &parse_end, 10);
				if (*parse_end != '\0') {
					_WriteString("\033[31mInvalid Format\033[0m\n", 1);
					continue;
				}

				//uint8_t CSP_QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)

				if (read_start < 0 || read_start >= N25Q128A_FLASH_SIZE) {
					_WriteString("\033[31mInvalid Start\033[0m\n", 1);
					continue;
				}

				if (read_size < 0 || read_start + read_size >= N25Q128A_FLASH_SIZE) {
					_WriteString("\033[31mInvalid Size\033[0m\n", 1);
					continue;
				}


				char* buf = (char*) malloc(read_size);
				if (buf == NULL) {
					_WriteString("\033[31mOut of Memory\033[0m\n", 1);
					continue;
				}

				uint8_t status = CSP_QSPI_Read(buf, read_start, read_size);

				if (status != HAL_OK) {
					char num_parse[10];
					sprintf(num_parse, "%d", (int) status);
					_WriteString(num_parse, 1);
				} else {
					//CDC_Transmit_FS(buf, read_size);
					CDC_Transmit(0, buf, read_size);
				}
				WriteString("\n");

				free(buf);
			}
			else if (size >= 2 && command[0] == 'W' && command[1] == 'S') {
//uint8_t CSP_QSPI_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
				if (strcmp(strtok(command, " "), "WS") != 0) {
					WriteString("\033[31mInvalid Format\033[0m\n");
					continue;
				}
				char* token = strtok(NULL, " ");
				char* parse_end;
				long write_start = strtol(token, &parse_end, 10);
				if (*parse_end != '\0') {
					WriteString("\033[31mInvalid Format\033[0m\n");
					continue;
				}

				token = strtok(NULL, " ");
				long write_size = strtol(token, &parse_end, 10);
				if (token == '\0' || *parse_end != '\0') {
					WriteString("\033[31mInvalid Format\033[0m\n");
					continue;
				}

				//uint8_t CSP_QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)

				if (write_start < 0 || write_start >= N25Q128A_FLASH_SIZE) {
					WriteString("\033[31mInvalid Start\033[0m\n");
					continue;
				}

				if (write_size < 0 || write_start + write_size >= N25Q128A_FLASH_SIZE) {
					WriteString("\033[31mInvalid Size\033[0m\n");
					continue;
				}

				char* write_buf = (char*) malloc(write_size);
				if (write_buf == NULL) {
					WriteString("\033[31mOut of Memory\033[0m\n");
					continue;
				}

				while (Read(write_buf, write_size+1) == 0) HAL_Delay(10);

				for (size_t i = 0; i < write_size; i++) {
					char* write_value;
					if (i == 0) write_value = strtok(write_buf, " ");
					else write_value = strtok(NULL, " ");

					char* end_ptr;
					write_buf[i] = (char) strtol(write_value, &end_ptr, 10);
					if (*end_ptr != '\0') {
						WriteString("\033[31mInvalid Value\033[0m\n");
						free(write_buf);
						write_buf = NULL;
						break;
					}
				}

				if (write_buf != NULL) {
					if (CSP_QSPI_WriteMemory(write_buf, write_start, write_size) != HAL_OK) {
						WriteString("\033[31mFailed to write to FLASH\033[0m\n");
						free(write_buf);
					}
					WriteString("\nWrote\n");
					for (size_t i = 0; i < write_size; i++) {
						char num_parse[20];
						sprintf(num_parse, "%d: %d\n", (int) (write_start + i), (int) write_buf[i]);
						WriteString(num_parse);
					}

					free(write_buf);
				}
			}
			else if (size >= 2 && command[0] == 'W' && command[1] == 'H') {
			//uint8_t CSP_QSPI_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
							if (strcmp(strtok(command, " "), "WH") != 0) {
								WriteString("\033[31mInvalid Format\033[0m\n");
								continue;
							}
							char* token = strtok(NULL, " ");
							char* parse_end;
							long write_start = strtol(token, &parse_end, 10);
							if (*parse_end != '\0') {
								WriteString("\033[31mInvalid Format\033[0m\n");
								continue;
							}

							token = strtok(NULL, " ");
							long write_size = strtol(token, &parse_end, 10);
							if (*parse_end != '\0') {
								WriteString("\033[31mInvalid Format\033[0m\n");
								continue;
							}

							//uint8_t CSP_QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)

							if (write_start < 0 || write_start >= N25Q128A_FLASH_SIZE) {
								WriteString("\033[31mInvalid Start\033[0m\n");
								continue;
							}

							if (write_size < 0 || write_start + write_size >= N25Q128A_FLASH_SIZE) {
								WriteString("\033[31mInvalid Size\033[0m\n");
								continue;
							}

							char* write_buf = (char*) malloc(write_size*2);
							if (write_buf == NULL) {
								WriteString("\033[31mOut of Memory\033[0m\n");
								continue;
							}


							while (ReadFill(write_buf, write_size*2) == 0) HAL_Delay(10);

							for (size_t i = 0; i < write_size; i++) {
								char copy[] = {write_buf[i*2], write_buf[i*2+1], '\0'};
								char* end;
								long value = strtol(copy, &end, 16);
								if (*end != '\0') {
									WriteString("\033[31mBad value\033[0m\n");
									break;
								} else {
									write_buf[i] = value;
								}
							}

							if (write_buf != NULL) {
								if (CSP_QSPI_WriteMemory(write_buf, write_start, write_size) != HAL_OK) {
									WriteString("\033[31mFailed to write to FLASH\033[0m\n");
									free(write_buf);
								}

								free(write_buf);
							}
						}
			else if (command[0] == 'C') {
				int32_t checksum = 0;
				CSP_QSPI_EnableMemoryMappedMode();

				uint8_t* address = 0x90000000;
				uint8_t* end = address + 0x100000;

				while (address != end) {
					checksum += *address;
					address++;
				}

				char buf[30];
				sprintf(buf, "%d\n", checksum);
				WriteString(buf);
				HAL_QSPI_Abort(&hqspi);
			} else if (strcmp(command, "txt") == 0) {
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
			 else if (command[0] == 'W') {
			//uint8_t CSP_QSPI_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
							if (strcmp(strtok(command, " "), "W") != 0) {
								WriteString("\033[31mInvalid Format\033[0m\n");
								continue;
							}
							char* token = strtok(NULL, " ");
							char* parse_end;
							long write_start = strtol(token, &parse_end, 10);
							if (*parse_end != '\0') {
								WriteString("\033[31mInvalid Format\033[0m\n");
								continue;
							}

							token = strtok(NULL, " ");
							long write_size = strtol(token, &parse_end, 10);
							if (*parse_end != '\0') {
								WriteString("\033[31mInvalid Format\033[0m\n");
								continue;
							}

							//uint8_t CSP_QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)

							if (write_start < 0 || write_start >= N25Q128A_FLASH_SIZE) {
								WriteString("\033[31mInvalid Start\033[0m\n");
								continue;
							}

							if (write_size < 0 || write_start + write_size >= N25Q128A_FLASH_SIZE) {
								WriteString("\033[31mInvalid Size\033[0m\n");
								continue;
							}
							static char __attribute__((section(".shared"))) write_value_buf[1024];
							//char* write_value_buf = malloc(write_size+1);
							for (size_t i = 0; i < write_size; i++) write_value_buf[i] = 0;
							if (write_value_buf == NULL) {
								WriteString("\033[31mOut of Memory\033[0m\n");
							}

							while (ReadFill(write_value_buf, write_size) == 0) HAL_Delay(10);

							//for (int z = 0; z < write_size; z++) {
							//	CSP_QSPI_WriteMemory(write_value_buf+z, write_starAAAAAAAAt+z, 1);
							//}
							if (CSP_QSPI_WriteMemory(write_value_buf, write_start, write_size) != HAL_OK)
								_WriteString("Error: failed to read", 1);
							else _WriteString("done", 1);

							//free(write_value_buf);
						}
			 else if (strcmp(command, "ef") == 0) {
				 int progress = 0;
				 for (unsigned int start_address = 0x7c0000; start_address < 0x800000; start_address += 0x10000) {
					 CSP_QSPI_EraseSector(start_address);
					 set_progress(progress);
					 progress += 64;
				 }
				 set_progress(0);
				 WriteString("Done\n");
			 }
			 else if (strcmp(command, "ec") == 0) {
				 for (unsigned int start_address = 0; start_address < 0x100000; start_address += 0x10000) {
					 CSP_QSPI_EraseSector(start_address);
					 set_progress(16 * (start_address / 0x10000));
				 }
				 set_progress(0);

				 WriteString("Done\n");
				 //CSP_QSPI_Erase_Chip();
			 }
			else if (strcmp(command, "we") == 0) {
				QSPI_WriteEnable();
				WriteString("Write Enable flag set\n");
			}
			else {
				fputs("Command not found\n", stderr);
			}
		}
	}
}

/**
 * @brief Reads exactly 'capacity' bytes from the CDC RX ring buffer.
 * @param buf Pointer to the destination buffer.
 * @param capacity The exact number of bytes to read.
 * @retval Number of bytes read (either 'capacity' or 0).
 */
uint32_t ReadFill(uint8_t* buf, uint32_t capacity) {
    if (capacity == 0) {
        return 0;
    }

    uint8_t *local_write_ptr;
    uint8_t *local_read_ptr;
    uint32_t current_size_in_buffer;

    // Temporarily disable interrupts or use other atomic means
    // to ensure consistent read of write_ptr and read_ptr
    // For example:
    uint32_t primask = __get_PRIMASK(); __disable_irq();
    local_write_ptr = cdc_rx_write_ptr;
    local_read_ptr = cdc_rx_read_ptr;
    __set_PRIMASK(primask); __enable_irq();


    if (local_read_ptr == local_write_ptr) { // Buffer is empty
        return 0;
    }

    // Calculate current number of bytes in the buffer
    if (local_write_ptr >= local_read_ptr) {
        current_size_in_buffer = local_write_ptr - local_read_ptr;
    } else {
        current_size_in_buffer = (RX_BUFFER_SIZE - (local_read_ptr - cdc_rx_buffer)) +
                                 (local_write_ptr - cdc_rx_buffer);
    }

    // Check if enough data is available to "fill" the request
    if (current_size_in_buffer < capacity) {
        return 0; // Not enough data to satisfy the "fill" request
    }

    // Read 'capacity' bytes
    for (uint32_t i = 0; i < capacity; i++) {
        buf[i] = *local_read_ptr;
        local_read_ptr++;
        if (local_read_ptr == (cdc_rx_buffer + RX_BUFFER_SIZE)) { // Wrap around
            local_read_ptr = cdc_rx_buffer;
        }
    }

    // Atomically update the global read pointer
    // __disable_irq();
    cdc_rx_read_ptr = local_read_ptr;

    // If the buffer *was* full, we've now made space, so clear the flag.
    // The USB receive callback (CDC_Receive_FS) should always re-prime the endpoint,
    // so the host's retries (if it got USBD_BUSY before) will eventually succeed.
    if (is_cdc_rx_buffer_full) {
        is_cdc_rx_buffer_full = 0;
    }
    // __enable_irq();

    return capacity; // Successfully read 'capacity' bytes
}


/**
 * @brief Reads from the CDC RX ring buffer.
 * Returns data ONLY if a newline is found OR the provided 'buf' is filled to capacity-1.
 * Otherwise, returns 0 and does not advance the read pointer.
 * The output in 'buf' is null-terminated on a successful read.
 * @param buf Pointer to the destination character buffer.
 * @param capacity The maximum number of characters to store in 'buf' (including null terminator).
 * @retval Number of characters read into 'buf' (excluding null terminator) on success, otherwise 0.
 */
uint32_t Read(char* buf, uint32_t capacity) {
    // Basic capacity checks
    if (capacity == 0) {
        return 0;
    }
    if (capacity == 1) { // Only space for null terminator
        buf[0] = '\0'; // Ensure it's a valid empty string
        return 0;
    }

    uint8_t *local_write_ptr;
    uint8_t *initial_read_ptr;          // Ring buffer's read pointer state at the start of this call
    uint8_t *current_read_iter_ptr;     // Iterator for this potential read operation
    uint32_t bytes_copied_this_attempt = 0;
    _Bool newline_found_this_attempt = 0;
    _Bool capacity_reached_this_attempt = 0;

    // Atomically get current global pointer states
    // It's crucial to get a consistent view, especially of cdc_rx_write_ptr
    // For example:
    uint32_t primask = __get_PRIMASK(); __disable_irq();
    local_write_ptr = cdc_rx_write_ptr;
    initial_read_ptr = cdc_rx_read_ptr;
     __set_PRIMASK(primask); __enable_irq();

    current_read_iter_ptr = initial_read_ptr;

    // Check if the ring buffer is empty
    if (current_read_iter_ptr == local_write_ptr) {
        buf[0] = '\0'; // Ensure output buffer is a valid empty string
        return 0;
    }

    // Attempt to read data
    while (bytes_copied_this_attempt < (capacity - 1) &&  // Leave space for null terminator
           current_read_iter_ptr != local_write_ptr) {     // Don't read past what's written

        uint8_t byte_from_buffer = *current_read_iter_ptr;
        buf[bytes_copied_this_attempt] = (char)byte_from_buffer;
        if (byte_from_buffer == '\b') {
        	if (bytes_copied_this_attempt > 0)
        		bytes_copied_this_attempt--;
        }
        else bytes_copied_this_attempt++;

        current_read_iter_ptr++;
        if (current_read_iter_ptr == (cdc_rx_buffer + RX_BUFFER_SIZE)) { // Wrap around
            current_read_iter_ptr = cdc_rx_buffer;
        }

        if (byte_from_buffer == '\n') {
            newline_found_this_attempt = 1;
            break; // Stop after processing newline
        }
    }

    // Check if the user-provided buffer was filled up to its data capacity
    if (bytes_copied_this_attempt == (capacity - 1)) {
        capacity_reached_this_attempt = 1;
    }

    // --- Conditional Commit ---
    // Only commit the read (update global cdc_rx_read_ptr) if a newline was found
    // OR the destination buffer 'buf' was filled to its data capacity.
    if (newline_found_this_attempt || capacity_reached_this_attempt) {
        buf[bytes_copied_this_attempt] = '\0'; // Null-terminate the successfully read string

        // Atomically update the global read pointer
        // __disable_irq();
        cdc_rx_read_ptr = current_read_iter_ptr; // Commit the read by advancing global pointer

        // If the ring buffer *was* full, we've now definitely made space
        if (is_cdc_rx_buffer_full) {
            is_cdc_rx_buffer_full = 0;
        }
        // __enable_irq();

        return bytes_copied_this_attempt; // Return number of chars copied (excluding null)
    } else {
        // Conditions not met (no newline AND buffer not filled, but ring buffer emptied)
        // Discard the bytes "copied" in this attempt by not advancing cdc_rx_read_ptr.
        buf[0] = '\0'; // Ensure output buffer is a valid empty string
        return 0;      // Indicate no successful read satisfying the conditions
    }
}

void WriteBuf(char* buf, unsigned int len, uint8_t important) {
	//if (important == 0 && (COM_STATUS&&ECHO) == 0) return;
	while (CDC_Transmit(0, buf, len) == USBD_BUSY) HAL_Delay(100);
}

void _WriteString(char* buf, uint8_t important) {
	WriteBuf(buf, strlen(buf), important);
}

void WriteString(char* buf) {
	_WriteString(buf, 0);
}
