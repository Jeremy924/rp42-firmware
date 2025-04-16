#include "COMInterface.h"
//#include "usbd_cdc_if.h"
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "n25q128a.h"
#include "quadspi.h"
#include "usbd_cdc_if.h"


uint32_t ReadFill(char* buf, uint32_t capacity);

_Noreturn void run_console() {

#define MAIN_MENU 0
#define FLASH_MENU 1
#define HAL_MENU 2
#define LCD_MENU 3
	uint8_t current_menu = MAIN_MENU;

	uint32_t program_address = 0;
	uint8_t* program_buf = malloc(256);
	char command[100] = { 0 };

	while (1) {

		int size;
		do {
			size = Read(command, 100);
		} while (size == 0);

		//command[0] = 'h';
		//command[1] = 'i';

		//unsigned char temp[50] = { 0 };
		//sprintf(temp, "\r\nsize: %d\r\n", size);
		//CDC_Transmit_FS(temp, sizeof(temp));

		command[size - 1] = '\0';

		if (current_menu == MAIN_MENU) {
			if (strcmp("clear", command) == 0) {
				WriteString("\033[2J\033[H>");
			} else if (strcmp("whoami", command) == 0) {
				WriteString("RP42 0.0.0.4-b\n>");
			} else if (strcmp("lscpu", command) == 0) {
				WriteString("RP42 0.0.0.4-b\n\tMCU: STM32L475RCT6\n\tCPU: ARM M4 Cortex\n\tMax Clock Speed: 80MHZ\n\tRAM: 128KB\n\tFLASH: 256KB\n>");
			} else if (strcmp("lsblk", command) == 0) {
				WriteString("NAME\tRM\tSIZE\tRO\tTYPE\tMOUNTPOINTS\nsda\t0\t256K\t1\tFLASH\nsdb\t0\t8M\t0\tFLASH\n>");
			} else if (strcmp("frw", command) == 0) {
				WriteString("\033[32mFlash Read/Write Tool\033[0m\nfrw>");
				current_menu = FLASH_MENU;
			} else if (strcmp("echo off", command) == 0) {
				COM_STATUS &= ~ECHO;
			} else if (strcmp("echo on", command) == 0) {
				COM_STATUS |= ECHO;
			} else if (strcmp("sudo hal-config", command) == 0) {
				current_menu = HAL_MENU;
			//} else if (strcmp("whereami", command) == 0) {
			//	_WriteString("Running from FLASH!!!!", 1);
			}
			else if (strcmp("mount", command) == 0) {

				if (CSP_QSPI_EnableMemoryMappedMode() != HAL_OK) {
					WriteString("\033[31mFailed to mount\033[0m");
				}
				WriteString("\n>");
			} else if (strcmp("run", command) == 0) {
				  CSP_QSPI_EnableMemoryMappedMode();

					uint32_t initial_sp = *(__IO uint32_t*)   0x90000000;
					uint32_t reset_vector = *(__IO uint32_t*) 0x90000004;

					SCB->VTOR = 0x90000000;

					typedef void (*free42App)(void);
					free42App jumpToApp = (free42App) reset_vector;

					__set_MSP(initial_sp);
					jumpToApp();
			}
			else if (strcmp("version", command) == 0) {
				WriteString("Version 0.0.5b\n>");
			}
			else if (strcmp("RDF", command) == 0) {
				char* buf = malloc(20);
				uint8_t size;

				if (buf == NULL) continue;

				do {
					HAL_Delay(1);
					size = Read(buf, 20);
				} while (size == 0);

				buf[size - 1] = '\0';

				char* end;
				uint32_t num = 0;
				sscanf(buf, "%16lu", &num);

				char* print_buf = malloc(20);
				for (unsigned int i = 0; i < 20; i++) print_buf[i] = 0;
				sprintf(print_buf, "%lu\n", num);
				WriteString(print_buf);
				free(print_buf);

				for (long i = 0; i < 100; i++) {
					{
						char value = ((char*) num)[i];
						char buf2[5] = {
								value / 100 + '0', value / 10 % 10 + '0', value % 10 + '0', ' ', '\0'
						};

						WriteString(buf2);
					}
				}
				free(buf);
				WriteString(">");
			}
			else {
				WriteString("\033[31mCommand not found\033[0m\n>");
			}
		} else if (current_menu == FLASH_MENU) {
			if (strcmp(command, "exit") == 0) {
				current_menu = MAIN_MENU;
				WriteString("\n>");
			} else if(strcmp(command, "status1") == 0) {
				uint8_t status = CSP_QSPI_Read_StatusRegister1();
				char buf[20];
				sprintf(buf, "%d\nfrw>", (int) status);
				WriteString(buf);
			} else if(strcmp(command, "status2") == 0) {
				uint8_t status = CSP_QSPI_Read_StatusRegister2();
				char buf[20];
				sprintf(buf, "%d\nfrw>", (int) status);
				WriteString(buf);
			} else if(strcmp(command, "status3") == 0) {
				uint8_t status = CSP_QSPI_Read_StatusRegister3();
				char buf[20];
				sprintf(buf, "%d\nfrw>", (int) status);
				WriteString(buf);
			} else if (strcmp(command, "test") == 0) {
				CSP_QSPI_EnableMemoryMappedMode();

				uint32_t original[100];
				uint32_t* read = (uint32_t*) 0x90000000;
				uint8_t pass = 1;

				for (int i = 0; i < 100; i++) original[i] = read[i];

				for (int i = 0; i < 100000; i++) {
					if (i % 1000 == 0) {
						char buf[15];
						sprintf(buf, "% 2d%% complete\r", i / 1000);
						WriteString(buf);
					}
					for (int j = 0; j < 100; j++)
						if (original[j] != read[j]) {
							char buf[15];
							sprintf(buf, "Failed %d\n", i);
							WriteString(buf);
							pass = 0;
							break;
						}

					if (pass == 0) break;
				}

				if (pass == 1) WriteString("Passed");
			} else if (strcmp(command, "PWR ON") == 0) {
				HAL_GPIO_WritePin(PWR_PERPH_GPIO_Port, PWR_PERPH_Pin, SET);
				WriteString("\nfrw>");
			} else if (strcmp(command, "PWR OFF") == 0) {
				HAL_GPIO_WritePin(PWR_PERPH_GPIO_Port, PWR_PERPH_Pin, RESET);
				WriteString("\nfrw>");
			}
			else if (size >= 2 && command[0] == 'R' && command[1] == 'S') {
				if (strcmp(strtok(command, " "), "RS") != 0) {
					WriteString("\033[31mInvalid Format\033[0m\nfrw>");
					continue;
				}
				char* token = strtok(NULL, " ");
				char* parse_end;
				long read_start = strtol(token, &parse_end, 10);
				if (*parse_end != '\0') {
					WriteString("\033[31mInvalid Format\033[0m\nfrw>");
					continue;
				}

				token = strtok(NULL, " ");
				long read_size = strtol(token, &parse_end, 10);
				if (*parse_end != '\0') {
					WriteString("\033[31mInvalid Format\033[0m\nfrw>");
					continue;
				}

				//uint8_t CSP_QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)

				if (read_start < 0 || read_start >= N25Q128A_FLASH_SIZE) {
					WriteString("\033[31mInvalid Start\033[0m\nfrw>");
					continue;
				}

				if (read_size < 0 || read_start + read_size >= N25Q128A_FLASH_SIZE) {
					WriteString("\033[31mInvalid Size\033[0m\nfrw>");
					continue;
				}

				char* buf = (char*) malloc(read_size);
				if (buf == NULL) {
					WriteString("\033[31mOut of Memory\033[0m\nfrw>");
					continue;
				}

				for (size_t i = 0; i < read_size; i++) buf[i] = 0;

				uint8_t status = CSP_QSPI_Read(buf, read_start, read_size);

				if (status != HAL_OK) {
					char num_parse[10];
					sprintf(num_parse, "%d", (int) status);
					WriteString(num_parse);
				} else {
					for (size_t i = 0; i < read_size; i++) {
						char num_parse[20] = {0};
						sprintf(num_parse, "%d: %d\n", (int) (read_start + i), (int) buf[i]);
						_WriteString(num_parse, 1);
					}
				}
				WriteString("frw>");

				free(buf);
			} else if (command[0] == 'R') {
				if (strcmp(strtok(command, " "), "R") != 0) {
					_WriteString("\033[31mInvalid Format\033[0m\nfrw>", 1);
					continue;
				}
				char* token = strtok(NULL, " ");
				char* parse_end;
				long read_start = strtol(token, &parse_end, 10);
				if (*parse_end != '\0') {
					_WriteString("\033[31mInvalid Format\033[0m\nfrw>", 1);
					continue;
				}

				token = strtok(NULL, " ");
				long read_size = strtol(token, &parse_end, 10);
				if (*parse_end != '\0') {
					_WriteString("\033[31mInvalid Format\033[0m\nfrw>", 1);
					continue;
				}

				//uint8_t CSP_QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)

				if (read_start < 0 || read_start >= N25Q128A_FLASH_SIZE) {
					_WriteString("\033[31mInvalid Start\033[0m\nfrw>", 1);
					continue;
				}

				if (read_size < 0 || read_start + read_size >= N25Q128A_FLASH_SIZE) {
					_WriteString("\033[31mInvalid Size\033[0m\nfrw>", 1);
					continue;
				}


				char* buf = (char*) malloc(read_size);
				if (buf == NULL) {
					_WriteString("\033[31mOut of Memory\033[0m\nfrw>", 1);
					continue;
				}

				uint8_t status = CSP_QSPI_Read(buf, read_start, read_size);

				if (status != HAL_OK) {
					char num_parse[10];
					sprintf(num_parse, "%d", (int) status);
					_WriteString(num_parse, 1);
				} else {
					CDC_Transmit_FS(buf, read_size);
				}
				WriteString("\nfrw>");

				free(buf);
			}
			else if (size >= 2 && command[0] == 'W' && command[1] == 'S') {
//uint8_t CSP_QSPI_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
				if (strcmp(strtok(command, " "), "WS") != 0) {
					WriteString("\033[31mInvalid Format\033[0m\nfrw>");
					continue;
				}
				char* token = strtok(NULL, " ");
				char* parse_end;
				long write_start = strtol(token, &parse_end, 10);
				if (*parse_end != '\0') {
					WriteString("\033[31mInvalid Format\033[0m\nfrw>");
					continue;
				}

				token = strtok(NULL, " ");
				long write_size = strtol(token, &parse_end, 10);
				if (token == '\0' || *parse_end != '\0') {
					WriteString("\033[31mInvalid Format\033[0m\nfrw>");
					continue;
				}

				//uint8_t CSP_QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)

				if (write_start < 0 || write_start >= N25Q128A_FLASH_SIZE) {
					WriteString("\033[31mInvalid Start\033[0m\nfrw>");
					continue;
				}

				if (write_size < 0 || write_start + write_size >= N25Q128A_FLASH_SIZE) {
					WriteString("\033[31mInvalid Size\033[0m\nfrw>");
					continue;
				}

				char* write_buf = (char*) malloc(write_size);
				if (write_buf == NULL) {
					WriteString("\033[31mOut of Memory\033[0m\nfrw>");
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
						WriteString("\033[31mInvalid Value\033[0m\nfrw>");
						free(write_buf);
						write_buf = NULL;
						break;
					}
				}

				if (write_buf != NULL) {
					if (CSP_QSPI_WriteMemory(write_buf, write_start, write_size) != HAL_OK) {
						WriteString("\033[31mFailed to write to FLASH\033[0m\nfrw>");
						free(write_buf);
					}
					WriteString("\nWrote\n");
					for (size_t i = 0; i < write_size; i++) {
						char num_parse[20];
						sprintf(num_parse, "%d: %d\n", (int) (write_start + i), (int) write_buf[i]);
						WriteString(num_parse);
					}

					WriteString("frw>");
					free(write_buf);
				}
			}
			else if (size >= 2 && command[0] == 'W' && command[1] == 'H') {
			//uint8_t CSP_QSPI_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
							if (strcmp(strtok(command, " "), "WH") != 0) {
								WriteString("\033[31mInvalid Format\033[0m\nfrw>");
								continue;
							}
							char* token = strtok(NULL, " ");
							char* parse_end;
							long write_start = strtol(token, &parse_end, 10);
							if (*parse_end != '\0') {
								WriteString("\033[31mInvalid Format\033[0m\nfrw>");
								continue;
							}

							token = strtok(NULL, " ");
							long write_size = strtol(token, &parse_end, 10);
							if (*parse_end != '\0') {
								WriteString("\033[31mInvalid Format\033[0m\nfrw>");
								continue;
							}

							//uint8_t CSP_QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)

							if (write_start < 0 || write_start >= N25Q128A_FLASH_SIZE) {
								WriteString("\033[31mInvalid Start\033[0m\nfrw>");
								continue;
							}

							if (write_size < 0 || write_start + write_size >= N25Q128A_FLASH_SIZE) {
								WriteString("\033[31mInvalid Size\033[0m\nfrw>");
								continue;
							}

							char* write_buf = (char*) malloc(write_size*2);
							if (write_buf == NULL) {
								WriteString("\033[31mOut of Memory\033[0m\nfrw>");
								continue;
							}


							while (ReadFill(write_buf, write_size*2) == 0) HAL_Delay(10);

							for (size_t i = 0; i < write_size; i++) {
								char copy[] = {write_buf[i*2], write_buf[i*2+1], '\0'};
								char* end;
								long value = strtol(copy, &end, 16);
								if (*end != '\0') {
									WriteString("\033[31mBad value\033[0m\nfrw>");
									break;
								} else {
									write_buf[i] = value;
								}
							}

							if (write_buf != NULL) {
								if (CSP_QSPI_WriteMemory(write_buf, write_start, write_size) != HAL_OK) {
									WriteString("\033[31mFailed to write to FLASH\033[0m\nfrw>");
									free(write_buf);
								}

								WriteString("frw>");
								free(write_buf);
							}
						}
			else if (size == 2 && command[0] == 'R' && command[1] == 'T') {
				CSP_QSPI_EnableMemoryMappedMode();
				const uint8_t* qspi = (uint8_t*) 0x90000000;

				/*uint8_t* ptr = qspi;
				uint8_t* end_ptr = qspi + 0x80000;
				uint8_t repeats = 100;
				uint32_t success = 1;
				uint32_t last_value = -1;
				// this was written in asm for fun, not for efficiency
				asm(
					"begin:\n"
					"CBZ %1, end\n"
					"SUB %1, %1, #1\n" // decrement repeats counter
					"MOV %0 %3\n" // reset ptr to beginning
					"inner_loop:\n"
					"CMP %0, %2\n" // ptr == end_ptr
					"BEQ begin\n"    // if ptr == end_ptr than go to beginning of loop
					"ADD %0, %0, #1\n" // increment ptr by 1
					"B inner_loop\n" // go to beginning of loop
					"end:"
					: "=r" (success)
					: "r" (ptr), "r" (repeats), "r" (end_ptr), "r" (qspi));*/
			}
			else if (command[0] == 'C') {
				uint8_t buf[256];
				uint32_t check = 0;
				// last couple of kilobytes aren't important
				for (uint32_t address = 0; address < 800000; address+=256) {
					CSP_QSPI_Read(buf, address, 256);
					for (unsigned int i = 0; i < 256; i++)
						if (buf[i] != 255)
							check += buf[i]; // ignore the extra 1s where nothing has been written
				}

				char temp_buf[30];
				sprintf(temp_buf, "%d\n", check);
				WriteString(temp_buf);
			}
			 else if (command[0] == 'W') {
			//uint8_t CSP_QSPI_Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
							if (strcmp(strtok(command, " "), "W") != 0) {
								WriteString("\033[31mInvalid Format\033[0m\nfrw>");
								continue;
							}
							char* token = strtok(NULL, " ");
							char* parse_end;
							long write_start = strtol(token, &parse_end, 10);
							if (*parse_end != '\0') {
								WriteString("\033[31mInvalid Format\033[0m\nfrw>");
								continue;
							}

							token = strtok(NULL, " ");
							long write_size = strtol(token, &parse_end, 10);
							if (*parse_end != '\0') {
								WriteString("\033[31mInvalid Format\033[0m\nfrw>");
								continue;
							}

							//uint8_t CSP_QSPI_Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)

							if (write_start < 0 || write_start >= N25Q128A_FLASH_SIZE) {
								WriteString("\033[31mInvalid Start\033[0m\nfrw>");
								continue;
							}

							if (write_size < 0 || write_start + write_size >= N25Q128A_FLASH_SIZE) {
								WriteString("\033[31mInvalid Size\033[0m\nfrw>");
								continue;
							}
							char write_value_buf[256];
							//char* write_value_buf = malloc(write_size+1);
							for (size_t i = 0; i < write_size; i++) write_value_buf[i] = 0;
							if (write_value_buf == NULL) {
								WriteString("\033[31mOut of Memory\033[0m\nfrw>");
							}

							while (ReadFill(write_value_buf, write_size) == 0) HAL_Delay(10);

							//for (int z = 0; z < write_size; z++) {
							//	CSP_QSPI_WriteMemory(write_value_buf+z, write_starAAAAAAAAt+z, 1);
							//}
							if (CSP_QSPI_WriteMemory(write_value_buf, write_start, write_size) != HAL_OK)
								_WriteString("Error: failed to read", 1);

							WriteString("\nfrw>");

							//free(write_value_buf);
						}
			 else if (strcmp(command, "ec") == 0) {
				 CSP_QSPI_Erase_Chip();
				 WriteString("Please wait 100 seconds before running another flash command\nfrw>");
			 }
			else if (strcmp(command, "we") == 0) {
				QSPI_WriteEnable();
				WriteString("Write Enable flag set\nfrw>");
			}
			else if (strcmp(command, "prg") == 0) {
				char buf[256] = {255};
                char read_back_buf[256] = {0};

                uint32_t pages = 0;

                uint32_t read_size;
                do {
                    read_size = Read(buf, N25Q128A_PAGE_SIZE);
                } while (read_size == 0);

                buf[read_size - 1] = '\0';
                pages = strtoul(buf, NULL, 10);

                if (pages == 0 || pages > N25Q128A_FLASH_SIZE / N25Q128A_PAGE_SIZE) {
                    WriteString("\033[31mInvalid number of pages\033[0m\nfrw>");
                    continue;
                }

                for (uint32_t i = 0; i < pages; i++) {
                    while (ReadFill(program_buf, N25Q128A_PAGE_SIZE) == 0) continue;
                    uint8_t status = CSP_QSPI_WriteMemory(program_buf, i * N25Q128A_PAGE_SIZE, N25Q128A_PAGE_SIZE);
                    if (status != HAL_OK) {
                        _WriteString("\033[31mFailed to write to FLASH\033[0m\nfrw>", 1);
                        break;
                    }

                    CSP_QSPI_Read(read_back_buf, i * N25Q128A_PAGE_SIZE, N25Q128A_PAGE_SIZE);

                    CDC_Transmit_FS(program_buf, N25Q128A_PAGE_SIZE);

                    program_address += N25Q128A_PAGE_SIZE;
                }
			}
			else {
				WriteString(command);
				WriteString("\033[31mCommand not found\033[0m\nfrw>");
			}
		} else if (current_menu == HAL_MENU) {
			if (strcmp(command, "jump") == 0) {
				char* end;
				char* buf = malloc(15);
				while (Read(buf, 15) == 0) HAL_Delay(1);

				uint8_t found_end = 0;
				for (unsigned int i = 0; i < 15; i++) {
					if (buf[i] < '0' || buf[i] > '9') {
						found_end = 1;
						buf[i] = '\0';
						break;
					}
				}
				if (!found_end) buf[14] = '\0';

				long num = strtol(buf, &end, 16);
				if (*end == '\0') {
					char new_buf[15];
					sprintf(new_buf, "%ld\n", num);
					((void(*)(void))num)();
				} else {
					WriteString("\033[31mInvalid address\033[0m\nhal>");
				}

				free(buf);
			} else if (strcmp(command, "exit") == 0) {
				current_menu = MAIN_MENU;
				WriteString("\n>");
			}
			else {
				WriteString(command);
				WriteString("\033[31mCommand not found\033[0m\nhal>");
			}
		}
	}
}

uint32_t ReadFill(char* buf, uint32_t capacity) {
	if (capacity == 0 || com_write_ptr == com_read_ptr) return 0;

	char* write_ptr = com_write_ptr;
	char* read_ptr  = com_read_ptr;

	size_t current_size;

	if (write_ptr < com_read_ptr) {
		current_size = (com_buf + RX_BUFFER_SIZE) - com_read_ptr + (write_ptr - com_buf);
	} else current_size = write_ptr - read_ptr;

	if (current_size < capacity) return 0;


	for (uint32_t i = 0; i < capacity; i++) {
		*buf = *read_ptr;
		buf++;

		if (read_ptr < com_buf + RX_BUFFER_SIZE - 1) read_ptr++;
		else read_ptr = com_buf;
	}

	com_read_ptr = read_ptr;

	return capacity;
}

/**
 * Fills buf with the next string ending with a new line character.
 * A null character will be placed at the end of the string.
 */
uint32_t Read(char* buf, uint32_t capacity) {
    if (capacity == 0) return 0;
    if (com_read_ptr == com_write_ptr) return 0;


    char* read_ptr = com_read_ptr;
    uint32_t size;
    for (size = 0; size < capacity - 1 && read_ptr != com_write_ptr && *read_ptr != '\n'; size++) {
        *buf = *read_ptr;
        buf++;
        read_ptr++;
        if (read_ptr == com_buf + RX_BUFFER_SIZE) read_ptr = com_buf;
    }

    if (size < capacity) {
    	size++;
    	buf[size - 1] = '\0';
    }

    uint8_t is_newline = *read_ptr == '\n' && read_ptr < com_write_ptr;

    if (read_ptr != com_write_ptr) {
        if (read_ptr < com_buf + RX_BUFFER_SIZE - 1) read_ptr++;
        else read_ptr = com_buf;
    }

    if (is_newline || size == capacity - 1) {
        com_read_ptr = read_ptr;
        buf[size] = '\0';
    } else return 0;

    return size;
}
void WriteBuf(char* buf, unsigned int len, uint8_t important) {
	if (important == 0 && (COM_STATUS&&ECHO) == 0) return;
	CDC_Transmit_FS(buf, len);
}

void _WriteString(char* buf, uint8_t important) {
	WriteBuf(buf, strlen(buf), important);
}

void WriteString(char* buf) {
	_WriteString(buf, 0);
}
