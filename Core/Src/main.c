/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include <FSUtils.h>
#include "main.h"
#include "fatfs.h"
#include "quadspi.h"
#include "spi.h"
#include "tim.h"
#include "usb_otg.h"
#include "gpio.h"
#include "rng.h"
#include "TestApp.h"
#include <time.h>       // Required for time_t, struct tm, mktime
#include <sys/stat.h>   // Required for struct stat and S_IFREG, S_IFDIR, etc.
#include <string.h>     // Required for memset
#include "ff.h"         // Required for FILINFO and FatFs attribute defines (AM_DIR, AM_RDO)

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l475xx.h"
#include "usb_device.h"
#include "usbd_hid_keyboard.h"
#include "keyboardUtils.h"
#include "ProgressBar.h"
#include "tetris.h"
#include "stm32l4xx_hal_rng.h"
#include "usbd_cdc_acm_if.h"
#include "RP.cc"
#include "COMInterface.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/**
 * Scans keyboard to find if key is currently being pressed. This function will temporarily
 * disable keyboard interrupts while scanning.
 *
 * Returns a number from 1 to 37 representing the key currently being pressed, or 255 if no key is being pressed
 */

#define RAM_FUNC __attribute__((section(".ramtext"))) __attribute__((long_call)) __attribute__((noinline))

/**
 * A function to test the buttons and screen. This function will never return
 */
void Basic_Hardware_Test();


/**
 * Determines which button in the row is being pressed given the row containing the button being pressed.
 * If the incorrect row is given, the the incorrect button will be retruned
 *
 * Parameter pin: pin corresponding row with key being pressed
 * Returns a number from 1 to 37 representing the key being pressed, or 255 if no key is being pressed
 */
uint8_t GetKey(uint16_t pin);

/**
 * Performs the operations specified by command with the given arguments.
 *
 * Parameter command: integer corresponding to the command to run
 * Parameter args: whatever arguments are required for the given command
 *
 * Returns an optional 32-bit integer depending on the command used.
 *
 * See https://Jeremy924.github.io/rp42/firmware_docs.html for more info
 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/**
 * Capacity of queue for storing key presses. May dynamically increase in size if too many keys are enqueued
 */
unsigned int KEY_QUEUE_CAP = 5;

/**
 * Queue for storing key presses. Size is specificied by KEY_QUEUE_SIZE
 */
uint8_t* key_queue;

/**
 * Number of elements in key queue
 */
uint8_t key_queue_size = 0;

#define MAX_OPEN_FILES 1
/**
 * List of currently open files
 */
unsigned int num_open_files = 0;
FIL open_files[MAX_OPEN_FILES];

DIR* open_directory;

/**
 * Index in key_queue where the next unprocessed key is located
 */
uint8_t kqri = 0;

/**
 * Index in key_queue where to write the next key received from keyboard
 */
uint8_t kqwi = 0;

/**
 * Indicates that the application has started. This is used to prevent key interrupts before the program has started.
 */
bool has_started = false;

SystemConfigData systemConfigData;

/**
 * Struct containing the data from system calls. This data is only updated if the SVC instruction is used, not if system_call
 * is call directly.
 *
 * The struct must be located in the SYS_CALL_DATA section so that applications can read and write to it.
 */
SystemCallData __attribute__((section(".SYS_CALL_DATA"))) systemCallData;

void InstallFirmware();
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  int code = 2; // boot option. 0=run app, 1=run cli over USB, 2=run hardware test
  	  	  	  	// boot option is selected when bootloader() is called
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // this may not be necessary?
  HAL_NVIC_SetPriority(SVCall_IRQn, 15, 0);

  // allocate queue to store key presses
  key_queue = (uint8_t*) malloc(sizeof(uint8_t) * KEY_QUEUE_CAP);
  memset(key_queue, 255, KEY_QUEUE_CAP);

  // other peripherals are initialized in the bootloader, not in the seciond below
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  // call the bootloader
  code = bootloader();

  // run the basic_hardware test
  // hardware test should launch like a normal application
  if (code == 2) {
	  // allocate 1kb for the stack for the hardware test program
	  __set_PSP(malloc(1000));

	  // switch to app mode and switch to psp
	  __asm("MRS R0, CONTROL");
	  __asm("ORR R0, R0, #0x07");
	  __asm("MSR CONTROL, R0");
	  __asm("MRS R0, CONTROL");

	  // this may not be necessary since instruction cache is not being used
	  __DSB();
	  __ISB();

	  // start the app
	  Basic_Hardware_Test();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // if the boot code is not 0 or 2, then run the serial interface
	  run_console(&hUsbDevice);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

FRESULT create_system_variable_file(FIL* f) {
	FRESULT res;
	FILINFO fno;
	res = f_stat("/System", &fno);
	if (res == FR_OK) {
		f_unlink("/System");
	}

	FRESULT result = f_mkdir("/System");

	if (result != FR_EXIST && result != FR_OK) return result;

	result = f_open(f, "0:/System/sys.dat", FA_CREATE_ALWAYS | FA_WRITE);
	if (result != FR_OK) return result;

	systemConfigData.version = 2;
	systemConfigData.debounce_ms = 50;
	systemConfigData.bootmode = 0;
	strcpy(systemConfigData.device_name, "no-name");

	char* data = (char*) &systemConfigData;

	unsigned int bytesWritten;
	result = f_write(f, data, sizeof(SystemConfigData), &bytesWritten);

	if (result != FR_OK) return result;

	result = f_close(f);
	if (result != FR_OK) return result;

	result = f_open(f, "/System/sys.dat", FA_READ);

	return result;
}

void load_system_variables() {
	FIL f;

	FRESULT result = f_open(&f, "/System/sys.dat", FA_READ);
	if (result != FR_OK) {
		if (result == FR_NO_FILE || result == FR_NO_PATH) {
			result = create_system_variable_file(&f);

			if (result != FR_OK) Error_Handler();
		} else Error_Handler();
	}

	char* read_buf = &systemConfigData;
	uint32_t buf_size = sizeof(SystemConfigData);
	unsigned int bytesRead;
	result = f_read(&f, read_buf, buf_size, &bytesRead);

	if (bytesRead < sizeof(SystemConfigData)) {
		f_close(&f);
		create_system_variable_file(&f);
	} else {
		if (systemConfigData.version != 2) {
			f_close(&f);
			create_system_variable_file(&f);
		}
	}

	result = f_close(&f);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}


void initialize_firmware_install() {
	if (CSP_QSPI_EnableMemoryMappedMode() != HAL_OK) return;

	__set_MSP(_estack);


	__disable_irq();

	InstallFirmware();
}

RAM_FUNC volatile void InstallFirmware() {
	// unlock flash
	  if(READ_BIT(FLASH->CR, FLASH_CR_LOCK) != 0U)
	  {
		/* Authorize the FLASH Registers access */
		FLASH->KEYR = FLASH_KEY1;
		FLASH->KEYR = FLASH_KEY2;

		/* Verify Flash is unlocked */
		if((FLASH->CR & FLASH_CR_LOCK) != 0U)
		{
			return;
		}
	  }

	  // clear error flags
	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);


	// mass erase flash
	// wait for last operation to complete
	  while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {

	  }

      __HAL_FLASH_DATA_CACHE_DISABLE();

	  /* Mass erase to be done */
	  SET_BIT(FLASH->CR, FLASH_CR_MER1);
	  SET_BIT(FLASH->CR, FLASH_CR_STRT);


	  /* Wait for last operation to be completed */
	  while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {

	  }

	  CLEAR_BIT(FLASH->CR, (FLASH_CR_MER1));



	unsigned int NumberOfDoubleWords = 0x8000 * 2; // 2^18/4
	__IO uint32_t* address = (__IO uint32_t*) 0x08000000;
	uint32_t* pData = (uint32_t*) (0x90000000 + 0x7C0000);

	// program it
    for (uint32_t i = 0; i < NumberOfDoubleWords; i+=2) {
        // If HAL_FLASH_Program is in Flash, this call will fail.
        // Consider direct register access for programming.
    	SET_BIT(FLASH->CR, FLASH_CR_PG);
    	*address = pData[i];
    	address++;

    	__ISB();

    	*address = pData[i + 1];
    	address++;

		  while(__HAL_FLASH_GET_FLAG(FLASH_FLAG_BSY)) {

		  }

    	CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
    }

    SET_BIT(FLASH->CR, FLASH_CR_LOCK);

    __DSB();                                                          /* Ensure all outstanding memory accesses included
                                                                         buffered write are completed before reset */
    SCB->AIRCR  = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos)    |
                             (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                              SCB_AIRCR_SYSRESETREQ_Msk    );         /* Keep priority group unchanged */
    __DSB();

    while (1) {

	}
}

void jumpToSTBootloader() {
	void (*STBootJump)(void);

	volatile uint32_t BootAddr = 0x1FFF0000;

	__disable_irq();
	SysTick->CTRL = 0;

	HAL_RCC_DeInit();

	for (int i = 0; i < 5; i++) {
		NVIC->ICER[i] = 0xffffffff;
		NVIC->ICPR[i] = 0xffffffff;
	}

	__enable_irq();

	STBootJump = (void (*)(void)) (*(uint32_t*) ((BootAddr + 4)));

	__set_MSP(*(uint32_t*) BootAddr);

	STBootJump();

	while (1) {
		// :(
	}
}

/* USER CODE BEGIN 4 */

int bootloader() {
	// disable instruction cache for now since it causes weird errors when switching from intenral flash to qspi flash
	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
	__HAL_FLASH_DATA_CACHE_ENABLE();

	// initialize gpios
	  MX_GPIO_Init();

	  //???
	  ///SCB->SHCSR |= 0x70000;


	// Set all rows to high so that interrupts are triggered when button is pressed
	  HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin|ROW5_Pin|ROW6_Pin, GPIO_PIN_SET);

	  // enable power for flash and lcd
	  HAL_GPIO_WritePin(PWR_PERPH_GPIO_Port, PWR_PERPH_Pin, SET);
	  // give it 10ms for things to start up
	  HAL_Delay(10);

	  // initialize tim16 for deboucning
	  MX_TIM16_Init();
	  // qspi flash interface
	  MX_QUADSPI_Init();
	  // screen interface
	  MX_SPI3_Init();

	  // start usb
	  //MX_USB_OTG_FS_PCD_Init();
	  //MX_USB_DEVICE_Init();

	  // initialize fs
	  MX_FATFS_Init();
	  FS_mount();
	  load_system_variables();

	  // scan keyboard to get bootcode
	  int code;
	  if (systemConfigData.bootmode == 0)
		  code = Scan_Keyboard();
	  else code = systemConfigData.bootmode;
	  if (code == 6) jumpToSTBootloader();
	  /*init_progress_bar();
	  uint8_t progress = 0;
	  while (1) {
		  progress++;
		  HAL_Delay(10);
		  //set_progress(progress);
		  continuous_progres_bar(&progress);
	  }*/


	 // while (1) {
	 //		SendKeystrokes(&hUsbDevice, "hello", 1000);
     //HAL_Delay(2000);
	 // }

	  if (code == 4 || code == 5) run_fatfs_qspi_test(code);
	  else {
		  LCD_BUFFER = (uint8_t*) malloc(132 * 4);
		  if (code == 1) {
			  cdc_rx_buffer = (uint8_t*) 0x20000000 + 50000;
			  cdc_rx_write_ptr = cdc_rx_buffer;
			  cdc_rx_read_ptr = cdc_rx_buffer;

			  MX_USB_OTG_FS_PCD_Init();
			  MX_USB_DEVICE_Init();
			  uint8_t page = 3;
			  uint8_t col = 0;

		  }
		  //run_file_selector("0:", ".dat");
	  }

/*
	  FATFS fs;
	  FRESULT fr;

	  uint8_t force_makefs = 0;
	  fr = f_mount(&fs, "0:", 1);
	  if (fr == FR_NO_FILESYSTEM || (force_makefs == 1)) {
		  BYTE work_buf[512];
		 fr = f_mkfs("0:", FM_ANY, 0, work_buf, 512);

		 f_mkdir("0:/TEST");
	  }


	  if (fr != FR_OK) Error_Handler();

	  FIL f;
	  const char* path = "0:/TEST/FROMRP.TXT"; // Replace with your path

	  fr = f_open(&f, path, FA_READ);
	  BYTE readBuf[256];
	  UINT bytesRead;
	  memset(readBuf, 0, 256);
	  fr = f_read(&f, readBuf, 255, &bytesRead);
	  f_close(&f);

	  fr = f_open(&f, "0:/TEST/FROMRP.TXT", FA_WRITE | FA_CREATE_ALWAYS);

	  BYTE writeData[] = "HELLO FROM RP-42";
	  fr = f_write(&f, writeData, sizeof(writeData), &bytesRead);

	  f_close(&f);
	  f_mount(NULL, "0:", 0);
	  // In your main while(1) loop
*/
	  if (code == 3) {
		  //MX_RNG_Init();
		  uint32_t seed;
		  //HAL_RNG_GenerateRandomNumber(&hrng, &seed);

		  has_started = true;
		  LCD_BUFFER = (uint8_t*) malloc(132 * 4);
		  tetris_main(seed);
	  }

	  // if code is not 1 or 2 then start application
	  if (code != 1 && code != 2) {
		  	has_started = true;
		    //run_file_selector("/AppData/Free42", ".txt");

		    CSP_QSPI_EnableMemoryMappedMode();

			uint32_t initial_sp = *(__IO uint32_t*)   0x90000000;
			uint32_t reset_vector = *(__IO uint32_t*) 0x90000004;
			uint32_t magic = *(__IO uint32_t*) 0x90000188;

			/*if (magic != 0xDECAF) {
				HAL_QSPI_Abort(&hqspi);
				uint8_t page = 3;
				uint8_t col = 0;
				clearSegment(3, 0, 132);
				printText("No program found", &page, &col);
				return 1;
			}*/

			typedef void (*free42App)(void);
			free42App jumpToApp = (free42App) reset_vector;

			  __set_PSP(initial_sp);

			  __asm("MRS R0, CONTROL");
			  __asm("ORR R0, R0, #0x07");
			  __asm("MSR CONTROL, R0");
			  __asm("MRS R0, CONTROL");

			  __DSB();
			  __ISB();

			has_started = true;

			if (code == 6) {
				//printf("starting\r\n");
				//printf("%d\r\n", test_app_main());
			}

			uint8_t screen_buffer[132 * 4];
			LCD_BUFFER = screen_buffer;

			f_chdir("AppData/Free42");
			jumpToApp();
	  }
		has_started = true;



	  return code;
}


void pushKeyQueue(uint8_t key) {
	// if the queue is full, then attempt to increase its size
	if (key_queue_size == KEY_QUEUE_CAP-1) {
		//__asm("BKPT #0");
		// use constant increments since it should almost never expand
		uint8_t* new_queue = malloc(sizeof(uint8_t) * (KEY_QUEUE_CAP + 3));
		memset(new_queue, 255, KEY_QUEUE_CAP + 3);

		// if its out of memory, then just overwrite the current queue
		if (new_queue != NULL) {
			// copy items over to new queue starting at index 0 in the new queue
			unsigned int new_kqri = 0;
			while (kqri != kqwi) {
				new_queue[new_kqri++] = key_queue[kqri++];
				if (kqri == KEY_QUEUE_CAP) kqri = 0;
			}
			kqri = 0; // queue is filled with items that have not yet been read
			kqwi = KEY_QUEUE_CAP; // next index to write is the max size of the old queue
			KEY_QUEUE_CAP += 3; // increase its size by 3

			free(key_queue);

			key_queue = new_queue;
		} else {
			// If it can't resize then just overwrite
		}
	}

	key_queue[kqwi++] = key;
	key_queue_size++;
	if (kqwi == KEY_QUEUE_CAP) kqwi = 0;
	/*uint8_t new_kqwi = kqwi + 1;
	if (new_kqwi == KEY_QUEUE_SIZE) new_kqwi = 0;

	// queue overflowed
	if (new_kqwi == kqri) {
		//while (1) {
			// this is just for testing. In the future it should probably allocate a bigger queue
		//	__asm("NOP");
		//}
	}

	key_queue[kqwi] = key;
	kqwi = new_kqwi;*/
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (!has_started) return;

	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);
	HAL_NVIC_DisableIRQ(EXTI4_IRQn);

	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

	uint8_t key = GetKey(GPIO_Pin);

	pushKeyQueue(key);

	__HAL_TIM_SET_COUNTER(&htim16, 0);
	uint32_t new_period = (systemConfigData.debounce_ms * 10) - 1;
	__HAL_TIM_SET_AUTORELOAD(&htim16, new_period);

	HAL_TIM_Base_Start_IT(&htim16);
}


void Powerdown() {
	// close all open files
	for (unsigned int i = 0; i < num_open_files; i++) {
		f_close(&open_files[i]);
	}

	// turn off lcd
	uint8_t command = 0b10101110;
	sendCommand(&command, 1);

    GPIOA->MODER = 0xFFFFFFFF; // Set all 16 pins to Analog mode (binary 11 for each pin)
    GPIOA->PUPDR = 0x00000000; // No pull-up, no pull-down

    // Configure GPIOB
    GPIOB->MODER = 0xFFFFFFFF;
    GPIOB->PUPDR = 0x00000000;

    // Configure GPIOC
    GPIOC->MODER = 0xFFFFFFFF;
    GPIOC->PUPDR = 0x00000000;

    // Configure GPIOD
    GPIOD->MODER = 0xFFFFFFFF;
    GPIOD->PUPDR = 0x00000000;

    // Configure GPIOE
    GPIOE->MODER = 0xFFFFFFFF;
    GPIOE->PUPDR = 0x00000000;

    // Configure GPIOF
    // Note: Not all STM32L475 packages have all ports fully available.
    // This code is safe as writes to non-existent register addresses are ignored.
    GPIOF->MODER = 0xFFFFFFFF;
    GPIOF->PUPDR = 0x00000000;

    // Configure GPIOG
    GPIOG->MODER = 0xFFFFFFFF;
    GPIOG->PUPDR = 0x00000000;

    // Configure GPIOH
    GPIOH->MODER = 0xFFFFFFFF;
    GPIOH->PUPDR = 0x00000000;

    HAL_PWREx_EnterSHUTDOWNMode();
}

void fil_to_limited_stat(const FIL* fil_ptr, struct stat* st, int logical_drive_num) {
	     if (!fil_ptr || !st) return;
     memset(st, 0, sizeof(struct stat));

     // Size
     st->st_size = fil_ptr->obj.objsize;

     // Mode (from attributes)
     if (fil_ptr->obj.attr & AM_DIR) {
         st->st_mode |= S_IFDIR;
         // Set default directory permissions
     } else {
         st->st_mode |= S_IFREG;
         // Set default file permissions
     }
     if (fil_ptr->obj.attr & AM_RDO) {
         // Remove write permissions
     }

     // Timestamps (st_mtime, st_atime, st_ctime) would be unknown or set to default (e.g., 0)
     st->st_mtime = 0; // Indicate timestamp is not available from FIL directly
     st->st_atime = st->st_mtime;
     st->st_ctime = st->st_mtime;

     // Inode (optional, from start cluster)
     st->st_ino = (ino_t)fil_ptr->obj.sclust;

     // Device
     st->st_dev = (dev_t)logical_drive_num;
     st->st_blksize = 512;

     // ... other fields like nlink, uid, gid would be defaults
}

void filinfo_to_stat(const FILINFO* finfo, struct stat* st, int fatfs_logical_drive_num) {
    if (!finfo || !st) {
        return; // Safety check
    }

    // Initialize the stat structure to zero.
    // This ensures all fields are cleared, especially those not directly mapped.
    memset(st, 0, sizeof(struct stat));

    // 1. File size
    st->st_size = finfo->fsize;

    // 2. File type and permissions (st_mode)
    st->st_mode = 0;
    if (finfo->fattrib & AM_DIR) {
        st->st_mode |= S_IFDIR; // It's a directory
        // Default permissions for a directory: rwxr-xr-x
        st->st_mode |= S_IRUSR | S_IWUSR | S_IXUSR; // User read, write, execute
        st->st_mode |= S_IRGRP | S_IXGRP;         // Group read, execute
        st->st_mode |= S_IROTH | S_IXOTH;         // Other read, execute
    } else {
        st->st_mode |= S_IFREG; // It's a regular file
        // Default permissions for a file: rw-r--r--
        st->st_mode |= S_IRUSR | S_IWUSR;         // User read, write
        st->st_mode |= S_IRGRP;                   // Group read
        st->st_mode |= S_IROTH;                   // Other read
    }

    // Adjust write permissions if the read-only attribute is set
    if (finfo->fattrib & AM_RDO) {
        // Remove write permissions for user, group, and other
        st->st_mode &= ~(S_IWUSR | S_IWGRP | S_IWOTH);
    }

    // 3. Timestamps (st_mtime, st_atime, st_ctime)
    // FatFs fdate and ftime need to be converted to time_t.
    struct tm tminfo;
    memset(&tminfo, 0, sizeof(struct tm));

    // Decode FatFs fdate:
    // Bits 15-9: Year (0..127, offset from 1980)
    // Bits 8-5:  Month (1..12)
    // Bits 4-0:  Day (1..31)
    tminfo.tm_year = ((finfo->fdate >> 9) & 0x7F) + 1980 - 1900; // Year since 1900
    tminfo.tm_mon  = ((finfo->fdate >> 5) & 0x0F) - 1;          // Month (0-11)
    tminfo.tm_mday = (finfo->fdate & 0x1F);                     // Day of month (1-31)

    // Decode FatFs ftime:
    // Bits 15-11: Hour (0..23)
    // Bits 10-5:  Minute (0..59)
    // Bits 4-0:   Second/2 (0..29) -> multiply by 2 for actual seconds
    tminfo.tm_hour = (finfo->ftime >> 11) & 0x1F;
    tminfo.tm_min  = (finfo->ftime >> 5) & 0x3F;
    tminfo.tm_sec  = (finfo->ftime & 0x1F) * 2;

    // Let mktime handle daylight saving time calculations if applicable for the system
    // For embedded systems without DST configuration, it usually doesn't matter much.
    tminfo.tm_isdst = -1; // Let the library determine if DST is in effect

    st->st_mtime = mktime(&tminfo); // Time of last data modification

    // FatFs FILINFO doesn't store separate access or status change times.
    // It's common practice to set them to the modification time.
    st->st_atime = st->st_mtime; // Time of last access
    st->st_ctime = st->st_mtime; // Time of last status change

    // 4. Number of hard links (st_nlink)
    // For FAT filesystems:
    // - Files typically have 1 link.
    // - Directories have at least 2 ('.' and '..'). For simplicity, we can use 1 for files and 2 for dirs.
    //   A more accurate count for directories would require iterating, which is beyond this simple conversion.
    if (st->st_mode & S_IFDIR) {
        st->st_nlink = 2; // Minimum for directories ('.' and '..')
    } else {
        st->st_nlink = 1; // Files typically have one link
    }

    // 5. User ID (st_uid) and Group ID (st_gid)
    // FatFs doesn't have a concept of users or groups in the POSIX sense.
    // Set to 0 (root) or a default value.
    st->st_uid = 0;
    st->st_gid = 0;

    // 6. Device ID (st_dev)
    // This identifies the device containing the file.
    // We can use the FatFs logical drive number if known.
    st->st_dev = (dev_t)fatfs_logical_drive_num;

    // 7. Inode number (st_ino)
    // FatFs doesn't have inodes. The closest might be the start cluster,
    // but FILINFO doesn't directly provide this in a readily usable form for st_ino.
    // Setting to 0 or a placeholder is common. Some systems might use the file's
    // directory entry location or start cluster if available and if it fits ino_t.
    // For simplicity and general compatibility, 0 is often used.
    // If you have the start cluster (e.g., from finfo->obj.sclust if FF_FS_LOCK is enabled
    // and you are using f_stat with FF_FS_LOCK > 0), you could potentially use it,
    // but ensure it fits within ino_t.
    st->st_ino = 0; // Placeholder for inode number

    // 8. Device ID (for special files) (st_rdev)
    // Not applicable for regular FatFs files/directories.
    st->st_rdev = 0;

    // 9. Block size for filesystem I/O (st_blksize)
    // This is the preferred block size for I/O.
    // For FatFs, this is typically the sector size (512 bytes for most SD cards).
    // If you have access to the FATFS object, you could use fs->ssize.
    // Otherwise, use a common default.
    st->st_blksize = 512; // Or fs->ssize if FATFS* fs is available

    // 10. Number of blocks allocated (st_blocks)
    // This is the number of st_blksize blocks allocated to the file.
    if (st->st_blksize > 0) {
        st->st_blocks = (st->st_size + st->st_blksize - 1) / st->st_blksize;
    } else {
        st->st_blocks = 0; // Avoid division by zero if st_blksize is somehow 0
    }
}

#define INDEX_OUT_OF_RANGE 0x0002
#define INVALID_COMMAND 0x0001
uint32_t system_status = 0;
uint8_t* LCD_BUFFER;

#define NOP             0x0000
#define GET_KEY         0x0001
#define WA_KEY          0x0002
#define PUSH_KEY        0x0003
#define RELEASE_KEY     0x0004
#define CLEAR_KEY_QUEUE 0x0005
#define SET_DEBOUNCE_TIME 0x0006
#define LCD_ON          0x0010
#define LCD_OFF         0x0011
#define DRAW_LCD        0x0012
#define CLEAR_LCD       0x0013
#define DRAW_PAGE0      0x0018
#define DRAW_PAGE1      0x0019
#define DRAW_PAGE2      0x001A
#define DRAW_PAGE3      0x001B

#define SELECT_FILE     0x1000
#define POWER_DOWN      0x0020
#define GET_ERROR       0x0030
#define CLEAR_ERROR     0x0031
#define DELAY           0x0040
#define DELAY_UNTIL     0x0041
#define MILLIS          0x0042
#define PASTE           0x0050
#define FOPEN           0x0100
#define FCLOSE          0x0101
#define FREAD           0x0110
#define FWRITE          0x0111
#define FSEEK           0x0112

#define FSTAT           0x0113
#define STAT            0x0114
#define LIST_FILES      0x0120
#define FUNLINK         0x0121
#define FRENAME         0x0122
#define MKDIR           0x0123
#define RMDIR           0x0124
#define OPENDIR         0x0125
#define CLOSEDIR        0x0126
#define NEXT_FILE       0x0127

#define PRINT_TEXT      0x0200
#define DRAW_ICON       0x0201
#define GET_LCD_BUFFER  0x0202

#define SET_STATUS(flag) system_status |= flag
int TEMP_count = 0;
uint32_t system_call(uint16_t command, void* args) {
	uint32_t return_value = 0;
	uint8_t key = 255;
	uint8_t lcd_command = 0;



	if (command == DRAW_LCD) {
		uint8_t* buf = (uint8_t*) args;

		for (unsigned int i = 0; i < 132*4; i++) {
			LCD_BUFFER[i] = buf[i];
		}

		UpdateLCD();

		return 0;
	}
	switch (command) {
	case NOP:
		return *(uint32_t*) args;
	case POWER_DOWN:
		Powerdown();
		return 0;
	case GET_KEY:
		if (kqri == kqwi) return 254;
		char key = key_queue[kqri++];
		if (kqri == KEY_QUEUE_CAP) kqri = 0;
		return key;
		//return (uint32_t) Scan_Keyboard();
	case WA_KEY:
		bool should_wait = false;
		__disable_irq();
		should_wait = kqri == kqwi;
		__enable_irq();

		while (should_wait) {
			HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

			__disable_irq();
			should_wait = kqri == kqwi;
			__enable_irq();
			//abort_memory_mapped_mode();
			//reinitialize_qspi();
		}
		key = key_queue[kqri++];
		key_queue_size--;
		//uint8_t key = 255;
		//while (key == 255) {
		//	key = Scan_Keyboard();
		//}
		//uint8_t key = Scan_Keyboard();
		if (kqri == KEY_QUEUE_CAP) kqri = 0;

		return key;
	case SET_DEBOUNCE_TIME:
		uint32_t debounce_time = (uint32_t) args;



		break;
	case PUSH_KEY:
		key = *(uint8_t*) args;
		if (key < 1 || key > 37) {
			SET_STATUS(INDEX_OUT_OF_RANGE);
			break;
		}
		break;
	case RELEASE_KEY:
		//key_queue[kqwi++] = key;

		//if (kqwi == KEY_QUEUE_CAP) kqwi = 0;
		break;
	case CLEAR_KEY_QUEUE:
		kqwi = kqri;
		break;
	case OPENDIR:
		if (open_directory != NULL) return -1;

		DIR d;
		open_directory = &d;

		return (uint32_t) f_opendir(open_directory, (char*) args);
	case CLOSEDIR:
		if (open_directory == NULL) return -1;

		return f_closedir(open_directory);
		open_directory = NULL;
		break;
	case NEXT_FILE:
		if (open_directory == NULL) return -1;

		struct NextFileParams next_file_params = *(struct NextFileParams*) args;
		FILINFO info;
		FRESULT status = f_readdir(open_directory, &info);
		memcpy(*next_file_params.file_name, info.fname, sizeof(info.fname));
		next_file_params.attributes = info.fattrib;
		break;
	case LCD_ON:
	case LCD_OFF:
		uint8_t command = 0b10101110;
		sendCommand(&command, 1);
		break;
	case DRAW_LCD:
		//uint8_t* buf = (uint8_t*) args;

		//UpdateLCD();
		break;
	case GET_LCD_BUFFER:
		return (uint32_t) LCD_BUFFER;
	case PRINT_TEXT:
		struct TextUIParams* text_params = (struct TextUIParams*) args;
		printText(text_params->text, text_params->page, text_params->col);

		break;
	case DRAW_ICON:
		struct IconUIParams* icon_params = (struct IconUIParams*) args;

		for (uint8_t y = *icon_params->page; y < *icon_params->page + icon_params->height_bytes; y++) {
			setAddress(y, *icon_params->col);
			sendData(icon_params->bitmap + (y - *icon_params->page) * icon_params->width_bits, icon_params->width_bits);
			//memcpy(LCD_BUFFER + 132 * y + *icon_params->col, icon_params->bitmap + y * icon_params->width_bits, icon_params->width_bits);
		}

		*icon_params->col += icon_params->width_bits;
		break;
	case DRAW_PAGE0:
	case DRAW_PAGE1:
	case DRAW_PAGE2:
	case DRAW_PAGE3:
		uint8_t* draw_buf = (uint8_t*) args;
		uint8_t page = command & (0b11); // page is the first two bits in the command
		uint8_t column = draw_buf[0];
		uint8_t length = draw_buf[1];

		setAddress(page, column);
		sendData(draw_buf, length);

		break;
	case CLEAR_LCD:
		for (unsigned int i = 0; i < 132 * 4; i++)
			LCD_BUFFER[i] = 0;
		UpdateLCD();
		break;
	case SELECT_FILE:
		struct FileSelectParams* file_select_params = (struct FileSelectParams*) args;
		run_file_selector(file_select_params->starting_location, file_select_params->file_type, file_select_params->result, file_select_params->size_of_result);

		break;
	case GET_ERROR:
		system_status &= *(uint32_t*) args;
		break;
	case CLEAR_ERROR:
		return_value = system_status;
		break;
	case DELAY:
		break;
	case DELAY_UNTIL:
		break;
	case MILLIS:
		return HAL_GetTick();
	case PASTE:
		char* paste_buf = (char*) args;
		SendKeystrokes(&hUsbDevice, paste_buf, 1000);
		return 0;
		//uint16_t length = strlen(paste_buf);
		//CDC_Transmit_FS(paste_buf, length);
	case FOPEN:
		if (num_open_files == MAX_OPEN_FILES) return -1;
		uint32_t f_handle = num_open_files;
		FIL* f = &open_files[f_handle];

		struct FileOpenParams* params = (struct FileOpenParams*) args;

		if (params->flags == 0) params->flags = FA_READ;
		else params->flags = FA_WRITE | FA_CREATE_ALWAYS;

		status = f_open(f, params->filename, params->flags);

		if (status != FR_OK) return 1;

		num_open_files++;

		params->handle = f_handle + 3; // 0=stdin,1=stdout,2=stderr

		return 0;
	case FCLOSE:
		f_handle = (uint32_t) args;
		if (f_handle < 3) return 0;
		if (f_handle >= (3 + num_open_files)) return 1;

		status = f_close(&open_files[f_handle - 3]);

		if (status != FR_OK) return 0;
		num_open_files--;
		return 0;
	case FWRITE:
		struct FileRWParams* rwParams = (struct FileRWParams*) args;

		rwParams->bytesRW = 0;
		if (rwParams->handle == 0) { // stdin
			return 0;
		} else if (rwParams->handle == 1 || rwParams->handle == 2) { // stdout or stderr
			if (command == FWRITE) {

				printf("%lu", rwParams->bytesRW);
				return 0;
			}
			return 1;
		}

		if (rwParams->handle >= num_open_files + 3) return 2; // out of range of file handles

		f = &open_files[rwParams->handle - 3];

		status = f_write(f, rwParams->buf, rwParams->len, (UINT*)&rwParams->bytesRW);

		if (status == FR_OK) return 0;
		return 3;
	case FREAD:
		rwParams = (struct FileRWParams*) args;

		rwParams->bytesRW = 0;
		if (rwParams->handle == 0) { // stdin
			return 0;
		} else if (rwParams->handle == 1 || rwParams->handle == 2) { // stdout or stderr
			if (command == FWRITE) {

				printf("%lu", rwParams->bytesRW);
				return 0;
			}
			return 1;
		}

		if (rwParams->handle >= num_open_files + 3) return 2; // out of range of file handles

		f = &open_files[rwParams->handle - 3];

		status = f_read(f, rwParams->buf, rwParams->len, (UINT*)&rwParams->bytesRW);

		if (status == FR_OK) return 0;
		return 3;
	case FSEEK:
		struct FileSeekParams* seekParams = (struct FileSeekParams*) args;

		if (seekParams->handle < 3 || seekParams->handle >= 3 + num_open_files) return 1;
		f = &open_files[seekParams->handle - 3];

		FSIZE_t current = f_tell(f);
		FSIZE_t new_offset;
		switch (seekParams->dir) {
		case SEEK_SET:
			new_offset = seekParams->offset;
			break;
		case SEEK_CUR:
			if (seekParams->offset < 0 && (FSIZE_t)(-seekParams->offset) > current)
				new_offset = 0;
			else new_offset = current + seekParams->offset;
			break;
		case SEEK_END:
			FSIZE_t fileSize = f_size(f);
            if (seekParams->offset < 0 && (FSIZE_t)(-seekParams->offset) > fileSize) {
                 new_offset = 0; // Cannot seek before the beginning of the file
            } else {
                new_offset = fileSize + seekParams->offset;
            }

            if (f_lseek(f, new_offset) != FR_OK) return -1;

            return new_offset;
		}

		return 0;
	case FUNLINK:
			const char* path = (const char*) args;

			status = f_unlink(path);
			if (status != FR_OK) return 1;
			return 0;
	case FRENAME:
		struct FileRenameParams* renameParams = (struct FileRenameParams*) args;

		status = f_rename(renameParams->old_path, renameParams->new_path);

		if (status == FR_OK)
			return 0;
		return 1;
	case STAT:
		struct FileStat* stat = (struct FileStat*) args;

		FILINFO finfo;
		status = f_stat(stat->path, &finfo);

		if (status != FR_OK) return 1;

		filinfo_to_stat(&finfo, stat->stat, 0);

		return 0;
	case FSTAT:
		struct FileHandleStat* fstat = (struct FileHandleStat*) args;

		if (fstat->handle < 3 || fstat->handle >= num_open_files + 3) return 1;

		f = &open_files[fstat->handle - 3];

		fil_to_limited_stat(f, fstat->stat, 0);

		return 0;
	case MKDIR:
		const char* name = (const char*)args;

		status = f_mkdir(name);
		if (status != FR_OK) return 1;
		return 0;
	case RMDIR:
		name = (const char*) args;
		status = f_rmdir(name);
		if (status != FR_OK) return 1;

		return 0;
	default:
		SET_STATUS(INVALID_COMMAND);
	}

	return 0;
}

void UpdateLCD() {
	  setAddress(0, 0);
	  sendData(LCD_BUFFER, 132);
	  setAddress(1, 0);
	  sendData(LCD_BUFFER + 132, 132);
	  setAddress(2, 0);
	  sendData(LCD_BUFFER + 132 * 2, 132);
	  setAddress(3, 0);
	  sendData(LCD_BUFFER + 132 * 3, 132);
}

uint8_t GetKey(uint16_t pin)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  GPIO_InitStruct.Pin = COL0_Pin|COL1_Pin|COL2_Pin|COL3_Pin
	                          |COL4_Pin|COL5_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	  //EXTI->IMR1 &= ~(COL0_Pin | COL1_Pin | COL2_Pin | COL3_Pin | COL4_Pin | COL5_Pin);

	  ROW0_GPIO_Port->BRR = ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin|ROW5_Pin|ROW6_Pin;

	uint16_t rows = 0b10100011111;
	uint8_t key_press = 255;

	for (uint8_t r = 0; r < 11; r++) {
		if ((rows & (1 << r)) == 0) continue;
		ROW0_GPIO_Port->BSRR = (1 << r);

		int c = -1;

		if (HAL_GPIO_ReadPin(COL0_GPIO_Port, pin) == GPIO_PIN_SET) {
			switch (pin) {
				case COL0_Pin: c = 0; break;
				case COL1_Pin: c = 1; break;
				case COL2_Pin: c = 2; break;
				case COL3_Pin: c = 3; break;
				case COL4_Pin: c = 4; break;
				case COL5_Pin: c = 5; break;
			}

			int row = r == 8 ? 5 : (r == 10 ? 6 : r);
			key_press = row * 5 + c + 1;
			if (row > 0) key_press++;
			if (row > 1) key_press++;
			if (row > 2) key_press++;
			if (key_press > 13) key_press--;

			ROW0_GPIO_Port->BRR = (1 << r);
			break;
		}
	}

	  ROW0_GPIO_Port->BSRR = ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin|ROW5_Pin|ROW6_Pin;
	  GPIO_InitStruct.Pin = COL0_Pin|COL1_Pin|COL2_Pin|COL3_Pin
	                          |COL4_Pin|COL5_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  //NVIC_ClearPendingIRQ(EXTPin);
	  //EXTI->IMR1 |= (COL0_Pin | COL1_Pin | COL2_Pin | COL3_Pin | COL4_Pin | COL5_Pin);

	return key_press;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim->Instance == TIM16) {
		HAL_TIM_Base_Stop_IT(&htim16);

		uint8_t key = Scan_Keyboard();
		if (key == 255) {
			// they key had been released during the debounce period
			pushKeyQueue(255);
		}

		  __HAL_GPIO_EXTI_CLEAR_IT(COL0_Pin);
		  __HAL_GPIO_EXTI_CLEAR_IT(COL1_Pin);
		  __HAL_GPIO_EXTI_CLEAR_IT(COL2_Pin);
		  __HAL_GPIO_EXTI_CLEAR_IT(COL3_Pin);
		  __HAL_GPIO_EXTI_CLEAR_IT(COL4_Pin);
		  __HAL_GPIO_EXTI_CLEAR_IT(COL5_Pin);

			HAL_NVIC_EnableIRQ(EXTI0_IRQn);
			HAL_NVIC_EnableIRQ(EXTI1_IRQn);
			HAL_NVIC_EnableIRQ(EXTI2_IRQn);
			HAL_NVIC_EnableIRQ(EXTI3_IRQn);
			HAL_NVIC_EnableIRQ(EXTI4_IRQn);
			HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
	}
}

/**
 * This code used to be better, but when setting up git, it deleted all the files, so I had to use an older backup of this file
 *
 */
uint8_t Scan_Keyboard()
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  GPIO_InitStruct.Pin = COL0_Pin|COL1_Pin|COL2_Pin|COL3_Pin
	                          |COL4_Pin|COL5_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	  EXTI->IMR1 &= ~(COL0_Pin | COL1_Pin | COL2_Pin | COL3_Pin | COL4_Pin | COL5_Pin);

	uint16_t rows[] = {ROW0_Pin, ROW1_Pin, ROW2_Pin, ROW3_Pin, ROW4_Pin, ROW5_Pin, ROW6_Pin};
	uint16_t columns = 0b111111;

	int key_press = 255;

	for (int r = 0; r < sizeof(rows); r++) {
		HAL_GPIO_WritePin(ROW0_GPIO_Port, rows[r], SET);

		int c = -1;

		if (HAL_GPIO_ReadPin(COL0_GPIO_Port, COL0_Pin) == GPIO_PIN_SET)
			c = 0;
		else 		if (HAL_GPIO_ReadPin(COL1_GPIO_Port, COL1_Pin) == GPIO_PIN_SET)
			c = 1;
		else		if (HAL_GPIO_ReadPin(COL2_GPIO_Port, COL2_Pin) == GPIO_PIN_SET)
			c = 2;
		else 		if (HAL_GPIO_ReadPin(COL3_GPIO_Port, COL3_Pin) == GPIO_PIN_SET)
			c = 3;
		else		if (HAL_GPIO_ReadPin(COL4_GPIO_Port, COL4_Pin) == GPIO_PIN_SET)
			c = 4;
		else 		if (HAL_GPIO_ReadPin(COL5_GPIO_Port, COL5_Pin) == GPIO_PIN_SET)
			c = 5;

		if (c != -1) {
			int row = r == 8 ? 5 : (r == 10 ? 6 : r);
			key_press = row * 5 + c + 1;
			if (row > 0) key_press++;
			if (row > 1) key_press++;
			if (row > 2) key_press++;
			if (key_press > 13) key_press--;
		}

		HAL_GPIO_WritePin(ROW0_GPIO_Port, rows[r], RESET);

		if (key_press != 255)
			break;

	}

	  GPIO_InitStruct.Pin = COL0_Pin|COL1_Pin|COL2_Pin|COL3_Pin
	                          |COL4_Pin|COL5_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	  EXTI->IMR1 |= (COL0_Pin | COL1_Pin | COL2_Pin | COL3_Pin | COL4_Pin | COL5_Pin);

	  ROW0_GPIO_Port->BSRR = ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin
              |ROW4_Pin|ROW5_Pin|ROW6_Pin;

	return key_press;
}

void Basic_Hardware_Test() {
	HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin|ROW5_Pin|ROW6_Pin, GPIO_PIN_SET);

	  //GPIO_InitTypeDef GPIO_InitStruct = {0};

	  //GPIO_InitStruct.Pin = COL0_Pin;
	  //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  //GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	  //HAL_GPIO_Init(COL0_GPIO_Port, &GPIO_InitStruct);

	  unsigned int page = 0;
	  unsigned int column = 0;
	  //char zero[132] = {0};
	  //for (int p = 0; p < 4; p++) {
		//  setAddress(p, 0);
		 // sendData(zero, 132);
	  //}
	  //system_call(CLEAR_LCD, 0);
	  systemCallData.command = CLEAR_LCD;
	  __asm("SVC #0");

	  while (1)
	  {
		  //int key = Scan_Keyboard();
		  //if (key == 255) continue;

		  //if (key == 33) Powerdown();

		  //HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
		  //if (kqri == kqwi) continue;
		  systemCallData.command = WA_KEY;
		  __asm("SVC #0");
		  uint8_t key = (uint8_t) systemCallData.result;//(uint8_t) sys_func(WA_KEY, 0);//key_queue[kqri++];

		  uint8_t ten = key / 10;
		  uint8_t one = key % 10;

		  if (key == 255) {
			  ten = 3;
			  one = 8;
		  }

		  uint8_t key_string[] = { ten + '0', one + '0', '\0' };

		  systemCallData.command = DRAW_PAGE0 + page;

		  uint8_t draw_data[2 + sizeof(digits[0])];
		  draw_data[0] = column;
		  draw_data[1] = sizeof(digits[0]);
		  for (unsigned int i = 0; i < sizeof(digits[0]); i++)
			  draw_data[2 + i] = digits[ten][i];
		  systemCallData.args = draw_data;
		  systemCallData.command = DRAW_PAGE0 + page;
		  __asm("SVC #0");
		  //system_call(DRAW_PAGE0 + page, draw_data);

		  draw_data[0] += sizeof(digits[0]);
		  for (unsigned int i = 0; i < sizeof(digits[0]); i++)
			  draw_data[2 + i] = digits[one][i];
		  __asm("SVC #0");
		  //system_call(DRAW_PAGE0 + page, draw_data);


		  //setAddress(page, column);
		  //sendData(characters[ten], 5);

		  column += 5;
		  if (column > 131 - 5) {
			  column = 0;
			  page++;
			  if (page > 3) {
				  page = 0;
			  }
		  }

		  //setAddress(page, column);
		  //sendData(characters[one], 5);

		  column += 8;
		  if (column > 131 - 5) {
			  column = 0;
			  page++;
			  if (page > 3) {
				  page = 0;
			  }
		  }
	  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
