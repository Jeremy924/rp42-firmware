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
#include "main.h"
#include "fatfs.h"
#include "quadspi.h"
#include "spi.h"
#include "tim.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l475xx.h"
#include "usb_device.h"
#include "usbd_hid_keyboard.h"
#include "keyboardUtils.h"
#include "FatFSTest.h"
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
uint8_t Scan_Keyboard(void);

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
uint32_t system_call(uint16_t command, void* args);

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

/**
 * Struct containing the data from system calls. This data is only updated if the SVC instruction is used, not if system_call
 * is call directly.
 *
 * The struct must be located in the SYS_CALL_DATA section so that applications can read and write to it.
 */
SystemCallData __attribute__((section(".SYS_CALL_DATA"))) systemCallData;

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
	  run_console();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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

/* USER CODE BEGIN 4 */

int bootloader() {
	// disable instruction cache for now since it causes weird errors when switching from intenral flash to qspi flash
	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
	__HAL_FLASH_DATA_CACHE_ENABLE();

	// initialize gpios
	  MX_GPIO_Init();

	  //???
	SCB->SHCSR |= 0x70000;


	// Set all rows to high so that interrupts are triggered when button is pressed
	  HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin|ROW5_Pin|ROW6_Pin, GPIO_PIN_SET);

	  // enable power for flash and lcd
	  HAL_GPIO_WritePin(PWR_PERPH_GPIO_Port, PWR_PERPH_Pin, SET);
	  // give it 10ms for things to start up
	  HAL_Delay(10);

	  // scan keyboard to get bootcode
	  int code = Scan_Keyboard();

	  // initialize tim16 for deboucning
	  MX_TIM16_Init();
	  // qspi flash interface
	  MX_QUADSPI_Init();
	  // screen interface
	  MX_SPI3_Init();

	  // start usb stack
	  MX_USB_OTG_FS_PCD_Init();
	  MX_USB_DEVICE_Init();

	  // initialize fs
	  MX_FATFS_Init();

	  //run_fatfs_qspi_test();
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
	  // if code is not 1 or 2 then start application
	  if (code != 1 && code != 2) {
		  CSP_QSPI_EnableMemoryMappedMode();

			uint32_t initial_sp = *(__IO uint32_t*)   0x90000000;
			uint32_t reset_vector = *(__IO uint32_t*) 0x90000004;

			// CDC_USB_DEINIT();

			//SCB->VTOR = 0x90000000;

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

			HAL_Delay(50);
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

	HAL_TIM_Base_Start_IT(&htim16);
}


void Powerdown() {
}

#define INDEX_OUT_OF_RANGE 0x0002
#define INVALID_COMMAND 0x0001
uint32_t system_status = 0;
uint8_t LCD_BUFFER[132 * 4];

#define NOP             0x0000
#define GET_KEY         0x0001
#define WA_KEY          0x0002
#define PUSH_KEY        0x0003
#define RELEASE_KEY     0x0004
#define CLEAR_KEY_QUEUE 0x0005
#define LCD_ON          0x0010
#define LCD_OFF         0x0011
#define DRAW_LCD        0x0012
#define DRAW_PAGE0      0x0018
#define DRAW_PAGE1      0x0019
#define DRAW_PAGE2      0x001A
#define DRAW_PAGE3      0x001B
#define CLEAR_LCD       0x0013
#define POWER_DOWN      0x0020
#define GET_ERROR       0x0030
#define CLEAR_ERROR     0x0031
#define DELAY           0x0040
#define DELAY_UNTIL     0x0041
#define MILLIS          0x0042
#define PASTE           0x0050

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
	}
	switch (command) {
	case NOP:
		return *(uint32_t*) args;
	case POWER_DOWN:
		Powerdown();
		return 0;
	case GET_KEY:
		return key_queue[kqri];
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
	case PUSH_KEY:
		key = *(uint8_t*) args;
		if (key < 1 || key > 37) {
			SET_STATUS(INDEX_OUT_OF_RANGE);
			break;
		}
	case RELEASE_KEY:
		//key_queue[kqwi++] = key;

		//if (kqwi == KEY_QUEUE_CAP) kqwi = 0;
		break;
	case CLEAR_KEY_QUEUE:
		kqwi = kqri;
		break;
	case LCD_ON:
		command = 0b10101111;
	case LCD_OFF:
		command = 0b10101110;
		sendCommand(&command, 1);
		break;
	case DRAW_LCD:
		//uint8_t* buf = (uint8_t*) args;

		//UpdateLCD();
		break;
	case DRAW_PAGE0:
	case DRAW_PAGE1:
	case DRAW_PAGE2:
	case DRAW_PAGE3:
		uint8_t* draw_buf = (uint8_t*) args;
		uint8_t page = command & (0b11); // page is the first two bits in the command
		uint8_t* end = draw_buf + draw_buf[1] + 2; // start + length + 2 (ignore the start and size)
		uint8_t* draw_target = LCD_BUFFER + page * 132 + draw_buf[0]; // buf[0] is start index of page

		draw_buf += 2;
		while (draw_buf != end) {
			*draw_target = *draw_buf;
			draw_target++;
			draw_buf++;
		}

		UpdateLCD();
		break;
	case CLEAR_LCD:
		for (unsigned int i = 0; i < 132 * 4; i++)
			LCD_BUFFER[i] = 0;
		UpdateLCD();
		break;
	case GET_ERROR:
		system_status &= *(uint32_t*) args;
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
		SendKeystrokes(&hUsbDevice, paste_buf);
		return 0;
		//uint16_t length = strlen(paste_buf);
		//CDC_Transmit_FS(paste_buf, length);
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

		  uint8_t draw_data[2 + sizeof(characters[0])];
		  draw_data[0] = column;
		  draw_data[1] = sizeof(characters[0]);
		  for (unsigned int i = 0; i < sizeof(characters[0]); i++)
			  draw_data[2 + i] = characters[ten][i];
		  systemCallData.args = draw_data;
		  systemCallData.command = DRAW_PAGE0 + page;
		  __asm("SVC #0");
		  //system_call(DRAW_PAGE0 + page, draw_data);

		  draw_data[0] += sizeof(characters[0]);
		  for (unsigned int i = 0; i < sizeof(characters[0]); i++)
			  draw_data[2 + i] = characters[one][i];
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
