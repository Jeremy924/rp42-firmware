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
#include "usb_device.h"
#include "gpio.h"

#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l475xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
TIM_HandleTypeDef htim16;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t Scan_Keyboard(void);
void Basic_Hardware_Test();
uint8_t GetKey(uint16_t pin);

uint32_t system_call(uint16_t command, void* args);

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define KEY_QUEUE_SIZE 5
uint8_t key_queue[KEY_QUEUE_SIZE] = {255, 255, 255, 255, 255};
//uint32_t test[1000];
uint8_t kqri = 0;
uint8_t kqwi = 0;

SystemCallData __attribute__((section(".SYS_CALL_DATA"))) systemCallData;

/*uint64_t __attribute__((section("SYS_FUNC"))) (*sys_func)(uint16_t command, void* args);*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

void reinitialize_qspi() {
    // Deinitialize QSPI
    HAL_QSPI_DeInit(&hqspi);
    // Reinitialize QSPI with desired settings
    MX_QUADSPI_Init();
}
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
  int code = 2;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();


  MX_QUADSPI_Init();

  code = bootloader();
  /* USER CODE BEGIN 2 */
  if (code == 2) {
	  __set_PSP(malloc(1000));
	  __asm("MRS R0, CONTROL");
	  __asm("ORR R0, R0, #0x07");
	  __asm("MSR CONTROL, R0");
	  __asm("MRS R0, CONTROL");

	  __DSB();
	  __ISB();

	  Basic_Hardware_Test();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MX_TIM16_Init();

	  run_console();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 249;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 49;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim16, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */
  HAL_TIM_Base_Start_IT(&htim16);

  /* USER CODE END TIM16_Init 2 */

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
  RCC_OscInitStruct.PLL.PLLN = 32;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}


/* USER CODE BEGIN 4 */

int bootloader() {
	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
	__HAL_FLASH_DATA_CACHE_ENABLE();

	SCB->SHCSR |= 0x70000;

	//sys_func = system_call;

	  HAL_GPIO_WritePin(ROW0_GPIO_Port, ROW0_Pin|ROW1_Pin|ROW2_Pin|ROW3_Pin|ROW4_Pin|ROW5_Pin|ROW6_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(PWR_PERPH_GPIO_Port, PWR_PERPH_Pin, SET);
	  HAL_Delay(10);

	  int code = Scan_Keyboard();

	  MX_QUADSPI_Init();
	  MX_SPI3_Init();

	  MX_USB_DEVICE_Init();
	  MX_FATFS_Init();
	  MX_TIM16_Init();

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

			jumpToApp();
	  }

	  return code;
}

uint8_t powered_down = 0;
uint8_t last = 0;
uint8_t last_down = 0;
uint32_t last_time = 0;
#define debounce_time 50
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	HAL_NVIC_DisableIRQ(EXTI1_IRQn);
	HAL_NVIC_DisableIRQ(EXTI2_IRQn);
	HAL_NVIC_DisableIRQ(EXTI3_IRQn);
	HAL_NVIC_DisableIRQ(EXTI4_IRQn);

	HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

	  HAL_TIM_Base_Start_IT(&htim16);

	uint8_t key = GetKey(GPIO_Pin);

	//if (key != 255 && uwTick - last_time > 50) {
	//	if (key != 255) last_time = uwTick;
		//last = key;
		//if (key != 255) last_down = key;
		//last_time = HAL_GetTick();


		key_queue[kqwi++] = key;
		if (kqwi == KEY_QUEUE_SIZE) kqwi = 0;
	//}
		  HAL_TIM_Base_Start_IT(&htim16);


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

void debounce_time_up() {
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
#define PASTE_TO_PC     0x0040
#define SET_COPY_ISR    0x0041
#define RM_COPY_ISR     0x0042
#define SET_COPY_BUF    0x0043

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
		//uint8_t key = 255;
		//while (key == 255) {
		//	key = Scan_Keyboard();
		//}
		//uint8_t key = Scan_Keyboard();
		if (kqri == KEY_QUEUE_SIZE) kqri = 0;

		return key;
	case PUSH_KEY:
		key = *(uint8_t*) args;
		if (key < 1 || key > 37) {
			SET_STATUS(INDEX_OUT_OF_RANGE);
			break;
		}
	case RELEASE_KEY:
		key_queue[kqwi++] = key;
		if (kqwi == KEY_QUEUE_SIZE) kqwi = 0;
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
		uint8_t* test = 0;
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
	case POWER_DOWN:
		break;
	case GET_ERROR:
		system_status &= *(uint32_t*) args;
	case CLEAR_ERROR:
		return_value = system_status;
		break;
	case SET_COPY_ISR:
		//COPY_ISR = (void (*)(void *)) args;
		break;
	case RM_COPY_ISR:
		//COPY_ISR = NULL;
		break;
	case PASTE_TO_PC:
		break;
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
