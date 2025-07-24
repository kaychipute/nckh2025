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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "math.h"
#include "mc522.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_LED 16
#define USE_BRIGHTNESS 1

#define PI 3.14159265

#define SIZE_DATA 5
#define WORD_DISTANCE_BETWEEN 4
#define FLASH_ADDR_BASE 0x08000000
#define FLASH_ADDR_TARGET_PAGE 127
#define FLASH_ADDR_TARGET (FLASH_ADDR_BASE + 1024*FLASH_ADDR_TARGET_PAGE)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t tx_data[10], volume = 30, package_data[MAX_LED + 2];

uint8_t rx_data[3];
uint16_t pwmData[(24*MAX_LED)+50];

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];

volatile int datasentflag = 0;
int brightness = 10, flag;
uint8_t led_num = 0;
uint8_t blink_num;
uint8_t save, wait_to_send = 0;
uint8_t r = 255, g = 255, b = 255;

typedef enum DIR_MODE{
	STOP = 0,
	FORWARD,
	BACKWARD,
	LEFT,
	RIGHT
}DIR_MODE;

DIR_MODE dir_data[MAX_LED] = {STOP};

typedef enum COLOR{
	WHITE,
	RED,
	GREEN,
	BLUE,
	PINK,
}COLOR;

uint8_t currentID[5], memoryID[5], data[5];

uint8_t forward_card[5] = {0x43, 0xd8, 0xf8, 0x4, 0x67};
uint8_t left_card[5] = {0xe3, 0x26, 0x8a, 0xc, 0x43};
uint8_t right_card[5] = {0xc4, 0x36, 0x29, 0x3, 0xd8};
uint8_t backward_card[5] = {0x88, 0x4, 0x36, 0x53, 0xe9};
uint8_t delete_card[5] = {0x83, 0x71, 0x7b, 0x14, 0x9d};
uint8_t confirm_card[5] = {0x73, 0x65, 0x28, 0xf5, 0xcb};


uint8_t current_step = 0, last_step = 0, run = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
#if USE_BRIGHTNESS
		color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
#else
		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
#endif

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwmData[indx] = 60;  // 2/3 of 90
			}

			else pwmData[indx] = 30;  // 1/3 of 90

			indx++;
		}

	}

	for (int i=0; i<50; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
	while (!datasentflag){};
	datasentflag = 0;
}

void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}

void Set_Brightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS

	if (brightness > 45) brightness = 45;
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j=1; j<4; j++)
		{
			float angle = 90-brightness;  // in degrees
			angle = angle*PI / 180;  // in rad
			LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
		}
	}

#endif

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
	datasentflag = 1;
}

void DFPlayer_sendCMD(uint8_t cmd, uint8_t param1, uint8_t param2)
{
	  tx_data[0] = 0x7E;
	  tx_data[1] = 0xFF;
	  tx_data[2] = 0x06;
	  tx_data[3] = cmd;
	  tx_data[4] = 0x00;
	  tx_data[5] = param1;
	  tx_data[6] = param2;
	  uint16_t check_sum = tx_data[1] + tx_data[2] + tx_data[3] + tx_data[4] + tx_data[5] + tx_data[6];
	  check_sum = 0 - check_sum;
	  tx_data[7] = (check_sum >> 8) & 0x00ff;
	  tx_data[8] = (check_sum) & 0x00ff;
	  tx_data[9] = 0xEF;

	  HAL_UART_Transmit(&huart1, tx_data, 10, HAL_MAX_DELAY);
}

void reset_buffer(uint8_t buffer[])
{
	for(uint8_t i = 0; i < 5; i++)
	{
		buffer[i] = 0;
	}
}

void assign_data()
{
	for(uint8_t i = 0; i < 5; i++)
	{
		memoryID[i] = data[i];
	}
}

void write_flash()
{
	HAL_FLASH_Unlock();

	FLASH_EraseInitTypeDef eraseInitStruct;
	uint32_t pageError;
	eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseInitStruct.PageAddress = FLASH_ADDR_TARGET;
	eraseInitStruct.NbPages = 1;
	HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);

	for(uint8_t i = 0; i < SIZE_DATA; i+=WORD_DISTANCE_BETWEEN)
	{
		uint64_t data_write = currentID[i] | (currentID[i + 1] << 8) | (currentID[i + 2] << 16) | (currentID[i + 3] << 24) | (currentID[i + 4] << 32);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR_TARGET + i, data_write);
	}
	HAL_FLASH_Lock();
}

void read_flash(void)
{
	for (uint8_t i = 0; i < SIZE_DATA; i++) {
		data[i] = *(uint8_t *)(FLASH_ADDR_TARGET + i);
	}
	assign_data();
}

uint8_t checkID()
{
	for(uint8_t i = 0; i < sizeof(currentID); i++)
	{
		if(currentID[i] != memoryID[i])
		{
			return 0;
			break;
		}
	}
	return 1;
}

uint8_t checkCard(uint8_t currentID[], uint8_t card[])
{
	for(uint8_t i = 0; i < 5; i++)
	{
		if(currentID[i] != card[i])
		{
			return 0;
			break;
		}
	}
	return 1;
}

void prepareToSend(uint8_t package[])
{
	package[0] = 0x0A;
	for(uint8_t i = 0; i < MAX_LED; i++)
	{
		package[1 + i] = dir_data[i];
	}
	package[MAX_LED + 1] = 0x1C;

	HAL_UART_Transmit(&huart3, package, 18, HAL_MAX_DELAY);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		if(rx_data[0] == 0x0A && rx_data[2] == 0x1C)
		{
			current_step = rx_data[1];
			for(uint8_t i = 0; i < 3; i++) rx_data[i] = 0;
		}
		else
		{
			prepareToSend(package_data);
			for(uint8_t i = 0; i < 3; i++) rx_data[i] = 0;
		}
		HAL_UART_Receive_IT(&huart3, (uint8_t *)rx_data, 3);
	}
}
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  MFRC522_init(GPIOA, MOSI_Pin, GPIOA, MISO_Pin, GPIOA, SCK_Pin, GPIOA, CS_Pin);
  DFPlayer_sendCMD(0x3F, 0x00, 0x01);
  HAL_Delay(200);

  DFPlayer_sendCMD(0x06, 0x00, volume);
  HAL_Delay(200);
  DFPlayer_sendCMD(0x08, 0x00, 0x07);
  HAL_Delay(200);
//  HAL_UART_Receive_IT(&huart3, (uint8_t *)rx_data, 3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(!flag)
	{
		Set_LED(led_num, r, g, b);
		Set_Brightness(brightness);
		WS2812_Send();
		HAL_Delay(250);
		Set_LED(led_num, 0, 0, 0);
		Set_Brightness(brightness);
		WS2812_Send();
		HAL_Delay(250);
		if(checkCard(currentID, forward_card))
		{
			r = 255;
			g = 0;
			b = 0;
			Set_LED(led_num, r, g, b);
			WS2812_Send();
			HAL_Delay(100);
			dir_data[led_num] = FORWARD;
			DFPlayer_sendCMD(0x03, 0x00, 0x01);
			HAL_Delay(900);
			reset_buffer(currentID);
			wait_to_send = 1;
		}
		if(checkCard(currentID, left_card))
		{
			r = 0;
			g = 255;
			b = 0;
			Set_LED(led_num, r, g, b);
			WS2812_Send();
			HAL_Delay(100);
			dir_data[led_num] = LEFT;
			DFPlayer_sendCMD(0x03, 0x00, 0x02);
			HAL_Delay(900);

			reset_buffer(currentID);
			wait_to_send = 1;
		}
		if(checkCard(currentID, right_card))
		{
			r = 0;
			g = 0;
			b = 255;
			Set_LED(led_num, r, g, b);
			WS2812_Send();
			HAL_Delay(100);
			dir_data[led_num] = RIGHT;
			DFPlayer_sendCMD(0x03, 0x00, 0x03);
			HAL_Delay(900);

			reset_buffer(currentID);
			wait_to_send = 1;
		}
		if(checkCard(currentID, backward_card))
		{
			r = 255;
			g = 0;
			b = 180;
			Set_LED(led_num, r, g, b);
			WS2812_Send();
			HAL_Delay(100);
			dir_data[led_num] = BACKWARD;
			DFPlayer_sendCMD(0x03, 0x00, 0x04);
			HAL_Delay(900);
			reset_buffer(currentID);
			wait_to_send = 1;
		}
		if(checkCard(currentID, delete_card))
		{
			r = 0;
			g = 0;
			b = 0;
			Set_LED(led_num, r, g, b);
			WS2812_Send();
			HAL_Delay(100);
			dir_data[led_num] = STOP;
			DFPlayer_sendCMD(0x03, 0x00, 0x06);
			HAL_Delay(900);
			if(led_num > 0) --led_num;
			r = 255;
			g = 255;
			b = 255;
			DFPlayer_sendCMD(0x08, 0x00, 0x07);
			reset_buffer(currentID);
		}
		if(led_num == MAX_LED)
		{
			WS2812_Send();
			HAL_Delay(500);
			DFPlayer_sendCMD(0x03, 0x00, 0x08);
			HAL_Delay(5000);
			DFPlayer_sendCMD(0x08, 0x00, 0x0B);
			flag = 1;
		}
		else
		{
			r = 255;
			g = 255;
			b = 255;
		}
	}
	else
	{
		HAL_UART_Receive_IT(&huart3, (uint8_t *)rx_data, 3);
		if(checkCard(currentID, confirm_card))
		{
			DFPlayer_sendCMD(0x03, 0x00, 0x09);
			HAL_Delay(1000);
			prepareToSend(package_data);
			HAL_Delay(100);
			Set_Brightness(0);
			WS2812_Send();
			HAL_Delay(200);
			Set_Brightness(brightness);
			WS2812_Send();
			HAL_Delay(200);
			Set_Brightness(0);
			WS2812_Send();
			HAL_Delay(200);
			Set_Brightness(brightness);
			WS2812_Send();
			HAL_Delay(200);
			Set_Brightness(0);
			WS2812_Send();
			HAL_Delay(200);
			Set_Brightness(brightness);
			WS2812_Send();
			HAL_Delay(200);
			reset_buffer(currentID);
			DFPlayer_sendCMD(0x08, 0x00, 0x0A);
			run = 1;

		}
		if(dir_data[last_step] == FORWARD)
		{
			r = 255;
			g = 0;
			b = 0;
		}
		if(dir_data[last_step] == BACKWARD)
		{
			r = 255;
			g = 0;
			b = 180;
		}
		if(dir_data[last_step] == LEFT)
		{
			r = 0;
			g = 255;
			b = 0;
		}
		if(dir_data[last_step] == RIGHT)
		{
			r = 0;
			g = 0;
			b = 255;
		}
		if(run)
		{
			if(current_step > last_step)
			{
				Set_LED(last_step, r, g, b);
				Set_Brightness(brightness);
				WS2812_Send();
				last_step = current_step;
			}
			if(current_step == MAX_LED)
			{
				DFPlayer_sendCMD(0x03, 0x00, 0x09);
				HAL_Delay(1000);
				Set_Brightness(0);
				WS2812_Send();
				HAL_Delay(200);
				Set_Brightness(brightness);
				WS2812_Send();
				HAL_Delay(200);
				Set_Brightness(0);
				WS2812_Send();
				HAL_Delay(200);
				Set_Brightness(brightness);
				WS2812_Send();
				HAL_Delay(200);
				Set_Brightness(0);
				WS2812_Send();
				HAL_Delay(200);
				Set_Brightness(brightness);
				WS2812_Send();
				HAL_Delay(200);
				for(uint8_t i = 0; i < MAX_LED; i++)
				{
					Set_LED(i, 0, 0, 0);
					WS2812_Send();
				}
				reset_buffer(currentID);
				flag = 0;
				led_num = 0;
				current_step = 0;
				run = 0;
				last_step = current_step;
				DFPlayer_sendCMD(0x08, 0x00, 0x07);
			}
			Set_LED(last_step, r, g, b);
			Set_Brightness(brightness);
			WS2812_Send();
			HAL_Delay(250);
			Set_LED(current_step, 255, 255, 255);
			Set_Brightness(brightness);
			WS2812_Send();
			HAL_Delay(250);
		}
	}
	if(isCard())
	{
		readCardSerial(currentID);
		MFRC522_Halt();
	}
	if(wait_to_send)
	{
	    DFPlayer_sendCMD(0x03, 0x00, 0x05);
		HAL_Delay(700);
	    DFPlayer_sendCMD(0x03, 0x00, 0x07);
		wait_to_send = 0;
		++led_num;
	}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 90-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BT2_Pin BT1_Pin */
  GPIO_InitStruct.Pin = BT2_Pin|BT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
