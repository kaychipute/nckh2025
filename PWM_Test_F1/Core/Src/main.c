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
#include "stdlib.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
float counter;
char buffer[50];
int16_t left_speed, right_speed;

typedef struct Motor
{
	TIM_HandleTypeDef *htim;
	uint16_t CH1, CH2;
	uint16_t speed;
} Motor;

Motor motor_left;
Motor motor_right;


typedef struct Encoder
{
	TIM_HandleTypeDef *htim;
	uint32_t xung;
	int xung_x4, pre_xung_x4;
	float real_vel, fil_vel, pre_vel, target_vel;
	float angle, target_angle;
	float d;
	float target_quang_duong;
	float quang_duong;
	float delta_T;
	uint16_t CPR;
} Encoder;

Encoder encoder_1;
Encoder encoder_2;

typedef struct PID
{
	float target, current;
	float kP, kI, kD;
	float uP, uI, pre_uI, uD, u, u_fil;
	float err, pre_err;
	float deltaT;
	float above_limit, below_limit;
} PID;

PID pid_speed_1;
PID pid_speed_2;
PID pid_position_1;
PID pid_position_2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void MotorInit(Motor *motor, TIM_HandleTypeDef *htim, uint16_t CH1, uint16_t CH2)
{
	motor -> htim = htim;
	motor -> CH1 = CH1;
	motor -> CH2 = CH2;
	HAL_TIM_PWM_Start(motor -> htim, motor -> CH1);
	HAL_TIM_PWM_Start(motor -> htim, motor -> CH2);
}

void drive(Motor *motor, int16_t speed)
{
	if(speed > 0)
	{
		__HAL_TIM_SET_COMPARE(motor->htim, motor->CH1, speed);
		__HAL_TIM_SET_COMPARE(motor->htim, motor->CH2, 0);

	}
	else if(speed < 0)
	{
		__HAL_TIM_SET_COMPARE(motor->htim, motor->CH1, 0);
		__HAL_TIM_SET_COMPARE(motor->htim, motor->CH2, abs(speed));
	}
	else
	{
		__HAL_TIM_SET_COMPARE(motor->htim, motor->CH1, 0);
		__HAL_TIM_SET_COMPARE(motor->htim, motor->CH2, 0);
	}
}

void EncoderInit(Encoder *enc, TIM_HandleTypeDef *htim, float d, float delta_T, uint16_t CPR)
{
	enc->htim = htim;
	enc->d = d;
	enc->delta_T = delta_T;
	enc->CPR = CPR;
	HAL_TIM_Encoder_Start_IT(enc->htim, TIM_CHANNEL_ALL);
}

void EncoderRead(Encoder *enc)
{
	enc -> xung = __HAL_TIM_GET_COUNTER(enc -> htim);
	enc -> xung_x4 += (int16_t) enc -> xung;
	__HAL_TIM_SET_COUNTER(enc -> htim, 0);
	enc -> angle = enc -> xung_x4 * 360 / enc -> CPR;
	enc -> quang_duong = (enc -> xung_x4 / 4) * enc -> d * M_PI / enc -> CPR;
	enc -> real_vel = ((enc -> xung_x4 - enc -> pre_xung_x4)/ enc -> delta_T) / (enc -> CPR)*60/4;
	enc -> fil_vel = 0.854 * enc -> fil_vel + 0.0728 * enc -> real_vel + 0.078 * enc -> pre_vel;
	enc -> pre_vel = enc -> real_vel;
	enc -> pre_xung_x4 = enc -> xung_x4;
}

void PID_setParam(PID *pid, float kP, float kI, float kD, float deltaT, float above_limit, float below_limit)
{
	pid->kP = kP;
	pid->kI = kI;
	pid->kD = kD;
	pid->deltaT = deltaT;
	pid->above_limit = above_limit;
	pid->below_limit = below_limit;
}

void PID_Cal(PID *pid, float target, float current)
{
	pid->err = target - current;

//	if(abs(pid->err) < 0.5) pid->err = 0;

	pid->uP = pid->kP * pid->err;
	pid->uI = pid->pre_uI + pid->kI * pid->err * pid->deltaT;
	pid->uI = (pid->uI > pid->above_limit) ? pid->above_limit : (pid->uI < pid->below_limit) ? pid->below_limit : pid->uI;

	pid->uD = pid->kD * (pid->err - pid->pre_err) / pid->deltaT;

	pid->pre_err = pid->err;
	pid->pre_uI = pid->uI;

	pid->u = pid->uP + pid->uI + pid->uD;
	pid->u = (pid->u > pid->above_limit) ? pid->above_limit : (pid->u < pid->below_limit) ? pid->below_limit : pid->u;
}
void LowPassFilter(float input, float alpha, float *filtered_value)
{
    *filtered_value = alpha * input + (1.0f - alpha) * (*filtered_value);
}

void PID_Speed(PID *pid_speed, Encoder *enc, Motor *motor)
{
	PID_Cal(pid_speed, enc->target_vel, enc->real_vel);
	LowPassFilter(pid_speed->u, 0.1f, &pid_speed->u_fil);
	drive(motor, pid_speed->u_fil);
}

void PID_Position(PID *pid_position, PID *pid_speed, Encoder *enc, Motor *motor)
{
	PID_Cal(pid_position, enc->target_quang_duong, enc->quang_duong);
	LowPassFilter(pid_position->u, 0.03f, &pid_position->u_fil);

	drive(motor, pid_position->u_fil);
//	enc -> target_vel = pid_position -> u;
//	PID_Speed(pid_speed, enc, motor);
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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  EncoderInit(&encoder_1, &htim1, 34.0, 0.001, 4096);
  EncoderInit(&encoder_2, &htim4, 34.0, 0.001, 4096);
  MotorInit(&motor_left, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2);
  MotorInit(&motor_right, &htim2, TIM_CHANNEL_3, TIM_CHANNEL_4);
//  PID_setParam(&pid_speed_2, 17.5549127919031, 254.606072797274, 0.101939398615924, 0.001, 1000, -1000);
  PID_setParam(&pid_position_2, 202.957367753907, 182.290905755023, 20.0825908954933, 0.001, 1000, -1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	EncoderRead(&encoder_1);
	EncoderRead(&encoder_2);
	counter+=0.005;
	sprintf(buffer, "%f, %f\n", counter, encoder_2.fil_vel);
	HAL_UART_Transmit(&huart3, (uint8_t *) buffer, sizeof(buffer), HAL_MAX_DELAY);
	for(uint8_t i = 0; i < sizeof(buffer); i++) buffer[i] = 0;
//	PID_Speed(&pid_speed_2, &encoder_2, &motor_left);
	PID_Position(&pid_position_2, &pid_speed_2, &encoder_2, &motor_left);
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
