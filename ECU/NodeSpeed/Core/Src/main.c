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
#include "stm32f1xx_hal.h"
#include "CAN_SPI.h"
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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uCAN_MSG txMessage;
uCAN_MSG rxMessage;
uint32_t last_can_received_time = 0;

#define DISK_SLOTS   20
#define WHEEL_DIA    0.065
#define PI           3.14159
#define FILTER_SIZE_SPEED 10
#define MAX_PULSES_PER_100MS  14.0

uint8_t safety_lock = 0;
static uint8_t road_limit_kmh = 120;

uint32_t last_speed_time = 0;
int16_t  encoder_cnt = 0;
float    speed_kmh = 0.0;
int16_t speed_buffer[FILTER_SIZE_SPEED] = {0};
uint8_t filter_index = 0;
int16_t raw_cnt = 0;
float filtered_cnt = 0.0;

uint8_t adas_speed_cmd;
int target_pwm = 0;

volatile uint16_t joystickValue = 2048;
uint16_t speedPWM = 0;

volatile uint32_t IC_Val1 = 0;
volatile uint32_t IC_Val2 = 0;
volatile uint32_t pulse_width = 0;
volatile uint8_t Is_First_Captured = 0;
volatile uint32_t encoder_freq = 0;

typedef struct {
    float Q;
    float R;
    float x;
    float P;
    float K;
} KalmanFilter;

KalmanFilter speedFilter = {0.05, 30.0, 0.0, 1.0, 0.0};

float Kalman_Update(KalmanFilter *f, float measurement) {
    f->P = f->P + f->Q;

    f->K = f->P / (f->P + f->R);
    f->x = f->x + f->K * (measurement - f->x);
    f->P = (1 - f->K) * f->P;

    return f->x;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	CANSPI_Initialize();
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	
	HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
	
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); //encoder
	
	HAL_GPIO_WritePin(motor_in1_GPIO_Port, motor_in1_Pin, 0); // chieu lui
	HAL_GPIO_WritePin(motor_in2_GPIO_Port, motor_in2_Pin, 0); //chieu tien
	HAL_GPIO_WritePin(Led_can_GPIO_Port, Led_can_Pin, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		while (CANSPI_Receive(&rxMessage)) {
			if(rxMessage.frame.id == 0x01) {
				joystickValue = (uint16_t)(rxMessage.frame.data3 << 8) | rxMessage.frame.data2;
				adas_speed_cmd = rxMessage.frame.data7;
				
				last_can_received_time = HAL_GetTick();
			}
			
			if (rxMessage.frame.id == 0x03) {
				uint8_t limit_from_cam = rxMessage.frame.data6;
				
					road_limit_kmh = limit_from_cam;
				
				//last_can_received_time = HAL_GetTick();
			}
		}
		
//		int max_allowed_pwm = (road_limit_kmh * 625) / 100;
//		if (max_allowed_pwm > 625) max_allowed_pwm = 625;
		
		if ((HAL_GetTick() - last_can_received_time) > 500){
			adas_speed_cmd = 0;
			joystickValue = 2890;
			safety_lock = 1;
			HAL_GPIO_WritePin(Led_can_GPIO_Port, Led_can_Pin, 0);
		}
		
		int max_allowed_pwm = (road_limit_kmh * 625) / 100;
		if (max_allowed_pwm > 625) max_allowed_pwm = 625;
		
		if (adas_speed_cmd > 0) {
			HAL_GPIO_WritePin(motor_in1_GPIO_Port, motor_in1_Pin, 1);
			HAL_GPIO_WritePin(motor_in2_GPIO_Port, motor_in2_Pin, 0);
			
			uint8_t effective_target = adas_speed_cmd;
			if (effective_target > road_limit_kmh) {
				effective_target = road_limit_kmh;
			}
			
			int base_pwm = (effective_target * 625) / 100;
			float error = (float)effective_target - speed_kmh;
			
			float Kp_val = 25.0;
			if (error < 0) {
				Kp_val = 6.0;
			}
			int compensation = (int)(error * Kp_val);
			target_pwm = base_pwm + compensation;
			
			if (target_pwm > max_allowed_pwm) target_pwm = max_allowed_pwm;
			if (target_pwm > 625) target_pwm = 625;
			if (target_pwm < 0) target_pwm = 0;

			speedPWM = target_pwm;
		} else {
				if (joystickValue < 2800) {
				HAL_GPIO_WritePin(motor_in1_GPIO_Port, motor_in1_Pin, 1);
				HAL_GPIO_WritePin(motor_in2_GPIO_Port, motor_in2_Pin, 0);
				
				speedPWM = (long)(2800 - joystickValue) * 625 / 2800;
				
			} else if (joystickValue > 2980) {
				HAL_GPIO_WritePin(motor_in1_GPIO_Port, motor_in1_Pin, 0);
				HAL_GPIO_WritePin(motor_in2_GPIO_Port, motor_in2_Pin, 1);
				
				speedPWM = (long)(joystickValue - 2980) * 625 / (4095 - 2980);
				
			} else {
				HAL_GPIO_WritePin(motor_in1_GPIO_Port, motor_in1_Pin, 0);
				HAL_GPIO_WritePin(motor_in2_GPIO_Port, motor_in2_Pin, 0);
				speedPWM = 0;
			} 
			
			if (speedPWM > 625) speedPWM = 625;
			if (HAL_GPIO_ReadPin(motor_in2_GPIO_Port, motor_in2_Pin) == 1) {
				if (speedPWM > max_allowed_pwm) {
            speedPWM = max_allowed_pwm;
        }
				if (speed_kmh > (float)(road_limit_kmh + 2)) {
            speedPWM = 0; 
        }
			}
		}
		
		if (HAL_GetTick() - last_speed_time >= 100) {
			raw_cnt = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			
			if (abs(raw_cnt) <= 2) { 
				raw_cnt = 0; 
			}
			
			float raw_speed = ((float)abs(raw_cnt) / MAX_PULSES_PER_100MS) * 100.0;
			speed_kmh = Kalman_Update(&speedFilter, raw_speed);
			
			if (speed_kmh > 120.0) speed_kmh = 120.0;
			if (speed_kmh < 1.0) speed_kmh = 0.0;
			
			last_speed_time = HAL_GetTick();
		}
		
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speedPWM);
		
		txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
		txMessage.frame.id = 0x02;
		txMessage.frame.dlc = 6;
		//encoder data
		txMessage.frame.data0 = (uint8_t)(encoder_freq >> 24);
    txMessage.frame.data1 = (uint8_t)(encoder_freq >> 16);
    txMessage.frame.data2 = (uint8_t)(encoder_freq >> 8);
    txMessage.frame.data3 = (uint8_t)(encoder_freq & 0xFF);
		//speed data
		txMessage.frame.data4 = (uint8_t)((int)(speed_kmh * 100) >> 8);
		txMessage.frame.data5 = (uint8_t)((int)(speed_kmh * 100) & 0xFF);
		
		//CANSPI_Transmit(&txMessage);
		
		if (CANSPI_Transmit(&txMessage)) {
			HAL_GPIO_TogglePin(Led_can_GPIO_Port, Led_can_Pin);
		}
		
		HAL_Delay(50);
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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 127;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 624;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 10;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Led_can_Pin|motor_in2_Pin|motor_in1_Pin|STBY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_CS_GPIO_Port, CAN_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Led_can_Pin CAN_CS_Pin motor_in2_Pin motor_in1_Pin
                           STBY_Pin */
  GPIO_InitStruct.Pin = Led_can_Pin|CAN_CS_Pin|motor_in2_Pin|motor_in1_Pin
                          |STBY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		if (Is_First_Captured == 0) {
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			Is_First_Captured = 1;
		} else {
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			if (IC_Val2 > IC_Val1) {
				pulse_width = IC_Val2 - IC_Val1;
			} else if (IC_Val2 < IC_Val1) { //timer tran
				pulse_width = (0xFFFF - IC_Val1) + IC_Val2;
			} else {
				pulse_width = 0;
			}
			
			if (pulse_width > 0) {
				encoder_freq = 72000000 / pulse_width;
			} else {
				encoder_freq = 0;
			}
			IC_Val1 = IC_Val2;
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
