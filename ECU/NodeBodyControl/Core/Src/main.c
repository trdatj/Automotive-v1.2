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
#include "stm32f1xx_hal.h"
#include "CAN_SPI.h"
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
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

uint8_t debug;
uint8_t cruise_state = 0;
uint16_t set_speed = 0;

typedef struct {
    // Byte 0: cum ADAS
    uint8_t btn_byte0;

    // byte 1: cum den
		uint8_t btn_byte1;

    // Byte 2-3: Joystick (16 bit)
    uint16_t joystick_val;

    // Byte 4-5: distance (16 bit)
    uint16_t distance_cm;

    // Byte 6: Level Gap (1, 2, 3)
    uint8_t gap_level;
    
    uint8_t target_speed;
    
} BodyControl_Data_t;

typedef union {
    BodyControl_Data_t frame;  
    uint8_t raw_data[8];
} CAN_Packet_u;

uint16_t Distance  = 0;  // cm

uint16_t readValueX;
uint16_t lastSentX = 2048;
#define JOYSTICK_DEADZONE 15

uCAN_MSG txMessage;
uCAN_MSG rxMessage;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ADC_SAMPLES 5

uint16_t readADC_Filtered(ADC_HandleTypeDef* hadc) {
	uint32_t adc_sum = 0;
	for (int i = 0; i < ADC_SAMPLES; i++) {
		adc_sum += HAL_ADC_GetValue(hadc);
	}
	return adc_sum / ADC_SAMPLES;
}

// Ham doc HC-SR04
uint16_t HCSR04_Read(void) {
    uint32_t pMillis;
    uint32_t Value1 = 0;
    uint32_t Value2 = 0;

    // Tao xung Trigger (10us)
    HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < 10);            
    HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);

    //Cho Echo phan hoi
    pMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin))) {
        if ((HAL_GetTick() - pMillis) > 10) return 0;
    }
    Value1 = __HAL_TIM_GET_COUNTER(&htim1);

    pMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin(Echo_GPIO_Port, Echo_Pin))) {
        if ((HAL_GetTick() - pMillis) > 50) return 0;
    }
    Value2 = __HAL_TIM_GET_COUNTER(&htim1);

    // (Time muc cao) * 0.034 / 2
    uint16_t distance = (Value2 - Value1) * 0.034 / 2;

    return distance;
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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start(&hadc1);
	CANSPI_Initialize();
	
	HAL_TIM_Base_Start(&htim1);
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Led_can_GPIO_Port, Led_can_Pin, 0);
	
	CAN_Packet_u myPacket;
	
	while (HAL_GPIO_ReadPin(Den_pha_GPIO_Port, Den_pha_Pin) == 0 ||
				 HAL_GPIO_ReadPin(Den_cos_GPIO_Port, Den_cos_Pin) == 0 ||
				 HAL_GPIO_ReadPin(XNT_GPIO_Port, XNT_Pin) == 0 ||
				 HAL_GPIO_ReadPin(XNP_GPIO_Port, XNP_Pin) == 0 ||
				 HAL_GPIO_ReadPin(Cruise_control_GPIO_Port, Cruise_control_Pin) == 0 ||
				 HAL_GPIO_ReadPin(SET_GPIO_Port, SET_Pin) == 0 ||
				 HAL_GPIO_ReadPin(RES_GPIO_Port, RES_Pin) == 0 ||
				 HAL_GPIO_ReadPin(GAP_GPIO_Port, GAP_Pin) == 0 ||
				 HAL_GPIO_ReadPin(LIMIT_GPIO_Port, LIMIT_Pin) == 0 ||
				 HAL_GPIO_ReadPin(Hazard_GPIO_Port, Hazard_Pin) == 0)
				 {
					HAL_Delay(10);
				 }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (CANSPI_Receive(&rxMessage)) {
			if (rxMessage.frame.id == 0x03) {
				uint8_t limit_val = rxMessage.frame.data6;
				
				if (limit_val == 0) {
					cruise_state = 0;
					set_speed = 0;
				}
			}
		}
		myPacket.frame.btn_byte0   = 0; 
		
		// --- Active button ---			
		debug = HAL_GPIO_ReadPin(Cruise_control_GPIO_Port, Cruise_control_Pin);
		
		if (HAL_GPIO_ReadPin(Cruise_control_GPIO_Port, Cruise_control_Pin) == GPIO_PIN_RESET) {
        myPacket.frame.btn_byte0 |= (1 << 0);
    }
		
		// --- Set button ---
		if (HAL_GPIO_ReadPin(SET_GPIO_Port, SET_Pin) == GPIO_PIN_RESET) {
        myPacket.frame.btn_byte0 |= (1 << 1);
    }
		
		// --- Res button ---
		if (HAL_GPIO_ReadPin(RES_GPIO_Port, RES_Pin) == GPIO_PIN_RESET) {
        myPacket.frame.btn_byte0 |= (1 << 2);
    }
		
		// --- Gap button
		if (HAL_GPIO_ReadPin(GAP_GPIO_Port, GAP_Pin) == GPIO_PIN_RESET) {
        myPacket.frame.btn_byte0 |= (1 << 3);  
    }
		
		if (HAL_GPIO_ReadPin(LIMIT_GPIO_Port, LIMIT_Pin) == GPIO_PIN_RESET) {
				HAL_Delay(20);
				if (HAL_GPIO_ReadPin(LIMIT_GPIO_Port, LIMIT_Pin) == GPIO_PIN_RESET) {
					myPacket.frame.btn_byte0 |= (1 << 4);
				}
          
    }
		
		myPacket.frame.btn_byte1 = 0;
		
		if (HAL_GPIO_ReadPin(Den_pha_GPIO_Port, Den_pha_Pin) == GPIO_PIN_RESET) {
			myPacket.frame.btn_byte1 |= (1 << 0);
		}
		if (HAL_GPIO_ReadPin(Den_cos_GPIO_Port, Den_cos_Pin) == GPIO_PIN_RESET) {
			myPacket.frame.btn_byte1 |= (1 << 1);
		}
		if (HAL_GPIO_ReadPin(XNT_GPIO_Port, XNT_Pin) == GPIO_PIN_RESET) {
			myPacket.frame.btn_byte1 |= (1 << 2);
		}
		if (HAL_GPIO_ReadPin(XNP_GPIO_Port, XNP_Pin) == GPIO_PIN_RESET) {
			myPacket.frame.btn_byte1 |= (1 << 3);
		}
		if (HAL_GPIO_ReadPin(Hazard_GPIO_Port, Hazard_Pin) == GPIO_PIN_RESET) {
			myPacket.frame.btn_byte1 |= (1 << 4);
		}
		
		myPacket.frame.joystick_val = lastSentX;
		
		Distance = HCSR04_Read();
		myPacket.frame.distance_cm  = Distance;
		
		uint16_t filteredX = readADC_Filtered(&hadc1);
		
		if (abs(filteredX - lastSentX) > JOYSTICK_DEADZONE) {
			lastSentX = filteredX;
		}
		
//		static uint8_t cruise_state = 0;
//    static uint16_t set_speed = 0;
    static uint8_t last_btn_byte0 = 0;
		static uint16_t safety_distance = 50;
		static uint8_t saved_gap_level = 3;
		
		uint8_t current_btn = myPacket.frame.btn_byte0;
		
		// cruise control button
		if ((current_btn & 0x01) && !(last_btn_byte0 & 0x01)) {
			if (cruise_state == 0) {
				cruise_state = 1;  //ready
			} else {
				cruise_state = 0;
				set_speed = 0;
			}
		}
		
		//set/+
		if ((current_btn & 0x02) && !(last_btn_byte0 & 0x02)) {
			if (cruise_state == 1) {
				cruise_state = 2;		//active
				set_speed = 40;
			} else if (cruise_state == 2) {
					set_speed -= 2;
					if (set_speed > 100) set_speed = 100;
			}
		}
		
		//res/-
		if ((current_btn & 0x04) && !(last_btn_byte0 & 0x04)) {
			if (cruise_state == 1) {
				cruise_state = 2;
				set_speed = 40;
			} else if (cruise_state == 2) {
				if (set_speed >= 5) set_speed += 2;
			}
		}
		
		//GAP
		if ((current_btn & 0x08) && !(last_btn_byte0 & 0x08)) {
			saved_gap_level++;
			if (saved_gap_level > 3) saved_gap_level = 1;
			
			// Cập nhật khoảng cách an toàn theo Level
			if (saved_gap_level == 3) safety_distance = 50;
			else if (saved_gap_level == 2) safety_distance = 30;
			else safety_distance = 15;
		}
		
		myPacket.frame.gap_level = saved_gap_level;
		
		last_btn_byte0 = current_btn;
		
		uint8_t final_speed_cmd = 0;
		
		if (cruise_state == 2) {
			if (Distance > 0 && Distance < safety_distance) {
				if (Distance < 10) {
					final_speed_cmd = 0;
				} else {
					final_speed_cmd = (uint8_t)((float)Distance / safety_distance * set_speed);
				}
			} else {
				final_speed_cmd = (uint8_t)set_speed;
			}
		} else {
			final_speed_cmd = 0;
		}
		
		myPacket.frame.target_speed = final_speed_cmd;
		//myPacket.frame.target_speed = (uint8_t)set_speed;
		
		txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
		txMessage.frame.id = 0x01;
		txMessage.frame.dlc = 8;
		txMessage.frame.data0 = myPacket.raw_data[0];
		txMessage.frame.data1 = myPacket.raw_data[1];
		txMessage.frame.data2 = myPacket.raw_data[2];
		txMessage.frame.data3 = myPacket.raw_data[3];
		txMessage.frame.data4 = myPacket.raw_data[4];
		txMessage.frame.data5 = myPacket.raw_data[5];
		txMessage.frame.data6 = myPacket.raw_data[6];
		txMessage.frame.data7 = myPacket.raw_data[7];
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CAN_CS_Pin|Trig_Pin|Led_can_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CAN_CS_Pin Trig_Pin Led_can_Pin */
  GPIO_InitStruct.Pin = CAN_CS_Pin|Trig_Pin|Led_can_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : XNT_Pin XNP_Pin Cruise_control_Pin */
  GPIO_InitStruct.Pin = XNT_Pin|XNP_Pin|Cruise_control_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : GAP_Pin SET_Pin LIMIT_Pin RES_Pin
                           Den_pha_Pin Hazard_Pin Den_cos_Pin Echo_Pin */
  GPIO_InitStruct.Pin = GAP_Pin|SET_Pin|LIMIT_Pin|RES_Pin
                          |Den_pha_Pin|Hazard_Pin|Den_cos_Pin|Echo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
