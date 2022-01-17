/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile int button_int=0;	//bandera de la rutina de interrupción
int led_select=0;	//determina en qué caso se enciende cada LED
int on_off=0;	//Determina el apagado y encendido de los LEDs. 1:on, 0:off.
int t, tiempo;	//variables auxiliares para usar con HAL_GetTick()
uint8_t lum=0;	//Define la intensidad del LED en función del valor del LDR.
uint32_t adcval;	//Almacena el valor recogido por el ADC del LDR

/*Funcióon Callback para el botón user*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if (GPIO_Pin==GPIO_PIN_0)
	{
		button_int=1;
	}
	else
	{
		button_int=0;
	}


}

/*Función callback del ADC que mide continuamente con el LDR*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	if (hadc->Instance == ADC1){
		adcval=(HAL_ADC_GetValue(&hadc1))/2;
	}

}


/*Función antirrebotes para el botón user y su bandera button_int*/
int debouncer(volatile int* button_int, GPIO_TypeDef* GPIO_port, uint16_t GPIO_number){
	static uint8_t button_count=0;
	static int counter=0;

	if (*button_int==1){
		if (button_count==0) {
			counter=HAL_GetTick();
			button_count++;
		}
		if (HAL_GetTick()-counter>=30){
			counter=HAL_GetTick();
			if (HAL_GPIO_ReadPin(GPIO_port, GPIO_number)!=1){
				button_count=1;
			}
			else{
				button_count++;
			}
			if (button_count==4){ //Periodo antirebotes
				button_count=0;
				*button_int=0;
				return 1;

			}
		}
	}
	return 0;
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(on_off==1)
	  {
		  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7))	//El 3V en el pulsador y GND en la resistencia
		  	  {

		  		  if((HAL_GetTick()-t)>=50)	//antirrebotes del pulsador externo
		  		  {
		  			  if(led_select==5)	//Seceuncia cíclica entre 0 y 5
		  			  {
		  				  led_select=0;
		  			  }
		  			  else
		  			  {
		  				  led_select++;
		  			  }
		  		  }

		  		  t=HAL_GetTick();
		  	  }

		  HAL_ADC_Start_IT(&hadc1);		//Llamada a la función del ADC

		  /*if(adcval>100)
		  {
		  	  lum=0;
		  }
		  else
		  {
		  	  lum=100-adcval;
		  }*/
		  //Transferimos el valor del ADC a una variable para que no exceda los límites del PWM
		  if(adcval>100)	//Acuérdate de conectar 3V a la pata de la resistencia y GND a la del LDR
		  {
		  	  lum=100;
		  }
		  else
		  {
		  	  lum=adcval;
		  }

		  switch(led_select)	//Para cada valor de cuenta se enciende el LED...: 0 verde, 1 naranja, 2 rojo y 3 azul.
		  {
		  	  case 0:
		  	  	 	 	  		 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
		  	  	 	 	  		 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
		  	  	 	 	  		 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		  	  	 	 	  		 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
		  	  	 	 	  		  break;
		  	  case 1:
		  	  	 	 			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, lum);
		  	  	 	 			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
		  	  	 	 			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		  	  	 	 			__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
		  	  	 	 			  break;
		  	  case 2:
		  	  	 	 	  		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
		  	  	 	 	  		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, lum);
		  	  	 	 	  		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		  	  	 	 	  		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
		  	  	 	 	  		  break;
		  	  case 3:
		  	  	 	 			  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
		  	  	 	 			  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
		  	  	 	 			  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, lum);
		  	  	 	 			  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
		  	  	 	 	  		  break;
		  	  case 4:
		  	  	 	 				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
		  	  	 	 				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
		  	  	 	 				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		  	  	 	 				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, lum);
		  	  	 	 				break;
		  	  case 5:
		  	  	 	 				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, lum);
		  	  	 	 				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, lum);
		  	  	 	 				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, lum);
		  	  	 	 				__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, lum);
		  	  	 	 			  	break;
		  }
	  }
	  else
	  {
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	  }
	  //Si el sensor detecta movimiento, se enciende el diodo LED externo durante 5s
	  	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3)==1)
	  	  {
	  		  tiempo = HAL_GetTick();
	  		  while(HAL_GetTick() - tiempo < 5000)
	  		  {
	  			  //Espera de 5s no bloqueante
	  			 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	  		  }
	  	  }
	  	  else
	  	  {
	  		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  	  }


	  	 	if(debouncer(&button_int, GPIOA, GPIO_PIN_0))	//Al pulsar el botón, alterna entre 0 y 1, apagado y encendido
	  	 		  	  	  	  {
	  	 							if(on_off==0)
	  	 							{
	  	 						   	   on_off=1;
	  	 							}
	  	 						  	else
	  	 						  	{
	  	 						   	   on_off=0;
	  	 						  	}
	  	 		  	  	  	  }//*/

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
