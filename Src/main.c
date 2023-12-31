/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <unistd.h>
#define STR_SIZE 50
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	typedef enum {

		Armed_Message_State,

		Motion_Detecting_State,
		Motion_Detected_State,
		Request_Password_State,
		Disarm_State,
		Arm_Device_State,



	} eSystemState;
	/* Prototype Event Handlers */
	eSystemState ArmDeviceHandler(void) {

		HAL_GPIO_WritePin(GPIOA, Arm_Led_Pin, GPIO_PIN_SET);
		return Armed_Message_State;

	}

  eSystemState ArmedMessageHandler(void) {

	  //send uart signal to arm
	  uint8_t buffer42[4]; // Assuming 4 bytes for an integer


	  int value_to_send1 = 42; // Replace with your actual data

	  // Copy the integer value into the buffer
	  buffer42[0] = (value_to_send1 >> 24) & 0xFF;
	  buffer42[1] = (value_to_send1 >> 16) & 0xFF;
	  buffer42[2] = (value_to_send1 >> 8) & 0xFF;
	  buffer42[3] = value_to_send1 & 0xFF;
	  HAL_UART_Transmit(&huart2, buffer42, sizeof(buffer42), HAL_MAX_DELAY);

	  return Motion_Detecting_State;
    }


	eSystemState MotionDetectingHandler(void) {
		//AKSHAY
		return Motion_Detected_State;

	}




  eSystemState MotionDetectedHandler(void)
  {
	  uint8_t buffer41[4]; // Assuming 4 bytes for an integer
	  int value_to_send = 41;
	  buffer41[0] = (value_to_send >> 24) & 0xFF;
	  buffer41[1] = (value_to_send >> 16) & 0xFF;
	  buffer41[2] = (value_to_send >> 8) & 0xFF;
	  buffer41[3] = value_to_send & 0xFF;
	  HAL_UART_Transmit(&huart2, buffer41, sizeof(buffer41), HAL_MAX_DELAY);
	  	 //STEVE
  		//send UART signal to RPI to send intruder message (send signal 41)


	  return Request_Password_State;
  }

	eSystemState PasswordHandler(void) {
		//uint8_t buffer3[4];
		bool correct_password_entered = false;
		uint8_t rx_data;
		while (!correct_password_entered) {
			if (HAL_UART_Receive(&huart2, &rx_data, 1, HAL_MAX_DELAY)) { //password is entered on flask page, if password entered is correct, RPI sends signal to nucleo

				if (rx_data == '9') {
					HAL_GPIO_WritePin(GPIOA, Arm_Led_Pin, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOA, Led_Sensor_Pin, GPIO_PIN_RESET);
					correct_password_entered = true;
				}

			else {
				sleep(5); //wait 5 seconds before sending message again
				MotionDetectedHandler(); //send message again
				correct_password_entered = false;
			}


			}
		}
		return Disarm_State;

	}

	eSystemState DisarmHandler(void) {

		//HAL_GPIO_WritePin(GPIOA, Arm_Led_Sensor, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(GPIOA, Led_Sensor_Pin, GPIO_PIN_RESET);
		bool correct_password_entered = false;
		;
		while (!correct_password_entered) {
			uint8_t rx_data;
			HAL_UART_Receive(&huart2, &rx_data, 1, HAL_MAX_DELAY);

			// Check if the received character is '9'
			if (rx_data == '8') {
			    // Assuming the LED is connected to GPIO pin GPIO_PIN_8
			    HAL_GPIO_WritePin(GPIOA, Arm_Led_Pin, GPIO_PIN_SET);
			    correct_password_entered = true;
			} else {
			    HAL_GPIO_WritePin(GPIOA, Arm_Led_Pin, GPIO_PIN_RESET);
			    correct_password_entered = false;
			    sleep(5); //wait 5 seconds before sending message again
			    MotionDetectedHandler(); //send message again


		}

		return Arm_Device_State;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  	  eSystemState eNextState = Arm_Device_State;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  sensorData = HAL_GPIO_ReadPin(GPIOB, Motion_Sensor_Pin);


	  switch(eNextState) {
	  case Arm_Device_State:
		  eNextState = ArmDeviceHandler();
	  break;
	  case Armed_Message_State:
		  eNextState = ArmedMessageHandler();
	  break;
	  case Motion_Detecting_State:
		  //AKSHAY
		  GPIO_PinState sensorData;
		  uint32_t startTime = 0;
		  uint8_t blinkCount = 0;
		  if (sensorData == GPIO_PIN_SET) {

			  HAL_GPIO_WritePin(GPIOA, Led_Sensor_Pin, GPIO_PIN_SET);
			  startTime = HAL_GetTick(); // Record the start time
			  eNextState = MotionDetectingHandler();
		   }
		  else {
			  eNextState = Motion_Detecting_State;
		  }


	  break;
	  case Motion_Detected_State:
		  eNextState = MotionDetectedHandler();
	  break;
	  case Request_Password_State:
		  eNextState = PasswordHandler();
	  break;
	  case Disarm_State:
			 eNextState = DisarmHandler();
	  break;
	  case Arm_Device_State:
		  eNextState = ArmDeviceHandler();
	  break;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EW_GREEN_LED_Pin|EW_YELLOW_LED_Pin|EW_RED_LED_Pin|Led_Sensor_Pin
                          |Arm_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EW_GREEN_LED_Pin EW_YELLOW_LED_Pin EW_RED_LED_Pin Led_Sensor_Pin
                           Arm_Led_Pin */
  GPIO_InitStruct.Pin = EW_GREEN_LED_Pin|EW_YELLOW_LED_Pin|EW_RED_LED_Pin|Led_Sensor_Pin
                          |Arm_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Motion_Sensor_Pin */
  GPIO_InitStruct.Pin = Motion_Sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Motion_Sensor_GPIO_Port, &GPIO_InitStruct);

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
