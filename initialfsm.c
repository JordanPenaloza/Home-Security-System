	typedef enum {

		Armed_Message_State,
		Web_Page_State,
		Motion_Detecting_State,
		Motion_Detected_State,
		Request_Password_State,
		Disarm_State,
		Arm_Device_State,



	} eSystemState;
	/* Prototype Event Handlers */
	eSystemState ArmDeviceHandler(void) {

		HAL_GPIO_WritePin(GPIO, Armed_LED, GPIO_PIN_SET);
		return Armed_Message_State;

	}

	eSystemState ArmedMessageHandler(void) {
		//send UART signal to RPI to send the armed text message
		return Web_Page_State;
	}

	eSystemState WebPageHandler(void) {

		//send UART signal to RPI to create web page
		return Motion_Detecting_State;

	}

	eSystemState MotionDetectingHandler(void) {
		bool MotionDetected = false;

		while (!MotionDetected) {
			if (HAL_GPIO_ReadPin(GPIO, Motion_Sensor)) {
				HAL_GPIO_WritePin(GPIO, Sensor_LED, GPIO_PIN_SET);
				MotionDetected = true;
			}
			else {
				MotionDetected = false;
			}
		}
		return Motion_Detected_State;

	}

	eSystemState MotionDetectedHandler(void) {

		//send UART signal to RPI to send intruder message
		return Request_Password_State;
	}

	eSystemState PasswordHandler(void) {
		bool correct_password_entered = false;
		while (!correct_password_entered) {
			if (HAL_GPIO_ReadPin(GPIO, RPI_Signal)) { //password is entered on flask page, if password entered is correct, RPI sends signal to nucleo
				correct_password_entered = true;

			}
			else {
				sleep(5); //wait 5 seconds before sending message again
				MotionDetectedHandler(); //send message again
				correct_password_entered = false;
			}
		}
		return Disarm_State;

	}

	eSystemState DisarmHandler(void) {

		HAL_GPIO_WritePin(GPIO, Armed_LED, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIO, Sensor_LED, GPIO_PIN_SET);
		bool correct_password_entered = false;
		while (!correct_password_entered) {
			if (HAL_GPIO_ReadPin(GPIO, RPI_Signal)) { //password is entered on flask page, if password entered is correct, RPI sends signal to nucleo
				correct_password_entered = true;

			}
			else {
				sleep(5); //wait 5 seconds before sending message again
				MotionDetectedHandler(); //send message again
				correct_password_entered = false;
			}
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
  /*Declare eNextState and initialize it to All StopNorth South */
  	  eSystemState eNextState = Transition_NS_State;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  /* USER CODE BEGIN 0 */


	  	/*	Armed_Message_State,
	  		Web_Page_State,
	  		Motion_Detecting_State,
	  		Motion_Detected_State,
	  		Request_Password_State,
	  		Disarm_State,
	  		Arm_State,
	  		Arm_Device_State,*/

    /* USER CODE BEGIN 3 */
	  switch(eNextState) {
	  case Arm_Device_State:
		  eNextState = ArmDeviceHandler();
	  break;
	  case Armed_Message_State:
		  eNextState = ArmedMessageHandler();
	  break;
	  case Web_Page_State:
		  eNextState = WebPageHandler();
	  break;
	  case Motion_Detecting_State:
		  eNextState = MotionDetectingHandler();
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