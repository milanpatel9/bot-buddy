/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_PERIOD	839

/* Define constants for our movements */
#define FORWARD		0
#define BACKWARD	1
#define LEFT		2
#define RIGHT		3
#define STOP		4
#define SPEED_CHANGE 5


/* POWER = 0 means off and POWER = 1 ON */
#define POWER 	16

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Create Global Struct for the BotBuddy */
typedef struct {
	unsigned char power;
	volatile unsigned char button_state; // each bit represents [X, X, X, power, forward, backward, left, right]
	volatile int read_flag; // read flag to indicate that we want to decode a value from the receiver
	volatile unsigned char read_buf; // read buffer to update the buttons state with
	unsigned int rot_speed; // how fast our motor is spinning
}Bot_Buddy;

volatile int speed_array[] = {50, 75, 100}; // [X, X, X, X, write, t2, t1, t0]
volatile int speed_idx = 0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* define our struct */
Bot_Buddy b_buddy;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void event_loop(void);
static void BT_BUDDY_Init(void);
static void RUN_MOTOR(void);
static void READ_CONTROLLER(void);
static void DECODE_CONTROLLER(void);

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

  /* Initialize our bot buddy */
   BT_BUDDY_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  RUN_MOTOR();
  event_loop();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 19200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  htim2.Init.Period = 839;
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
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7
                           PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


/* Event loop that handles our botbuddy when it is running */
void event_loop(void) {

	while (1) {

		while(!b_buddy.read_flag){

			READ_CONTROLLER();
		}

		DECODE_CONTROLLER();
		b_buddy.button_state = (~b_buddy.read_buf) & 0x07;

		switch(b_buddy.button_state){
			/* GPIO_PIN_11 -> write pin
			 * GPIO_PIN_10 -> t2
			 * GPIO_PIN_10 -> t1
			 * GPIO_PIN_10 -> t0
			 * Our encodings are active low because the transmitter is
			 */
			case FORWARD:
				/* set write pin, and [t2, t1, t0] = [0, 0, 0] */
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 | GPIO_PIN_5, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10 | GPIO_PIN_9, 0);

				/* Test for debugging */
				uint8_t Test1[] = "Forward!/n";
				HAL_UART_Transmit(&hlpuart1, Test1, sizeof(Test1), 10);

				break;

			case BACKWARD:
				/* set write pin, and [t2, t1, t0] = [0, 0, 1] */
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_5, 0);
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10 | GPIO_PIN_9, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 | GPIO_PIN_5, 0);

				/* Test for debugging */
				uint8_t Test2[] = "Backward!/n";
				HAL_UART_Transmit(&hlpuart1, Test2, sizeof(Test2), 10);

				break;

			case LEFT:
				/* set write pin, and [t2, t1, t0] = [0, 1, 0] */
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6 | GPIO_PIN_4, 0);
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9 | GPIO_PIN_5, 1);
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 | GPIO_PIN_10, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);

				/* Test for debugging */
				uint8_t Test3[] = "Left!/n";
				HAL_UART_Transmit(&hlpuart1, Test3, sizeof(Test3), 10);

				break;

			case RIGHT:
				/* set write pin, and [t2, t1, t0] = [0, 1, 1] */
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_4, 1);
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 | GPIO_PIN_10, 1);
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9 | GPIO_PIN_5, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);

				/* Test for debugging */
				uint8_t Test4[] = "Right!/n";
				HAL_UART_Transmit(&hlpuart1, Test4, sizeof(Test4), 10);

				break;

			case STOP:
				/* set write pin, and [t2, t1, t0] = [0, 1, 1] */
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_4, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_5 | GPIO_PIN_9, 0);

				/* Test for debugging */
				uint8_t Test5[] = "Stop!/n";
				HAL_UART_Transmit(&hlpuart1, Test5, sizeof(Test5), 10);

				break;

			case SPEED_CHANGE:
			/* set write pin, and [t2, t1, t0] = [0, 1, 1] */
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
//				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | GPIO_PIN_4, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_5 | GPIO_PIN_9, 0);
				if(b_buddy.rot_speed >= 100) {
					b_buddy.rot_speed = 50;
				}
				else {
					b_buddy.rot_speed = b_buddy.rot_speed + 10;
				}
				/* Test for debugging */
				uint8_t Test7[] = "Speed change!/n";
				HAL_UART_Transmit(&hlpuart1, Test7, sizeof(Test7), 10);

				break;
			}

//		/* Give a delay in order for the receiver to decode and write to the motors */
//		HAL_Delay(1000);
//
//		/* reset write pin, and [t2, t1, t0] = [1, 1, 1] */
//		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4, 1);
//
//		/* extra delay just to be safe? */
//		HAL_Delay(1000);

		/* Test for debugging */
		uint8_t Test6[] = "Done!\r\n";
		HAL_UART_Transmit(&hlpuart1, Test6, sizeof(Test6), 10);

		// reset button state
		b_buddy.button_state = 0x00;
		b_buddy.read_flag = 0;
		b_buddy.read_buf = 0;

		RUN_MOTOR();
		//HAL_Delay(100);
	}
}

/* Reads our GPIO pin*/
void READ_CONTROLLER(void) {
	if (HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_7)) {b_buddy.read_flag = 1;}
//	else if (HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_5)) {b_buddy.button_state = BACKWARD;}
//	else if (HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_6)) {b_buddy.button_state = LEFT;}
//	else if (HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_7)) {b_buddy.button_state = RIGHT;}
	else {b_buddy.read_flag = 0;}
}

/* Decode the output of the reciever */
void DECODE_CONTROLLER(void) {
	if (HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_6)) {b_buddy.read_buf |= 4;}
	if (HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_5)) {b_buddy.read_buf |= 2;}
	if (HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_4)) {b_buddy.read_buf |= 1;}
}
/* Initialization funciton for our BotBuddy */
void BT_BUDDY_Init(void) {
	b_buddy.power = 0x00;
	b_buddy.button_state = 0x00;
	b_buddy.read_flag = 0;
	b_buddy.read_buf = 0;
	b_buddy.rot_speed = 50;
}

void RUN_MOTOR(void) {
	/* RIGHT MOTOR */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (PWM_PERIOD / 100) * b_buddy.rot_speed);

	/* LEFT MOTOR */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (PWM_PERIOD / 100) * b_buddy.rot_speed);
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
