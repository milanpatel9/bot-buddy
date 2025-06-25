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

// instructions to output through transmitter (offset by 1 for the write pin)
#define FORWARD		8 //7 plus write pin
#define BACKWARD	9 //8 plus write pin
#define LEFT		10 //9 plus write pin
#define RIGHT		11 //10 plus write pin
#define STOP		12 //11 plus write pin
#define SPEED_CHANGE		13 //12 plus write pin

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;

/* USER CODE BEGIN PV */

/* Define transmitter button state */
volatile unsigned char tm_button_state; // [X, X, X, X, write, t2, t1, t0]

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void event_loop(void);
static void READ_CONTROLLER(void);
static void DEBUG_BLINKY(void);

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
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_10 | GPIO_PIN_9 | GPIO_PIN_8, 0);
  tm_button_state = 0x00;
  //DEBUG_BLINKY();
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_SET);

  /*Configure GPIO pins : PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Event loop that handles our botbuddy when it is running */
void event_loop(void) {
	while(1){

		while(!tm_button_state){
			/* Continue polling the button state until one of them is pressed */
			READ_CONTROLLER();
		}

		/* Set output through transmitter to be sent to the receiver depending on the button state */
		switch(tm_button_state){
			/* GPIO_PIN_11 -> write pin
			 * GPIO_PIN_10 -> t2
			 * GPIO_PIN_10 -> t1
			 * GPIO_PIN_10 -> t0
			 * Our encodings are active low because the transmitter is
			 */
			case FORWARD:
				/* set write pin, and [t2, t1, t0] = [0, 0, 0] */
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_10 | GPIO_PIN_9 | GPIO_PIN_8, 0);

				/* Test for debugging */
				uint8_t Test1[] = "Forward!/n";
				HAL_UART_Transmit(&hlpuart1, Test1, sizeof(Test1), 10);

				break;

			case BACKWARD:
				/* set write pin, and [t2, t1, t0] = [0, 0, 1] */
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_10 | GPIO_PIN_9, 0);

				/* Test for debugging */
				uint8_t Test2[] = "Backward!/n";
				HAL_UART_Transmit(&hlpuart1, Test2, sizeof(Test2), 10);

				break;

			case LEFT:
				/* set write pin, and [t2, t1, t0] = [0, 1, 0] */
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_10 | GPIO_PIN_8, 0);

				/* Test for debugging */
				uint8_t Test3[] = "Left!/n";
				HAL_UART_Transmit(&hlpuart1, Test3, sizeof(Test3), 10);

				break;

			case RIGHT:
				/* set write pin, and [t2, t1, t0] = [0, 1, 1] */
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_10, 0);

				/* Test for debugging */
				uint8_t Test4[] = "Right!/n";
				HAL_UART_Transmit(&hlpuart1, Test4, sizeof(Test4), 10);

				break;

			case STOP:
				/* set write pin, and [t2, t1, t0] = [1, 0, 0] */
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_9 | GPIO_PIN_8, 0);

				/* Test for debugging */
				uint8_t Test5[] = "Stop!/n";
				HAL_UART_Transmit(&hlpuart1, Test5, sizeof(Test5), 10);

				break;

			case SPEED_CHANGE:
				/* set write pin, and [t2, t1, t0] = [1, 0, 1] */
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_9, 0);

				/* Test for debugging */
				uint8_t Test6[] = "Speed change!/n";
				HAL_UART_Transmit(&hlpuart1, Test6, sizeof(Test6), 10);

				break;
			}

		/* Give a delay in order for the reciver to decode and write to the motors */
		HAL_Delay(750);

		/* reset write pin, and [t2, t1, t0] = [1, 1, 1] */
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11 | GPIO_PIN_10 | GPIO_PIN_9 | GPIO_PIN_8, 1);

		/* extra delay just to be safe? */
		HAL_Delay(750);

		/* Test for debugging */
		uint8_t Test7[] = "Done!\r\n";
		HAL_UART_Transmit(&hlpuart1, Test7, sizeof(Test7), 10);

		/* reset button state back to 0 for polling loop */
		tm_button_state = 0x00;
	}
}

/* Read the buttons passed in to determine what control operation to begin executing */
void READ_CONTROLLER(){
	if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_4)) {tm_button_state = FORWARD;}
	else if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_5)) {tm_button_state = BACKWARD;}
	else if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_6)) {tm_button_state = LEFT;}
	else if (HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_7)) {tm_button_state = RIGHT;}
	else if (HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_0)) {tm_button_state = STOP;}
	else if (HAL_GPIO_ReadPin (GPIOB, GPIO_PIN_1)) {tm_button_state = SPEED_CHANGE;}
	else {tm_button_state = 0x00;}
}

/* Helped debug transmitter signals */
void DEBUG_BLINKY(){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_Delay(500);
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
