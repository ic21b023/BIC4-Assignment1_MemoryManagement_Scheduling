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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
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

/* Definitions for fibonacciTask */
osThreadId_t fibonacciTaskHandle;
const osThreadAttr_t fibonacciTask_attributes = {
  .name = "fibonacciTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for outputTask */
osThreadId_t outputTaskHandle;
const osThreadAttr_t outputTask_attributes = {
  .name = "outputTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for devideTask */
osThreadId_t devideTaskHandle;
const osThreadAttr_t devideTask_attributes = {
  .name = "devideTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for printBinaryTask */
osThreadId_t printBinaryTaskHandle;
const osThreadAttr_t printBinaryTask_attributes = {
  .name = "printBinaryTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void startFibonacciTask(void *argument);
void startOutputTask(void *argument);
void startDevideTask(void *argument);
void startPrintBinaryTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief Fibonacci-Zahl berechnen
  * @param Index der Fibonacci-Folge
  * @retval Fibonacci-Zahl des Indexes
  */
int fibonacci(int zahl) {

	/* Die Fibonacci-Zahl von null ist null */
    if (zahl == 0) {
        return 0;
    }

    /* Die Fibonacci-Zahl von eins ist eins */
    if (zahl == 1) {
        return 1;
    }

    /* Ansonsten wird die Summe der zwei vorherigen Fibonacci-Zahlen zurückgegeben. */
    return fibonacci(zahl - 1) + fibonacci(zahl - 2);
}

/**
  * @brief Rest einer Zahl berechnen
  * @param Dividend
  * @param Devisor
  * @retval Ergebnis
  */
int divide(int x, int y) {

   if(x >= y)
	   return (1 + divide(x - y, y));
   if(x)
	   // printf("Zahl nicht teilbar -> Rest: %d -> ", x);
   return 0;
}

/**
  * @brief Dezimalzahl in Binärdarstellung darstellen
  * @param Dezimalzahl
  * @retval none
  */
void printBinary(long x) {
    if (x<2)
    {
    	// printf("%d",x);
    }
    else
    {
    	printBinary(x/2);
    	// printf("%d",x%2);
    }
}

/**
  * @brief Ausgabe am UART definieren
  * @param Zeichen zur Ausgabe am UART
  * @retval Zeichen
  */
int __io_putchar(int ch)
{
	int ret;

	while( (ret = HAL_UART_GetState(&huart2)) != HAL_UART_STATE_READY );

	if(ch == '\n')
	{
		static unsigned char buf[2]={'\r','\n'};
		HAL_UART_Transmit_IT(&huart2, buf, 2);
	}
	else
	{
		static unsigned char buf;
		buf= (unsigned char) ch;
		HAL_UART_Transmit_IT(&huart2, &buf, 1);
	}

	return ch;
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

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of fibonacciTask */
  fibonacciTaskHandle = osThreadNew(startFibonacciTask, NULL, &fibonacciTask_attributes);

  /* creation of outputTask */
  outputTaskHandle = osThreadNew(startOutputTask, NULL, &outputTask_attributes);

  /* creation of devideTask */
  devideTaskHandle = osThreadNew(startDevideTask, NULL, &devideTask_attributes);

  /* creation of printBinaryTask */
  printBinaryTaskHandle = osThreadNew(startPrintBinaryTask, NULL, &printBinaryTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  huart2.Init.BaudRate = 115200;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_startFibonacciTask */
/**
  * @brief  Function implementing the fibonacciTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_startFibonacciTask */
void startFibonacciTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  /* Zahl, welche sich bei jedem Durchlauf erhöht */
	  static int zahl = 0;

	  /* wenn Zahl = 20, Zahl auf 0 setzen */
	  if(zahl==20){
		  zahl=0;
	  }

	  /* Fibonacci-Zahl des Indexes(Zahl) ermitteln */
	  int erg =fibonacci(zahl);

	  /* Fibonacci-Zahl des Inexes(Zahl) ausgeben  */
	  // printf("Die Fibonacci-Zahl von(%d) lautet: %d\n", zahl, erg);

	  zahl++;
	  osDelay(2000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_startOutputTask */
/**
* @brief Function implementing the outputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startOutputTask */
void startOutputTask(void *argument)
{
  /* USER CODE BEGIN startOutputTask */
  /* Infinite loop */
  for(;;)
  {
	  /* Ausgabe des Gesamt-Stack-Size und den verbleibenden Stack-Size der 3 Tasks (FibonacciTask,DevideTask und PrintBinaryTask) */
	  printf("FibonacciTask: stackSize= %d remaining= %d | DevideTask: stackSize= %d remaining= %d | PrintBinaryTask: stackSize= %d remaining= %d\n",fibonacciTask_attributes.stack_size,osThreadGetStackSpace(fibonacciTaskHandle),     devideTask_attributes.stack_size,osThreadGetStackSpace(devideTaskHandle),  printBinaryTask_attributes.stack_size,osThreadGetStackSpace(printBinaryTaskHandle));
	  osDelay(1000);
  }
  osThreadTerminate(outputTaskHandle);
  /* USER CODE END startOutputTask */
}

/* USER CODE BEGIN Header_startDevideTask */
/**
* @brief Function implementing the devideTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startDevideTask */
void startDevideTask(void *argument)
{
  /* USER CODE BEGIN startDevideTask */
  /* Infinite loop */
  for(;;)
  {

	  /* Zahl, welche sich bei jedem Durchlauf erhöht */
	  static int zahl = 0;

	  /* wenn Zahl = 80, Zahl auf 0 setzen */
	  if(zahl==80)
	  {
		  zahl=0;
	  }

	  /* Rest der Zahl berechnen */
	  int erg = divide(zahl, 3);

	  /* Ausgabe des Restes des Zufallszahl */
	  // printf("%d/3 = Ergebnis : %d\n",zahl, divide(zahl, 3));

	  zahl+=4;
	  osDelay(2000);
  }

  osThreadTerminate(devideTaskHandle);
  /* USER CODE END startDevideTask */
}

/* USER CODE BEGIN Header_startPrintBinaryTask */
/**
* @brief Function implementing the printBinaryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startPrintBinaryTask */
void startPrintBinaryTask(void *argument)
{
  /* USER CODE BEGIN startPrintBinaryTask */
  /* Infinite loop */
  for(;;)
  {
	  /* Zahl, welche sich bei jedem Durchlauf erhöht */
	  static long zahl = 0;

	  /* wenn Zahl = 1000000000, Zahl auf 0 setzen */
	  if(zahl == 1000000000)
	  {
		  zahl=0;
	  }

	  /* Zahl in Binärdarstellung umrechnen */
	  printBinary(zahl);

	  zahl+=1000;
	  osDelay(70);
  }

  osThreadTerminate(printBinaryTaskHandle);
  /* USER CODE END startPrintBinaryTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
