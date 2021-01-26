/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "trace.h"
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

/* USER CODE BEGIN PV */
RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef sDate = {0};

// 0 bit - Uart2 transmit flag
// 1 bit - Uart2 receive flag
// 2 bit - Uart3 transmit flag
// 3 bit - Uart3 receive flag

uint8_t uartState = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void getTime(uint8_t* arr)
{
	  SDK_TRACE_Timestamp(TIME, 1);

	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	  SDK_TRACE_Print("Current time is - %uh %um %us", sTime.Hours, sTime.Minutes, sTime.Seconds);

	  arr[0] = sTime.Hours;
	  arr[1] = sTime.Minutes;
	  arr[2] = sTime.Seconds;

	  SDK_TRACE_Timestamp(TIME, 0);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t transmitBuffer[3] = {0};
	uint8_t receiveBuffer[3] = {0};

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
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Do not remove this code below */
  MX_TRACE_Init();
  SDK_TRACE_Start();
  /* Do not remove this code from above */

  SDK_TRACE_Timestamp(HEADER, 1);
  SDK_TRACE_Print("%s","Lab 1");
  SDK_TRACE_Timestamp(HEADER, 0);

  /* Place your code from here */

  SDK_TRACE_Print("uart3 uart2 BL mode");

  //Get time and reset uart state buffer
  getTime(transmitBuffer);
  uartState = 0x00;

  for (int offset = 0; offset < 3; ++offset)
  {
	  //send through U3
	  SDK_TRACE_Timestamp(U3_TSM, 1);
	  HAL_UART_Transmit(&huart3, transmitBuffer+offset, 1, 1000);
	  SDK_TRACE_Timestamp(U3_TSM, 0);

	  //receive through U2
	  SDK_TRACE_Timestamp(U2_RCV, 1);
	  HAL_UART_Receive(&huart2, receiveBuffer+offset, 1, 1000);
	  SDK_TRACE_Timestamp(U2_RCV, 0);
  }

  //Print received time
  SDK_TRACE_Timestamp(PRINT, 1);
  SDK_TRACE_Print("Printing received data");
  SDK_TRACE_Print("uart2: %uh %um %us", receiveBuffer[0], receiveBuffer[1], receiveBuffer[2]);
  SDK_TRACE_Timestamp(PRINT, 0);

  SDK_TRACE_Print("uart2 uart3 IT mode");

  //Get time and reset uart state buffer
  sTime.Hours = 13;
  sTime.Minutes = 13;
  sTime.Seconds = 13;
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

  getTime(transmitBuffer);
  uartState = 0x00;

  //receive through U3
  SDK_TRACE_Timestamp(U3_RCV, 1);
  HAL_UART_Receive_IT(&huart3, receiveBuffer, 3);
  SDK_TRACE_Timestamp(U3_RCV, 0);

  //send through U2
  SDK_TRACE_Timestamp(U2_TSM, 1);
  HAL_UART_Transmit_IT(&huart2, transmitBuffer, 3);
  SDK_TRACE_Timestamp(U2_TSM, 0);

  //Wait until data is received by uart3
  SDK_TRACE_Timestamp(IDLE, 1);
  while (!(uartState & 0x08)) { };
  SDK_TRACE_Timestamp(IDLE, 0);

  //Print received time
  SDK_TRACE_Timestamp(PRINT, 1);
  SDK_TRACE_Print("Printing received data");
  SDK_TRACE_Print("uart3: %uh %um %us", receiveBuffer[0], receiveBuffer[1], receiveBuffer[2]);
  SDK_TRACE_Timestamp(PRINT, 0);

  /* Place your code before here */
  /* Do not remove this code below */
  SDK_TRACE_Stop();
  /* Do not remove this code from above */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	SDK_TRACE_Timestamp(RX_CLB, 1);
	SDK_TRACE_Print("%s","rx callback");

    if (huart->Instance == USART2)
    { uartState |= 0x02; }

    if (huart->Instance == USART3)
    { uartState |= 0x08; }

    SDK_TRACE_Timestamp(RX_CLB, 0);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	SDK_TRACE_Timestamp(TX_CLB, 1);
	SDK_TRACE_Print("%s","tx callback");

    if (huart->Instance == USART2)
    { uartState |= 0x01; }

    if (huart->Instance == USART3)
    { uartState |= 0x04; }

	SDK_TRACE_Timestamp(TX_CLB, 0);
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

	SDK_TRACE_Print("%s","ERROR test");

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
