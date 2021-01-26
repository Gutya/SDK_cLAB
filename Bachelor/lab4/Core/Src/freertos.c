/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

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
/* USER CODE BEGIN Variables */
osThreadId stopTraceHandle;
osThreadId belowNormalHandle;
osThreadId aboveNormalTaskHandle;

osMessageQId queue01Handle;

uint32_t counter = 0;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osSemaphoreId myBinarySem01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void clabStopTraceTask(void const * argument);
void belowNormalTask(void const * argument);
void aboveNormalTask(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01 */
  osSemaphoreDef(myBinarySem01);
  myBinarySem01Handle = osSemaphoreCreate(osSemaphore(myBinarySem01), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  osMessageQDef(Queue01, 16, uint32_t);
  queue01Handle = osMessageCreate(osMessageQ(Queue01), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(Task2, clabStopTraceTask, osPriorityLow,0, 128);
  stopTraceHandle = osThreadCreate(osThread(Task2), NULL);

  osThreadDef(Task3, belowNormalTask, osPriorityBelowNormal,0, 128);
  stopTraceHandle = osThreadCreate(osThread(Task3), NULL);

  osThreadDef(Task4, aboveNormalTask, osPriorityAboveNormal,0, 128);
  stopTraceHandle = osThreadCreate(osThread(Task4), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* Infinite loop */
	for(;;)
	{
		vTaskDelay(197.1);
		SDK_TRACE_Timestamp(P2, 1);

		SDK_TRACE_Print("%s","MID - Entering task - counter++ - quit");
		counter++;

		SDK_TRACE_Timestamp(P2, 0);
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void clabStopTraceTask(void const * argument)
{

	for(;;)
	{
		vTaskDelay(20);

		if(counter > 3)
			SDK_TRACE_Stop();
	}
}

void belowNormalTask(void const * argument)
{
	uint32_t localCounter = 255;

	for(;;)
	{
		vTaskDelay(196);

		osSemaphoreWait(myBinarySem01Handle, osWaitForever);

		SDK_TRACE_Timestamp(P1, 1);
		SDK_TRACE_Print("%s","LOW - Entering task and !taking semaphore!");


		localCounter++;
		osMessagePut(queue01Handle, localCounter, 0);
		SDK_TRACE_Print("LOW - Queue put: %d", localCounter);

		SDK_TRACE_Print("%s","LOW <--- taskYeld\n");
		vTaskDelay(1);
		osThreadYield();
		SDK_TRACE_Print("%s","LOW - task Delay for 200ms before releasing semaphore\n");
		vTaskDelay(200);

		SDK_TRACE_Print("%s","!LOW - task releasing semaphore!\n");
		SDK_TRACE_Timestamp(P1, 0);

		osSemaphoreRelease(myBinarySem01Handle);
	}
}

void aboveNormalTask(void const * argument)
{

	for(;;)
	{
		vTaskDelay(198);
		SDK_TRACE_Timestamp(P4, 1);
		SDK_TRACE_Print("%s","!HIGH - task trying to access CS!");
		SDK_TRACE_Timestamp(P4, 0);
		osSemaphoreWait(myBinarySem01Handle, osWaitForever);

		SDK_TRACE_Print("%s","HIGH - Entering task and !taking semaphore!");
		SDK_TRACE_Timestamp(P3, 1);

		SDK_TRACE_Print("HIGH - Queue get: %d\n", osMessageGet(queue01Handle, 0).value.v);

		SDK_TRACE_Timestamp(P3, 0);

		osSemaphoreRelease(myBinarySem01Handle);
	}
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
