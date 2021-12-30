/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f7xx_hal.h"
#include "ltdc.h"
#include "is42s32800g_conf.h"
#include "mt25ql512abb.h"
#include "quadspi.h"
#include "i2c.h"
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
/* USER CODE END Variables */
osThreadId heartbeatTaskHandle;
osThreadId lcdTestTaskHandle;
osThreadId sdramTestTaskHandle;
osThreadId flashTestTaskHandle;
osThreadId touchTestTaskHandle;
osThreadId touchgfxTaskHandle;
osMutexId mutexLCDHandle;
osMutexId mutexSDRAMHandle;
osMutexId mutexFLASHHandle;
osMutexId mutexTOUCHHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void execHeartbeatTask(void const * argument);
void execLcdTestTask(void const * argument);
void execSdramTestTask(void const * argument);
void execFlashTestTask(void const * argument);
void execTouchTestTask(void const * argument);
void execTouchgfxTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
  return HAL_GetTick();
}
/* USER CODE END 1 */

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
  /* Create the mutex(es) */
  /* definition and creation of mutexLCD */
  osMutexDef(mutexLCD);
  mutexLCDHandle = osMutexCreate(osMutex(mutexLCD));

  /* definition and creation of mutexSDRAM */
  osMutexDef(mutexSDRAM);
  mutexSDRAMHandle = osMutexCreate(osMutex(mutexSDRAM));

  /* definition and creation of mutexFLASH */
  osMutexDef(mutexFLASH);
  mutexFLASHHandle = osMutexCreate(osMutex(mutexFLASH));

  /* definition and creation of mutexTOUCH */
  osMutexDef(mutexTOUCH);
  mutexTOUCHHandle = osMutexCreate(osMutex(mutexTOUCH));

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
  /* definition and creation of heartbeatTask */
  osThreadDef(heartbeatTask, execHeartbeatTask, osPriorityAboveNormal, 0, 128);
  heartbeatTaskHandle = osThreadCreate(osThread(heartbeatTask), NULL);

  /* definition and creation of lcdTestTask */
  osThreadDef(lcdTestTask, execLcdTestTask, osPriorityBelowNormal, 0, 1024);
  lcdTestTaskHandle = osThreadCreate(osThread(lcdTestTask), NULL);

  /* definition and creation of sdramTestTask */
  osThreadDef(sdramTestTask, execSdramTestTask, osPriorityBelowNormal, 0, 1024);
  sdramTestTaskHandle = osThreadCreate(osThread(sdramTestTask), NULL);

  /* definition and creation of flashTestTask */
  osThreadDef(flashTestTask, execFlashTestTask, osPriorityBelowNormal, 0, 1024);
  flashTestTaskHandle = osThreadCreate(osThread(flashTestTask), NULL);

  /* definition and creation of touchTestTask */
  osThreadDef(touchTestTask, execTouchTestTask, osPriorityBelowNormal, 0, 512);
  touchTestTaskHandle = osThreadCreate(osThread(touchTestTask), NULL);

  /* definition and creation of touchgfxTask */
  osThreadDef(touchgfxTask, execTouchgfxTask, osPriorityLow, 0, 8192);
  touchgfxTaskHandle = osThreadCreate(osThread(touchgfxTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_execHeartbeatTask */
/**
  * @brief  Function implementing the heartbeatTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_execHeartbeatTask */
void execHeartbeatTask(void const * argument)
{
  /* USER CODE BEGIN execHeartbeatTask */
  printf("Starting heartbeatTask...\r\n");

  while(1)
  {
    HAL_GPIO_TogglePin(LED_GRN_GPIO_Port, LED_GRN_Pin);
    osDelay(500);
  }
  /* USER CODE END execHeartbeatTask */
}

/* USER CODE BEGIN Header_execLcdTestTask */
/**
* @brief Function implementing the lcdTestTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_execLcdTestTask */
void execLcdTestTask(void const * argument)
{
  /* USER CODE BEGIN execLcdTestTask */
  // Acquire LCD mutex
  while(osMutexWait(mutexLCDHandle, 0) != osOK)
  {
    osThreadYield();
  }

  printf("Starting lcdTestTask...\r\n");

  printf("Starting internal framebuffer test...\r\n");

  // Allocate framebuffer
  uint8_t *framebuffer = malloc(480*272*3);

  if(framebuffer == NULL)
  {
    printf("Could not allocate memory for framebuffer!\r\n");
    Error_Handler();
  }

  // Set LTDC framebuffer address
  if(HAL_LTDC_SetAddress(&hltdc, (uint32_t)framebuffer, LTDC_LAYER_1) != HAL_OK)
  {
    printf("Error setting framebuffer address!\r\n");
    Error_Handler();
  }

  // Turn on display
  HAL_GPIO_WritePin(LTDC_BL_GPIO_Port, LTDC_BL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LTDC_RST_GPIO_Port, LTDC_RST_Pin, GPIO_PIN_SET);

  // Write red to the display
  uint8_t *fb = framebuffer;

  while(fb < (uint8_t *)(framebuffer + (480*272*3)))
  {
    *fb++ = 0x00;   // Blue
    *fb++ = 0x00;   // Green
    *fb++ = 0xFF;   // Red
  }

  osDelay(1000);

  // Write green to the display
  fb = framebuffer;

  while(fb < (uint8_t *)(framebuffer + (480*272*3)))
  {
    *fb++ = 0x00;   // Blue
    *fb++ = 0xFF;   // Green
    *fb++ = 0x00;   // Red
  }

  osDelay(1000);

  // Write blue to the display
  fb = framebuffer;

  while(fb < (uint8_t *)(framebuffer + (480*272*3)))
  {
    *fb++ = 0xFF;   // Blue
    *fb++ = 0x00;   // Green
    *fb++ = 0x00;   // Red
  }

  osDelay(1000);

  // Write white to the display
  fb = framebuffer;

  while(fb < (uint8_t *)(framebuffer + (480*272*3)))
  {
    *fb++ = 0xFF;   // Blue
    *fb++ = 0xFF;   // Green
    *fb++ = 0xFF;   // Red
  }

  osDelay(1000);

  // Turn off display
  HAL_GPIO_WritePin(LTDC_BL_GPIO_Port, LTDC_BL_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LTDC_RST_GPIO_Port, LTDC_RST_Pin, GPIO_PIN_RESET);

  printf("Ending lcdTestTask...\r\n");

  free(framebuffer);
  osMutexRelease(mutexLCDHandle);
  osThreadTerminate(lcdTestTaskHandle);
  /* USER CODE END execLcdTestTask */
}

/* USER CODE BEGIN Header_execSdramTestTask */
/**
* @brief Function implementing the sdramTestTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_execSdramTestTask */
void execSdramTestTask(void const * argument)
{
  /* USER CODE BEGIN execSdramTestTask */
  volatile uint32_t *externalRAM = (uint32_t *)IS42S32800G_ADDRESS;
  const uint32_t sizeRAM = IS42S32800G_SIZE;
  uint32_t i = 0;
  int32_t before, after;
  float elapsed;

  // Acquire SDRAM mutex
  while(osMutexWait(mutexSDRAMHandle, 0) != osOK)
  {
    osThreadYield();
  }

  printf("Starting sdramTestTask...\r\n");

  // Write Test
  before = HAL_GetTick();
  while(i < (sizeRAM/4))
  {
      *(externalRAM + i) = i;
      i++;
  }
  after = HAL_GetTick();

  elapsed = ((after - before) / 1000.0);

  printf("RAM - Wrote %ld words in %3.2f seconds (%3.2fMBs)\n",
          i, elapsed, ((((i * 4) / 1024) / 1024) / elapsed) );

  // Read Test
  i = 0;

  before = HAL_GetTick();
  while(i < (sizeRAM/4))
  {
      if(*(externalRAM + i) != i)
      {
          break;
      }
      else
      {
          i++;
      }
  }
  after = HAL_GetTick();

  elapsed = ((after - before) / 1000.0);

  printf("RAM - Read %ld words in %3.2f seconds (%3.2fMBs)\n",
          i, elapsed, ((((i * 4) / 1024) / 1024) / elapsed) );

  // Framebuffer in external SDRAM

  // Acquire LCD mutex
  while(osMutexWait(mutexLCDHandle, 0) != osOK)
  {
    osThreadYield();
  }

  printf("Starting external framebuffer test...\r\n");

  uint8_t *framebuffer = (uint8_t *)0xC0000000;

  if(HAL_LTDC_SetAddress(&hltdc, (uint32_t)framebuffer, LTDC_LAYER_1) != HAL_OK)
  {
    printf("Error changing LTDC framebuffer address!");
    Error_Handler();
  }

  // Turn on display
  HAL_GPIO_WritePin(LTDC_BL_GPIO_Port, LTDC_BL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LTDC_RST_GPIO_Port, LTDC_RST_Pin, GPIO_PIN_SET);

  // Write red to the display
  uint8_t *fb = framebuffer;

  while(fb < (uint8_t *)(framebuffer + (480*272*3)))
  {
    *fb++ = 0x00;   // Blue
    *fb++ = 0x00;   // Green
    *fb++ = 0xFF;   // Red
  }

  osDelay(1000);

  // Write green to the display
  fb = framebuffer;

  while(fb < (uint8_t *)(framebuffer + (480*272*3)))
  {
    *fb++ = 0x00;   // Blue
    *fb++ = 0xFF;   // Green
    *fb++ = 0x00;   // Red
  }

  osDelay(1000);

  // Write blue to the display
  fb = framebuffer;

  while(fb < (uint8_t *)(framebuffer + (480*272*3)))
  {
    *fb++ = 0xFF;   // Blue
    *fb++ = 0x00;   // Green
    *fb++ = 0x00;   // Red
  }

  osDelay(1000);

  // Write white to the display
  fb = framebuffer;

  while(fb < (uint8_t *)(framebuffer + (480*272*3)))
  {
    *fb++ = 0xFF;   // Blue
    *fb++ = 0xFF;   // Green
    *fb++ = 0xFF;   // Red
  }

  osDelay(1000);

  // Turn off display
  HAL_GPIO_WritePin(LTDC_BL_GPIO_Port, LTDC_BL_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LTDC_RST_GPIO_Port, LTDC_RST_Pin, GPIO_PIN_RESET);

  printf("Ending sdramTestTask...\r\n");
  osMutexRelease(mutexLCDHandle);
  osMutexRelease(mutexSDRAMHandle);
  osThreadTerminate(sdramTestTaskHandle);
  /* USER CODE END execSdramTestTask */
}

/* USER CODE BEGIN Header_execFlashTestTask */
/**
* @brief Function implementing the flashTestTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_execFlashTestTask */
void execFlashTestTask(void const * argument)
{
  /* USER CODE BEGIN execFlashTestTask */
  while(osMutexWait(mutexFLASHHandle, 0) != osOK)
  {
    osThreadYield();
  }

  printf("Starting flashTestTask...\r\n");

  uint32_t var;
  int32_t before, after;
  float elapsed;
  uint8_t *buffer_test;

  // Create test data
  do
  {
    buffer_test = malloc((size_t)MT25QL512ABB_SECTOR_64K);

    if(buffer_test == NULL)
    {
      osDelay(10);
    }
  }
  while(buffer_test == NULL);

  for (var = 0; var < MT25QL512ABB_SECTOR_64K; var++)
  {
    buffer_test[var] = (var & 0xff);
  }

  // Exit memory mapped mode
  if(HAL_QSPI_Abort(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }

  // Erase Sector
  before = HAL_GetTick();
  if(MT25QL512ABB_BlockErase(&hqspi,
                                MT25QL512ABB_QPI_MODE,
                                MT25QL512ABB_4BYTES_SIZE,
                                1023 * MT25QL512ABB_SECTOR_64K,
                                MT25QL512ABB_ERASE_64K)
      != MT25QL512ABB_OK)
  {
    Error_Handler();
  }

  // Poll WIP
  if(MT25QL512ABB_AutoPollingMemReady(&hqspi,
                                        MT25QL512ABB_QPI_MODE,
                                        MT25QL512ABB_DUALFLASH_DISABLE)
      != MT25QL512ABB_OK)
  {
    Error_Handler();
  }

  // Write test data
  for(int i = 0; i < MT25QL512ABB_SECTOR_64K /  MT25QL512ABB_PAGE_SIZE; i++)
  {
    // Set WIP
    if(MT25QL512ABB_WriteEnable(&hqspi,
                                  MT25QL512ABB_QPI_MODE,
                                  MT25QL512ABB_DUALFLASH_DISABLE)
        != MT25QL512ABB_OK)

    {
      Error_Handler();
    }
    // Program page
    if(MT25QL512ABB_PageProgram(&hqspi,
                                  MT25QL512ABB_QPI_MODE,
                                  MT25QL512ABB_4BYTES_SIZE,
                                  (buffer_test + (MT25QL512ABB_PAGE_SIZE * i)),
                                  ((1023 * MT25QL512ABB_SECTOR_64K) + (MT25QL512ABB_PAGE_SIZE * i)),
                                  MT25QL512ABB_PAGE_SIZE)
        != MT25QL512ABB_OK)
    {
      Error_Handler();
    }
    // Poll WIP
    if(MT25QL512ABB_AutoPollingMemReady(&hqspi,
                                          MT25QL512ABB_QPI_MODE,
                                          MT25QL512ABB_DUALFLASH_DISABLE)
        != MT25QL512ABB_OK)
    {
      Error_Handler();
    }
  }
  after = HAL_GetTick();

  elapsed = ((after - before) / 1000.0);

  printf("FLASH - Wrote %ld bytes in %3.2f seconds (%3.2fKBs)\n",
      (uint32_t)MT25QL512ABB_SECTOR_64K, elapsed, (float)(((MT25QL512ABB_SECTOR_64K) / 1024) / elapsed) );

  // Put back into memory mapped mode
  if(MT25QL512ABB_EnableMemoryMappedModeSTR(&hqspi,
      MT25QL512ABB_QPI_MODE,
      MT25QL512ABB_4BYTES_SIZE)
  != MT25QL512ABB_OK)
  {
    Error_Handler();
  }

  // Check that written data matches
  before = HAL_GetTick();
  if(memcmp(buffer_test, (uint8_t *)(0x90000000 + (1023 * MT25QL512ABB_SECTOR_64K)), MT25QL512ABB_SECTOR_64K))
  {
    Error_Handler();
  }
  after = HAL_GetTick();

  elapsed = ((after - before) / 1000.0);

  printf("FLASH - Read %ld bytes in %3.2f seconds (%3.2fKBs)\n",
        (uint32_t)MT25QL512ABB_SECTOR_64K, elapsed, (float)(((MT25QL512ABB_SECTOR_64K) / 1024) / elapsed) );


  printf("Ending flashTestTask...\r\n");
  free(buffer_test);
  osMutexRelease(mutexFLASHHandle);
  osThreadTerminate(flashTestTaskHandle);
  /* USER CODE END execFlashTestTask */
}

/* USER CODE BEGIN Header_execTouchTestTask */
/**
* @brief Function implementing the touchTestTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_execTouchTestTask */
void execTouchTestTask(void const * argument)
{
  /* USER CODE BEGIN execTouchTestTask */
  while(osMutexWait(mutexTOUCHHandle, 0) != osOK)
  {
    osThreadYield();
  }

  printf("Starting touchTestTask...\r\n");



  printf("Ending touchTestTask...\r\n");
  osMutexRelease(mutexTOUCHHandle);
  osThreadTerminate(touchTestTaskHandle);
  /* USER CODE END execTouchTestTask */
}

/* USER CODE BEGIN Header_execTouchgfxTask */
/**
* @brief Function implementing the touchgfxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_execTouchgfxTask */
void execTouchgfxTask(void const * argument)
{
  /* USER CODE BEGIN execTouchgfxTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END execTouchgfxTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

