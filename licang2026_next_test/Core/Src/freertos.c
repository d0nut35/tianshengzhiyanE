/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "mux_service.h"
#include "test_config.h"
#include "chassis_mission_link.h"
#include "../01_App/mission/mission_app.h"

#if MULT_UART_FREERTOS_TEST_ENABLED
#include "mult_uart_freertos_test.h"
#endif

#if MULT_UART_MODULES_FREERTOS_TEST_ENABLED
#include "mult_uart_modules_freertos_test.h"
#endif

#if TURNTABLE_GRASP_LITE_FREERTOS_TEST_ENABLED
#include "turntable_grasp_lite_freertos_test.h"
#endif

#include "arm.h"

#if LSC16_FREERTOS_TEST_ENABLED
#include "lsc16_freertos_test.h"
#endif

#if IC_CARD_FREERTOS_TEST_ENABLED
#include "ic_card_service.h"
#include "ic_card_freertos_test.h"
#endif

#if ZDT_TURNTABLE_FREERTOS_TEST_ENABLED
#include "zdt_turntable_freertos_test.h"
#endif

#if LICANG_ACTIVE_TEST == LICANG_TEST_NONE
#include "chassis_bridge.h"
#endif

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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /*
   * CubeMX外设初始化已在main.c完成；此处只装配UART7复用底层并创建
   * worker，不启动任何测试或业务demo。任务真正运行要等osKernelStart()。
   */
#if !IC_CARD_FREERTOS_TEST_ENABLED && !ZDT_TURNTABLE_FREERTOS_TEST_ENABLED
  if (mux_init() != MULT_UART_OK)
  {
    Error_Handler();
  }

  /*
   * UART8舵控板与UART7复用器物理独立。机械臂Service在此完成装配，
   * 以验证动作组10(安全姿态)、11(识别姿态)和12(抓取放置)的单球闭环。
   */
  if (arm_init() != LSC16_OK)
  {
    Error_Handler();
  }
#elif IC_CARD_FREERTOS_TEST_ENABLED
  /* IC卡直连测试独占UART7，因此不能同时初始化复用总线的HAL adapter。 */
  if (ic_service_init() != IC_CARD_OK)
  {
    Error_Handler();
  }
#else
  /* ZDT直连测试在自己的测试装配中建立UART7 Service，不初始化其他所有者。 */
#endif

#if MULT_UART_FREERTOS_TEST_ENABLED
  /*
   * 测试任务只通过事务Service使用复用总线，不直接操作
   * HAL、DMA或A/B/EN。这可以同时验证完整FreeRTOS分层链路。
   */
  if (mult_uart_freertos_test_init() != MULT_UART_OK)
  {
    Error_Handler();
  }
#endif

#if LSC16_FREERTOS_TEST_ENABLED
  if (lsc16_freertos_test_init() != LSC16_OK)
  {
    Error_Handler();
  }
#endif

#if MULT_UART_MODULES_FREERTOS_TEST_ENABLED
  /* USART1命令任务通过复用Service测试通道1 IC卡和通道2 ZDT。 */
  if (mult_uart_modules_freertos_test_init() != MULT_UART_OK)
  {
    Error_Handler();
  }
#endif

#if TURNTABLE_GRASP_LITE_FREERTOS_TEST_ENABLED
  if (turntable_grasp_lite_freertos_test_init() != MULT_UART_OK)
  {
    Error_Handler();
  }
#endif

#if IC_CARD_FREERTOS_TEST_ENABLED
  if (ic_card_freertos_test_init() != IC_CARD_OK)
  {
    Error_Handler();
  }
#endif

#if ZDT_TURNTABLE_FREERTOS_TEST_ENABLED
  if (zdt_turntable_freertos_test_init() != ZDT_TURNTABLE_OK)
  {
    Error_Handler();
  }
#endif

#if LICANG_ACTIVE_TEST == LICANG_TEST_NONE
  /*
   * lhy底盘：内核启动前只登记huart2/3/4/5/6的UART路由；电机握手等阻塞装配
   * 在defaultTask中由chassis_bridge_boot()完成。测试场景下不编译，避免与
   * debug_uart1等夹具争抢UART。
   */
  if (chassis_bridge_init() != CHASSIS_BRIDGE_OK)
  {
    Error_Handler();
  }

  if (!chassis_mission_link_init())
  {
    Error_Handler();
  }

  if (mission_app_init() != MISSION_APP_OK)
  {
    Error_Handler();
  }
#endif

  /* USER CODE END Init */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;

#if LICANG_ACTIVE_TEST == LICANG_TEST_NONE
  /* app_init()阻塞到四轮握手完成，失败不影响其他子系统，因此不进Error_Handler。 */
  (void)chassis_bridge_boot();
#endif

  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

