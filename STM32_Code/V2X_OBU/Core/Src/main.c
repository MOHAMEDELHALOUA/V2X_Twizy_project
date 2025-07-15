/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "queue.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    unsigned int value;
    uint8_t MacAddress[6];  // Sender's MAC
} Item;  // Ensure consistent packing


volatile Item receivedData;

//struct for collecting CANframes:
typedef struct {
    uint32_t can_id;
    uint8_t dlc;
    uint8_t data[8];
} CANFrame;

volatile CANFrame CANreceivedData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_TIMEOUT_MS 1000
#define UART_TX_INTERVAL 2000   // 0.25 seconds transmission interval
#define UART2_TIMEOUT_MS 1000
#define UART2_TX_INTERVAL 2000   // 0.025 seconds transmission interval
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart1;

/* Definitions for SenderTask */
osThreadId_t SenderTaskHandle;
const osThreadAttr_t SenderTask_attributes = {
  .name = "SenderTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for TskUART */
osThreadId_t TskUARTHandle;
const osThreadAttr_t TskUART_attributes = {
  .name = "TskUART",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for SenderTask2 */
osThreadId_t SenderTask2Handle;
const osThreadAttr_t SenderTask2_attributes = {
  .name = "SenderTask2",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for TskUART2 */
osThreadId_t TskUART2Handle;
const osThreadAttr_t TskUART2_attributes = {
  .name = "TskUART2",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
uint32_t uart_timeout_counter = 0;
uint32_t message_counter = 0;
uint32_t last_tx_time = 0;

uint32_t uart_timeout_counter2 = 0;
//uint32_t message_counter2 = 0;
uint32_t last_tx_time2 = 0;
// Define the queue:
xQueueHandle UARTQueue;
xQueueHandle UARTQueue2;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_LPUART1_UART_Init(void);
void StartSenderTask(void *argument);
void StartTskUART(void *argument);
void StartSenderTask2(void *argument);
void StartTskUART2(void *argument);

/* USER CODE BEGIN PFP */
uint8_t ValidateReceivedData(Item *data); 

uint8_t ValidateReceivedCANData(CANFrame *data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//THIS IS TO VALIDATE RECEIVED DATA frome ESP-NOW 
uint8_t ValidateReceivedData(Item *data) {
    // Check if at least one byte of MAC address is non-zero
    for (int i = 0; i < 6; i++) {
        if (data->MacAddress[i] != 0) {
            return 1;
        }
    }
    return 0;
}

//THIS IS TO VALIDATE RECEIVED DATA frome CAN
uint8_t ValidateReceivedCANData(CANFrame *data) {
    if (data->can_id == 0 || data->dlc > 8) {
        return 0;  // Invalid
    }
    return 1; // Accept
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_LPUART1_UART_Init();
  /* USER CODE BEGIN 2 */

  printf("STM32 will start receiving Item struct data via USART and LPUART...\n");
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  UARTQueue = xQueueCreate(10, sizeof(Item));  // Increased queue size
  if (UARTQueue == NULL) {
      Error_Handler(); // Handle queue creation failure
  }
  UARTQueue2 = xQueueCreate(10, sizeof(CANFrame));  // Increased queue size
  if (UARTQueue2 == NULL) {
      Error_Handler(); // Handle queue creation failure
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of SenderTask */
  SenderTaskHandle = osThreadNew(StartSenderTask, NULL, &SenderTask_attributes);

  /* creation of TskUART */
  TskUARTHandle = osThreadNew(StartTskUART, NULL, &TskUART_attributes);

  /* creation of SenderTask2 */
  SenderTask2Handle = osThreadNew(StartSenderTask2, NULL, &SenderTask2_attributes);

  /* creation of TskUART2 */
  TskUART2Handle = osThreadNew(StartTskUART2, NULL, &TskUART2_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  /* Initialize leds */
  BSP_LED_Init(LED_BLUE);//Indicates received data from CAN bus via usart1rx
  BSP_LED_Init(LED_GREEN);//Indicates received data from esp-now ESP32 via lpuart1 rx
  BSP_LED_Init(LED_RED);//Indicates transmitted data to esp-now ESP32 via lpuart1 tx 
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */
  /* USER CODE END Smps */
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
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_USB;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : B2_Pin B3_Pin */
  GPIO_InitStruct.Pin = B2_Pin|B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  (void)file;
  int DataIdx;
  for (DataIdx = 0; DataIdx < len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}

/////////////////////////////////////////////// Code to receive ESP-now data ////////////////////////////////////////////

void StartSenderTask(void *argument)
{
  Item itemToSend;
  Item receivedItem;

  /* Infinite loop */
  for(;;)
  {
    // Check if we have received data to process
    if (xQueueReceive(UARTQueue, &receivedItem, 0) == pdTRUE) {
      printf("Received from ESP32 - Value: %u, MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
             receivedItem.value,
             receivedItem.MacAddress[0], receivedItem.MacAddress[1], receivedItem.MacAddress[2],
             receivedItem.MacAddress[3], receivedItem.MacAddress[4], receivedItem.MacAddress[5]);
      osDelay(100);
    }

    // Send data to ESP32 periodically
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_tx_time >= UART_TX_INTERVAL) {
      // Prepare data to send
      itemToSend.value = ++message_counter;
      uint8_t stm32_mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}; // Dummy STM32 MAC
      memcpy(itemToSend.MacAddress, stm32_mac, 6);

      // Send struct data
      HAL_StatusTypeDef status = HAL_UART_Transmit(&hlpuart1, (uint8_t*)&itemToSend, sizeof(Item), 1000);

      if (status == HAL_OK) {
        BSP_LED_Toggle(LED_RED);
        printf("Sent to ESP32 - Value: %u, MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
               itemToSend.value,
               itemToSend.MacAddress[0], itemToSend.MacAddress[1], itemToSend.MacAddress[2],
               itemToSend.MacAddress[3], itemToSend.MacAddress[4], itemToSend.MacAddress[5]);
      } else {
        printf("Failed to send data to ESP32\n");
      }

      last_tx_time = current_time;
    }

    osDelay(50);
  }
}

void StartTskUART(void *argument)
{
  for(;;)
  {
    // Receive Item struct data from ESP32
    HAL_StatusTypeDef status = HAL_UART_Receive(&hlpuart1, (uint8_t *)&receivedData, sizeof(Item), UART_TIMEOUT_MS);

    if (status == HAL_OK) {
      uart_timeout_counter = 0;

      // Validate received data
      if (ValidateReceivedData((Item*)&receivedData)) {
        // Send valid data to queue
        if (xQueueSend(UARTQueue, &receivedData, 0) != pdTRUE) {
          printf("Queue full - message dropped\n");
        }
        BSP_LED_Toggle(LED_GREEN); // Indicate successful reception
      } else {
        printf("UART Warning - Received corrupted data with invalid MAC\n");
      }
    }
    else if (status == HAL_TIMEOUT) {
      uart_timeout_counter++;
      if (uart_timeout_counter % 100 == 0) {  // Reduced frequency of timeout2 messages
        printf("UART Waiting for data... (%lu)\n", uart_timeout_counter);
      }
    }
    else {
      printf("UART Error: %d, resetting...\n", status);
      HAL_UART_DeInit(&hlpuart1);
      osDelay(10);
      MX_LPUART1_UART_Init();
      uart_timeout_counter = 0;
    }
    osDelay(10); // Small delay
  }
}


/////////////////////////////////////// Tasks to receive CAN Bus data ( CAN IDs, Data length, and Data) /////////////////
void StartSenderTask2(void *argument)
{
  Item itemToSend;
  CANFrame receivedCANFrame;

  /* Infinite loop */
  for(;;)
  {
    // Check if we have received data to process
    if (xQueueReceive(UARTQueue2, &receivedCANFrame, 0) == pdTRUE) {
      printf("Received from CAN Bus | CAN_ID: 0x%lX | DLC: %u | DATA:",receivedCANFrame.can_id,receivedCANFrame.dlc);
      for (int i = 0; i < receivedCANFrame.dlc; i++) {
          printf("%02X ", receivedCANFrame.data[i]);
      }
      printf("\n");
      osDelay(100);
    }
  }
}

void StartTskUART2(void *argument)
{
  for(;;)
  {
    // Receive CANFrame struct data from ESP32
    HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, (uint8_t *)&CANreceivedData, sizeof(CANFrame), UART2_TIMEOUT_MS);
    if (status == HAL_OK) {
      uart_timeout_counter2 = 0;
      // Validate received data
      if (ValidateReceivedCANData((CANFrame*)&CANreceivedData)) {
        // Send valid data to queue
        if (xQueueSend(UARTQueue2, (const void *)&CANreceivedData, 0) != pdTRUE) {
          printf("Queue full - message dropped\n");
        }
        BSP_LED_Toggle(LED_BLUE); // Indicate successful reception
      } else {
        printf("UART Warning - Received corrupted data with invalid CAN frame\n");
      }
    }
    else if (status == HAL_TIMEOUT) {
      uart_timeout_counter2++;
      if (uart_timeout_counter2 % 100 == 0) {  // Reduced frequency of timeout2 messages
        printf("UART Waiting for data... (%lu)\n", uart_timeout_counter2);
      }
    }
    else {
      printf("UART Error: %d, resetting...\n", status);
      HAL_UART_DeInit(&huart1);
      osDelay(10);
      MX_USART1_UART_Init();
      uart_timeout_counter2 = 0;
    }
    osDelay(10); // Small delay
  }
}
/* USER CODE END 4 */



/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
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
#ifdef USE_FULL_ASSERT
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
