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
//////////////////////////////////////////////////the stm32 receives data from esp-now via lpuart1, and CAN data via usart1/////////////////////////////
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
    unsigned short SOC;
    float speedKmh;
    float odometerKm;
    float displaySpeed;
    uint8_t MacAddress[6];  // Sender's MAC
} Item;  // Ensure consistent packing
volatile Item receivedData;
//struct for collecting CAN Received CAN Frames:
typedef struct {
    uint32_t can_id;
    uint8_t dlc;
    uint8_t data[8];
} CANFrame;
volatile CANFrame CANreceivedData;
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_TIMEOUT_MS 100          // Reduced timeout for better responsiveness
#define UART_TX_INTERVAL 500         // 0.5 seconds transmission interval
#define UART2_TIMEOUT_MS 100         // Reduced timeout for better responsiveness
#define QUEUE_SIZE 10                // Increased queue size
#define PRINT_TIMEOUT_INTERVAL 500   // Print timeout every 500 iterations instead of 100
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
  .priority = (osPriority_t) osPriorityHigh,
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
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
static uint32_t uart_timeout_counter = 0;
static uint32_t message_counter = 0;
static uint32_t last_tx_time = 0;
static uint32_t uart_timeout_counter2 = 0;
static uint32_t last_tx_time2 = 0;
// Define the queue:
static QueueHandle_t UARTQueue;
static QueueHandle_t UARTQueue2;
// Pre-allocated buffers for better performance
static const uint8_t stm32_mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
// Global variable to store parsed CAN data
static Item parsedCANData;
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
static inline uint8_t ValidateReceivedData(const Item *data);
static inline uint8_t ValidateReceivedCANData(const CANFrame *data);
static void PrintMAC(const uint8_t *mac);
static void PrintCANData(const CANFrame *ReceivedCANFrame);
static void parseTwizyFrame(CANFrame* ReceivedCANFrame);
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//THIS IS TO VALIDATE RECEIVED DATA from ESP-NOW
static inline uint8_t ValidateReceivedData(const Item *data) {
    // Check if at least one byte of MAC address is non-zero
    for (int i = 0; i < 6; i++) {
        if (data->MacAddress[i] != 0) {
            return 1;
        }
    }
    return 0;
}
//THIS IS TO VALIDATE RECEIVED DATA from CAN
static inline uint8_t ValidateReceivedCANData(const CANFrame *data) {
    return (data->can_id != 0 && data->dlc <= 8);
}
// Helper function to print MAC address
static void PrintMAC(const uint8_t *mac) {
	//
	
    printf("%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
// Helper function to print CAN data
static void PrintCANData(const CANFrame *ReceivedCANFrame) {

    printf("Received from CAN Bus | CAN_ID: 0x%lX | DLC: %u | DATA:",
           ReceivedCANFrame->can_id, ReceivedCANFrame->dlc);
    for (int i = 0; i < ReceivedCANFrame->dlc; i++) {
        printf("%02X ", ReceivedCANFrame->data[i]);
    }
    printf("\n");
}

//Function to parse the CAN data and update parsedCANData:
static void parseTwizyFrame(CANFrame* ReceivedCANFrame) {
    switch (ReceivedCANFrame->can_id) {
        case 0x155:  // Battery SoC and Current
            if (ReceivedCANFrame->dlc >= 8) {
                uint16_t raw_soc = (ReceivedCANFrame->data[4] << 8) | ReceivedCANFrame->data[5];
                float soc = raw_soc / 400.0f;
                parsedCANData.SOC = (unsigned short)(soc * 100);  // Convert to percentage * 100
                printf("SoC: %.1f%%\r\n", soc);
            }
            break;
        case 0x5D7:  // Display Speed and Odometer
            if (ReceivedCANFrame->dlc >= 6) {
                uint16_t displaySpeedRaw = (ReceivedCANFrame->data[0] << 8) | ReceivedCANFrame->data[1];
                parsedCANData.displaySpeed = displaySpeedRaw / 100.0f;
                uint32_t odometerRaw = (ReceivedCANFrame->data[2] << 24) | (ReceivedCANFrame->data[3] << 16) |
                                       (ReceivedCANFrame->data[4] << 8) | ReceivedCANFrame->data[5];
                parsedCANData.odometerKm = odometerRaw / 1600.0f;
                printf("Display Speed: %.1f km/h | Odometer: %.1f km\r\n", parsedCANData.displaySpeed, parsedCANData.odometerKm);
            }
            break;
        case 0x19F:  // Motor Speed and RPM
            if (ReceivedCANFrame->dlc >= 4) {
                uint8_t byte2 = ReceivedCANFrame->data[2];
                uint8_t byte3 = ReceivedCANFrame->data[3];
                uint16_t speedRaw = (byte2 << 4) | (byte3 & 0x0F);
                int16_t speedDeviation = speedRaw - 0x7D0;
                float speedRPM = speedDeviation * 10.0f;
                parsedCANData.speedKmh = (speedRPM / 7250.0f) * 80.0f;
                printf("Motor Speed: %.1f km/h | RPM: %.0f\r\n", parsedCANData.speedKmh, speedRPM);
            }
            break;
        case 0x436:  // System Uptime
            if (ReceivedCANFrame->dlc >= 4) {
                uint32_t minuteCounter = (ReceivedCANFrame->data[0] << 24) | (ReceivedCANFrame->data[1] << 16) |
                                         (ReceivedCANFrame->data[2] << 8) | ReceivedCANFrame->data[3];
                uint32_t hours = minuteCounter / 60;
                uint32_t days = hours / 24;
                uint32_t remainingHours = hours % 24;
                printf("Uptime: %u min | %u h | %u d\r\n", minuteCounter, remainingHours, days);
            }
            break;
        case 0x423:  // Charger Status
            if (ReceivedCANFrame->dlc >= 8) {
                uint8_t chargerStatus = ReceivedCANFrame->data[0];
                uint16_t counter = (ReceivedCANFrame->data[6] << 8) | ReceivedCANFrame->data[7];
                printf("Charger: %s | Counter: %u\r\n",
                       chargerStatus == 0x03 ? "ON" : "OFF", counter);
            }
            break;
        case 0x424:  // Power and Battery Health
            if (ReceivedCANFrame->dlc >= 6) {
                int maxRegenPower = ReceivedCANFrame->data[2] * 500;
                int maxDrivePower = ReceivedCANFrame->data[3] * 500;
                int batterySOH = ReceivedCANFrame->data[5];
                printf("Max Drive: %d W | Regen: %d W | SOH: %d%%\r\n",
                       maxDrivePower, maxRegenPower, batterySOH);
            }
            break;
        case 0x425:  // Battery Voltage and Energy
            if (ReceivedCANFrame->dlc >= 6) {
                uint16_t voltageRaw = (ReceivedCANFrame->data[4] << 8) | ReceivedCANFrame->data[5];
                float batteryVoltage = (voltageRaw >> 1) / 10.0f;
                float availableEnergy = ReceivedCANFrame->data[1] / 10.0f;
                printf("Battery: %.1f V | Energy: %.1f kWh\r\n", batteryVoltage, availableEnergy);
            }
            break;
        case 0x597:  // 12V System
            if (ReceivedCANFrame->dlc >= 8) {
                float dcCurrent = ReceivedCANFrame->data[2] / 5.0f;
                int chargerTemp = ReceivedCANFrame->data[7] - 40;
                uint8_t protocol = ReceivedCANFrame->data[3];
                const char* proto_str = "UNK";
                switch (protocol) {
                    case 0x41: proto_str = "ON"; break;
                    case 0xD1: proto_str = "OFF"; break;
                    case 0xB1: proto_str = "CHG"; break;
                    case 0x91: proto_str = "STB"; break;
                }
                printf("12V: %.1f A | Temp: %d°C | Protocol: %s\r\n",
                       dcCurrent, chargerTemp, proto_str);
            }
            break;
        default:
            // Unknown frame - could log if needed
            break;
    }
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
  // Initialize parsed CAN data
  memset(&parsedCANData, 0, sizeof(Item));
  memcpy(parsedCANData.MacAddress, stm32_mac, 6);

  ////Indicates reception of data from ESP32(2) responsible for CAN sniffing
  ////Indicates reception of data from ESP32(1) responsible for esp-now communication
  ////Indicates transmission of data to ESP32(1) responsible for esp-now communication
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
  UARTQueue = xQueueCreate(QUEUE_SIZE, sizeof(Item));
  if (UARTQueue == NULL) {
      Error_Handler(); // Handle queue creation failure
  }
  UARTQueue2 = xQueueCreate(QUEUE_SIZE, sizeof(CANFrame));
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
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);
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
/* USER CODE END 4 */
/* USER CODE BEGIN Header_StartSenderTask */
/**
  * @brief  Function implementing the SenderTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSenderTask */
void StartSenderTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

  Item itemToSend;
  Item receivedItem;
  uint32_t current_time;
  /* Infinite loop */
  for(;;)
  {
		//
		
    // Check if we have received data to process from ESP-NOW
    if (xQueueReceive(UARTQueue, &receivedItem, 0) == pdTRUE) {
      BSP_LED_Toggle(LED_GREEN);
      printf("Received from ESP32 - SOC: %u, Speed: %.1f, MAC: ", receivedItem.SOC, receivedItem.speedKmh);
      PrintMAC(receivedItem.MacAddress);
      printf("\n");
      osDelay(50); // Reduced delay
    }

    // Send parsed CAN data to ESP32 periodically via LPUART1
    current_time = HAL_GetTick();
    if (current_time - last_tx_time >= UART_TX_INTERVAL) {
      // Prepare parsed CAN data to send
      itemToSend = parsedCANData;  // Copy the parsed data

      // Send struct data via LPUART1
      if (HAL_UART_Transmit(&hlpuart1, (uint8_t*)&itemToSend, sizeof(Item), 100) == HAL_OK) {
        printf("Sent to ESP32 - SOC: %u, Speed: %.1f km/h, Display: %.1f km/h, Odometer: %.1f km, MAC: ",
               itemToSend.SOC, itemToSend.speedKmh, itemToSend.displaySpeed, itemToSend.odometerKm);
        PrintMAC(itemToSend.MacAddress);
        printf("\n");
        BSP_LED_Toggle(LED_RED);
      } else {
        printf("Failed to send CAN data to ESP32\n");
      }
      last_tx_time = current_time;
    }
    osDelay(50);
  }
  /* USER CODE END 5 */
}
/* USER CODE BEGIN Header_StartTskUART */
/**
* @brief Function implementing the TskUART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTskUART */
void StartTskUART(void *argument)
{
  /* USER CODE BEGIN StartTskUART */
  /* Infinite loop */
  for(;;)
  {
	//
	
    // Receive Item struct data from ESP32 via LPUART1
    HAL_StatusTypeDef status = HAL_UART_Receive(&hlpuart1, (uint8_t *)&receivedData, sizeof(Item), UART_TIMEOUT_MS);
    if (status == HAL_OK) {
      uart_timeout_counter = 0;
      // Validate received data
      if (ValidateReceivedData(&receivedData)) {
        // Send valid data to queue
        if (xQueueSend(UARTQueue, &receivedData, 0) != pdTRUE) {
          printf("Queue full - message dropped\n");
        }
      } else {
        printf("UART Warning - Received corrupted data with invalid MAC\n");
      }
    }
    else if (status == HAL_TIMEOUT) {
      uart_timeout_counter++;
      if (uart_timeout_counter % PRINT_TIMEOUT_INTERVAL == 0) {
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
    osDelay(5); // Reduced delay for better responsiveness
  }
  /* USER CODE END StartTskUART */
}
/* USER CODE BEGIN Header_StartSenderTask2 */
/**
* @brief Function implementing the SenderTask2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSenderTask2 */
void StartSenderTask2(void *argument)
{
  /* USER CODE BEGIN StartSenderTask2 */
  /* Infinite loop */
  CANFrame receivedCANFrame;
  /* Infinite loop */
  for(;;)
  {
	//
	
    // Check if we have received CAN data to process
    if (xQueueReceive(UARTQueue2, &receivedCANFrame, 0) == pdTRUE) {
      // Parse the CAN frame and update parsedCANData
      parseTwizyFrame(&receivedCANFrame);
      PrintCANData(&receivedCANFrame);
    }
    osDelay(10);
  }
  /* USER CODE END StartSenderTask2 */
}
/* USER CODE BEGIN Header_StartTskUART2 */
/**
* @brief Function implementing the TskUART2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTskUART2 */
void StartTskUART2(void *argument)
{
  /* USER CODE BEGIN StartTskUART2 */
  /* Infinite loop */
  for(;;)
  {
	//
	
    // Receive CANFrame struct data from ESP32 via USART1
    HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, (uint8_t *)&CANreceivedData, sizeof(CANFrame), UART2_TIMEOUT_MS);
    if (status == HAL_OK) {
      BSP_LED_Toggle(LED_BLUE); // Indicate successful reception
      uart_timeout_counter2 = 0;
      // Validate received data
      if (ValidateReceivedCANData(&CANreceivedData)) {
        // Send valid data to queue
        if (xQueueSend(UARTQueue2, &CANreceivedData, 0) != pdTRUE) {
          printf("Queue2 full - message dropped\n");
        }
      } else {
        printf("UART Warning - Received corrupted data with invalid CAN Frame\n");
      }
    }
    else if (status == HAL_TIMEOUT) {
      uart_timeout_counter2++;
      if (uart_timeout_counter2 % PRINT_TIMEOUT_INTERVAL == 0) {
        printf("UART Waiting for data from ESP_CAN... (%lu)\n", uart_timeout_counter2);
      }
    }
    else {
      printf("UART Error from ESP_CAN: %d, resetting...\n", status);
      HAL_UART_DeInit(&huart1);
      osDelay(10);
      MX_USART1_UART_Init();
      uart_timeout_counter2 = 0;
    }
    osDelay(5); // Reduced delay for better responsiveness
  }
  /* USER CODE END StartTskUART2 */
}
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
