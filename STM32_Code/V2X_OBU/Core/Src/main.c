/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Optimized with Interrupts & 64MHz
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
#define UART_TX_INTERVAL 500         // 0.5 seconds transmission interval
#define QUEUE_SIZE 20                // Increased queue size for interrupts
#define DEBUG_PRINT_INTERVAL 100     // Print every 100th message for performance
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
  .stack_size = 256 * 4  // Increased stack size
};
/* Definitions for TskUART */
osThreadId_t TskUARTHandle;
const osThreadAttr_t TskUART_attributes = {
  .name = "TskUART",
  .priority = (osPriority_t) osPriorityLow,  // Lower priority since interrupts handle reception
  .stack_size = 256 * 4
};
/* Definitions for SenderTask2 */
osThreadId_t SenderTask2Handle;
const osThreadAttr_t SenderTask2_attributes = {
  .name = "SenderTask2",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};
/* Definitions for TskUART2 */
osThreadId_t TskUART2Handle;
const osThreadAttr_t TskUART2_attributes = {
  .name = "TskUART2",
  .priority = (osPriority_t) osPriorityLow,  // Lower priority since interrupts handle reception
  .stack_size = 256 * 4
};
/* USER CODE BEGIN PV */
static uint32_t last_tx_time = 0;
static uint32_t debug_counter = 0;
// Define the queue:
static QueueHandle_t UARTQueue;
static QueueHandle_t UARTQueue2;
// Pre-allocated buffers for better performance
static const uint8_t stm32_mac[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
// Global variable to store parsed CAN data
static Item parsedCANData;
// Interrupt status flags
static volatile uint32_t uart_error_count = 0;
static volatile uint32_t can_error_count = 0;
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
static void StartUARTInterrupts(void);
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
    printf("%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}
// Helper function to print CAN data
static void PrintCANData(const CANFrame *ReceivedCANFrame) {
    if (debug_counter % DEBUG_PRINT_INTERVAL == 0) {
        printf("CAN ID:0x%lX DLC:%u DATA:", ReceivedCANFrame->can_id, ReceivedCANFrame->dlc);
        for (int i = 0; i < ReceivedCANFrame->dlc; i++) {
            printf("%02X ", ReceivedCANFrame->data[i]);
        }
        printf("\n");
    }
}
//Function to parse the CAN data and update parsedCANData:
static void parseTwizyFrame(CANFrame* ReceivedCANFrame) {
    switch (ReceivedCANFrame->can_id) {
        case 0x155:  // Battery SoC and Current
            if (ReceivedCANFrame->dlc >= 8) {
                uint16_t raw_soc = (ReceivedCANFrame->data[4] << 8) | ReceivedCANFrame->data[5];
                float soc = raw_soc / 400.0f;
                parsedCANData.SOC = (unsigned short)(soc * 100);  // Convert to percentage * 100
                if (debug_counter % (DEBUG_PRINT_INTERVAL * 2) == 0) {
                    printf("SoC: %.1f%%\n", soc);
                }
            }
            break;
        case 0x5D7:  // Display Speed and Odometer
            if (ReceivedCANFrame->dlc >= 6) {
                uint16_t displaySpeedRaw = (ReceivedCANFrame->data[0] << 8) | ReceivedCANFrame->data[1];
                parsedCANData.displaySpeed = displaySpeedRaw / 100.0f;
                uint32_t odometerRaw = (ReceivedCANFrame->data[2] << 24) | (ReceivedCANFrame->data[3] << 16) |
                                       (ReceivedCANFrame->data[4] << 8) | ReceivedCANFrame->data[5];
                parsedCANData.odometerKm = odometerRaw / 1600.0f;
                if (debug_counter % (DEBUG_PRINT_INTERVAL * 2) == 0) {
                    printf("Display: %.1f km/h | Odometer: %.1f km\n", parsedCANData.displaySpeed, parsedCANData.odometerKm);
                }
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
                if (debug_counter % (DEBUG_PRINT_INTERVAL * 2) == 0) {
                    printf("Motor: %.1f km/h | RPM: %.0f\n", parsedCANData.speedKmh, speedRPM);
                }
            }
            break;
        case 0x436:  // System Uptime
            if (ReceivedCANFrame->dlc >= 4) {
                uint32_t minuteCounter = (ReceivedCANFrame->data[0] << 24) | (ReceivedCANFrame->data[1] << 16) |
                                         (ReceivedCANFrame->data[2] << 8) | ReceivedCANFrame->data[3];
                uint32_t hours = minuteCounter / 60;
                uint32_t days = hours / 24;
                uint32_t remainingHours = hours % 24;
                if (debug_counter % (DEBUG_PRINT_INTERVAL * 5) == 0) {
                    printf("Uptime: %u min | %u h | %u d\n", minuteCounter, remainingHours, days);
                }
            }
            break;
        case 0x423:  // Charger Status
            if (ReceivedCANFrame->dlc >= 8) {
                uint8_t chargerStatus = ReceivedCANFrame->data[0];
                uint16_t counter = (ReceivedCANFrame->data[6] << 8) | ReceivedCANFrame->data[7];
                if (debug_counter % (DEBUG_PRINT_INTERVAL * 3) == 0) {
                    printf("Charger: %s | Counter: %u\n", chargerStatus == 0x03 ? "ON" : "OFF", counter);
                }
            }
            break;
        case 0x424:  // Power and Battery Health
            if (ReceivedCANFrame->dlc >= 6) {
                int maxRegenPower = ReceivedCANFrame->data[2] * 500;
                int maxDrivePower = ReceivedCANFrame->data[3] * 500;
                int batterySOH = ReceivedCANFrame->data[5];
                if (debug_counter % (DEBUG_PRINT_INTERVAL * 3) == 0) {
                    printf("Max Drive: %d W | Regen: %d W | SOH: %d%%\n", maxDrivePower, maxRegenPower, batterySOH);
                }
            }
            break;
        case 0x425:  // Battery Voltage and Energy
            if (ReceivedCANFrame->dlc >= 6) {
                uint16_t voltageRaw = (ReceivedCANFrame->data[4] << 8) | ReceivedCANFrame->data[5];
                float batteryVoltage = (voltageRaw >> 1) / 10.0f;
                float availableEnergy = ReceivedCANFrame->data[1] / 10.0f;
                if (debug_counter % (DEBUG_PRINT_INTERVAL * 3) == 0) {
                    printf("Battery: %.1f V | Energy: %.1f kWh\n", batteryVoltage, availableEnergy);
                }
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
                if (debug_counter % (DEBUG_PRINT_INTERVAL * 4) == 0) {
                    printf("12V: %.1f A | Temp: %d°C | Protocol: %s\n", dcCurrent, chargerTemp, proto_str);
                }
            }
            break;
        default:
            // Unknown frame - could log if needed
            break;
    }
}

// Start interrupt-based UART reception
static void StartUARTInterrupts(void) {
    // Enable NVIC interrupts for UART
    HAL_NVIC_SetPriority(LPUART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(LPUART1_IRQn);

    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    // Start interrupt-based reception
    HAL_UART_Receive_IT(&hlpuart1, (uint8_t*)&receivedData, sizeof(Item));
    HAL_UART_Receive_IT(&huart1, (uint8_t*)&CANreceivedData, sizeof(CANFrame));

    printf("UART Interrupts enabled - 64MHz System Ready\n");
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

  // Start interrupt-based UART reception
  StartUARTInterrupts();

  printf("STM32 64MHz + Interrupt UART System Started\n");
  printf("System Clock: %lu Hz\n", HAL_RCC_GetSysClockFreq());
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
  * @brief System Clock Configuration - 64MHz
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;  // 4MHz MSI

  /** Configure PLL for 64MHz */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;    // Divider: 4MHz / 1 = 4MHz
  RCC_OscInitStruct.PLL.PLLN = 32;   // Multiplier: 4MHz * 32 = 128MHz
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;  // Output: 128MHz / 2 = 64MHz

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;  // Use PLL for 64MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        // 64MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;         // 64MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;         // 64MHz
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;       // 32MHz for RF
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;       // 64MHz

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
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

// UART Reception Complete Callback - CRITICAL FOR INTERRUPT OPERATION
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == LPUART1) {
        // ESP-NOW data received via LPUART1
        if (ValidateReceivedData((Item*)&receivedData)) {
            if (xQueueSendFromISR(UARTQueue, (void*)&receivedData, &xHigherPriorityTaskWoken) != pdTRUE) {
                // Queue full - increment error counter
                uart_error_count++;
            }
        }
        // Restart reception for next packet
        HAL_UART_Receive_IT(&hlpuart1, (uint8_t*)&receivedData, sizeof(Item));
    }
    else if (huart->Instance == USART1) {
        // CAN data received via USART1
        if (ValidateReceivedCANData((CANFrame*)&CANreceivedData)) {
            if (xQueueSendFromISR(UARTQueue2, (void*)&CANreceivedData, &xHigherPriorityTaskWoken) != pdTRUE) {
                // Queue full - increment error counter
                can_error_count++;
            }
        }
        // Restart reception for next packet
        HAL_UART_Receive_IT(&huart1, (uint8_t*)&CANreceivedData, sizeof(CANFrame));
    }

    // Trigger context switch if higher priority task was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// UART Error Callback - CRITICAL FOR ROBUST OPERATION
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1) {
        // Clear error flags and restart reception
        __HAL_UART_CLEAR_OREFLAG(&hlpuart1);
        __HAL_UART_CLEAR_NEFLAG(&hlpuart1);
        __HAL_UART_CLEAR_FEFLAG(&hlpuart1);
        __HAL_UART_CLEAR_PEFLAG(&hlpuart1);
        uart_error_count++;
        HAL_UART_Receive_IT(&hlpuart1, (uint8_t*)&receivedData, sizeof(Item));
    }
    else if (huart->Instance == USART1) {
        // Clear error flags and restart reception
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        __HAL_UART_CLEAR_NEFLAG(&huart1);
        __HAL_UART_CLEAR_FEFLAG(&huart1);
        __HAL_UART_CLEAR_PEFLAG(&huart1);
        can_error_count++;
        HAL_UART_Receive_IT(&huart1, (uint8_t*)&CANreceivedData, sizeof(CANFrame));
    }
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
  Item itemToSend;
  Item receivedItem;
  uint32_t current_time;

  for(;;)
  {
    debug_counter++;

    // Check if we have received data to process from ESP-NOW
    if (xQueueReceive(UARTQueue, &receivedItem, 0) == pdTRUE) {
      BSP_LED_Toggle(LED_GREEN);

      // Reduce printf frequency for better performance
      if (debug_counter % DEBUG_PRINT_INTERVAL == 0) {
          printf("RX ESP32 - SOC: %u, Speed: %.1f, MAC: ", receivedItem.SOC, receivedItem.speedKmh);
          PrintMAC(receivedItem.MacAddress);
          printf("\n");
      }
    }
    // Send parsed CAN data to ESP32 periodically via LPUART1
    current_time = HAL_GetTick();
    if (current_time - last_tx_time >= UART_TX_INTERVAL) {
      itemToSend = parsedCANData;  // Copy the parsed data

      if (HAL_UART_Transmit(&hlpuart1, (uint8_t*)&itemToSend, sizeof(Item), 100) == HAL_OK) {
        if (debug_counter % DEBUG_PRINT_INTERVAL == 0) {
            printf("TX ESP32 - SOC: %u, Speed: %.1f km/h, Display: %.1f km/h, Odometer: %.1f km\n",
                   itemToSend.SOC, itemToSend.speedKmh, itemToSend.displaySpeed, itemToSend.odometerKm);
        }
        BSP_LED_Toggle(LED_RED);
      } else {
        if (debug_counter % (DEBUG_PRINT_INTERVAL * 5) == 0) {
            printf("Failed to send CAN data to ESP32\n");
        }
      }
      last_tx_time = current_time;
    }

    osDelay(20); // Reduced delay for better performance
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
  /* This task is now much simpler since interrupts handle reception */
  for(;;)
  {
    // Just handle monitoring and status reporting
    // Most work is now done in interrupt callbacks

    // Print status occasionally
    if (debug_counter % (DEBUG_PRINT_INTERVAL * 50) == 0) {
        printf("LPUART1 Status: OK | Errors: %lu\n", uart_error_count);
    }

    osDelay(500); // Much longer delay since interrupts handle reception
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
  CANFrame receivedCANFrame;

  for(;;)
  {
    // Check if we have received CAN data to process
    if (xQueueReceive(UARTQueue2, &receivedCANFrame, 0) == pdTRUE) {
      // Parse the CAN frame and update parsedCANData
      parseTwizyFrame(&receivedCANFrame);

      // Print CAN data with reduced frequency
      PrintCANData(&receivedCANFrame);
    }
    osDelay(20); // Reduced delay for better responsiveness
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
  /* This task is now much simpler since interrupts handle reception */
  for(;;)
  {
    // Just handle monitoring and status reporting
    // Most work is now done in interrupt callbacks

    // Print status occasionally
    if (debug_counter % (DEBUG_PRINT_INTERVAL * 50) == 0) {
        printf("USART1 Status: OK | Errors: %lu\n", can_error_count);
    }

    osDelay(500); // Much longer delay since interrupts handle reception
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
