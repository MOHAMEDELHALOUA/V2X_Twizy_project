/////////////////////////////////////////////code for testing uart communication esp32(2)---esp-now--->esp32(1)---uart--->stm32
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "string.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "nvs_flash.h"

// Pin definitions
#define UART_0_TX 17
#define UART_0_RX 16
#define UART_NUM UART_NUM_1
#define LED_PIN GPIO_NUM_2

// Buffer size
#define BUF_SIZE 256

// Logging tag
static const char* TAG = "ESP_NOW_UART";

// Structure to receive ESP-NOW data
typedef struct {
    unsigned int value;
    uint8_t MacAddress[6];  // Sender's MAC
} Item;  // Ensure consistent packing

//typedef struct {
//    uint32_t can_id;
//    uint8_t dlc;
//    uint8_t data[8];
//} CANFrame;

Item incomingReadings;
QueueHandle_t NowUARTQueue;

extern "C" void app_main();

void init_uart();
static void uart_rx_task(void *pvParameter);
static void uart_tx_task(void *pvParameter);
static void sendToSTM_uart(Item *data);
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len);

esp_err_t init_esp_now(void);

void app_main()
{
    int coreID = xPortGetCoreID();
    printf("Running on core %d\n", coreID);
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    NowUARTQueue = xQueueCreate(10, sizeof(Item)); // Store up to 10 items
    if (NowUARTQueue == NULL) {
        ESP_LOGE(TAG,"Failed to create queue");
        return;
    }
    ESP_LOGI(TAG,"Starting STM32-ESP32 Communication System");
    
    // Configure LED pin
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);
    
    // Initialize UART communication
    init_uart();
    
    // Initialize ESP-NOW
    ESP_ERROR_CHECK(init_esp_now());
    // Create communication tasks with appropriate priorities
    xTaskCreate(uart_rx_task, "uart_rx_task", 4096, NULL, 12, NULL);  // Higher priority for RX
    xTaskCreate(uart_tx_task, "uart_tx_task", 4096, NULL, 11, NULL);  // Slightly lower priority for TX   
    ESP_LOGI(TAG,"All communication tasks started successfully");
    
    // Main task can now do other work or simply monitor the system
    while(1) {
        // Add any main loop processing here if needed
        vTaskDelay(pdMS_TO_TICKS(5000));
        ESP_LOGI(TAG,"System running normally...");
    }
}

void init_uart()
{
      int coreID = xPortGetCoreID();
    printf("init_uart Running on core %d\n", coreID);
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,  // Use default clock source
    };
    
    // Install UART driver with appropriate buffer sizes
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_0_TX, UART_0_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // Flush any existing data
    uart_flush(UART_NUM);
    
    ESP_LOGI(TAG,"UART initialized successfully");
}
//Task to send data from the queue to the stm32 via uart
static void uart_tx_task(void *pvParameters) {
      int coreID = xPortGetCoreID();
    printf("uart_tx_task Running on core %d\n", coreID);
    Item item;
    ESP_LOGI(TAG,"UART TX task started");
    
    while (1) {
        if (xQueueReceive(NowUARTQueue, &item, portMAX_DELAY) == pdTRUE) {
            sendToSTM_uart(&item);
        }
    }
}

static void sendToSTM_uart(Item *data)
{
      int coreID = xPortGetCoreID();
    printf("sendToSTM_uart Running on core %d\n", coreID);
    // Prepare data to send via uart - send exact struct size
    esp_err_t err = uart_wait_tx_done(UART_NUM, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG,"UART TX timeout");
        return;
    }
    
    // Send exact struct size, not strlen!
    int bytes_written = uart_write_bytes(UART_NUM, (const char*)data, sizeof(Item));
    if (bytes_written == sizeof(Item)) {
        ESP_LOGI(TAG,"Data sent to STM32 successfully (%d bytes)", bytes_written);
        ESP_LOGI(TAG,"Sent value: %u, MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
              data->value, 
              data->MacAddress[0], data->MacAddress[1], data->MacAddress[2], 
              data->MacAddress[3], data->MacAddress[4], data->MacAddress[5]);
        
        // Blink LED to indicate successful transmission
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_PIN, 0);
    } else {
        ESP_LOGE(TAG,"Failed to send complete data to STM32 (sent %d/%d bytes)", bytes_written, sizeof(Item));
    }
}

static void uart_rx_task(void *pvParameter)
{
      int coreID = xPortGetCoreID();
    printf("uart_rx_task Running on core %d\n", coreID);
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    Item receivedItem;
    
    if (data == NULL) {
        ESP_LOGE(TAG,"Failed to allocate memory for RX buffer");
        vTaskDelete(NULL);
        return;
    }
    
    ESP_LOGI(TAG,"UART RX task started");
     
    while (1) {
        // Try to receive exact struct size
        int len = uart_read_bytes(UART_NUM, data, sizeof(Item), 1000 / portTICK_PERIOD_MS);
        
        if (len == sizeof(Item)) {
            // Copy received data to Item struct
            memcpy(&receivedItem, data, sizeof(Item));
            
            ESP_LOGI(TAG,"Received from STM32 - Value: %u, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                     receivedItem.value,
                     receivedItem.MacAddress[0], receivedItem.MacAddress[1], receivedItem.MacAddress[2],
                     receivedItem.MacAddress[3], receivedItem.MacAddress[4], receivedItem.MacAddress[5]);
            
            // Blink LED to indicate successful reception
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_PIN, 0);
            
            // Here you could forward this data via ESP-NOW if needed
            // or process it as required
            
        } else if (len > 0) {
            ESP_LOGW(TAG,"Received incomplete data (%d bytes, expected %d)", len, sizeof(Item));
            // Clear the buffer of any partial data
            uart_flush_input(UART_NUM);
        }
        
        // Small delay to prevent overwhelming the task
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    free(data);
}

// ESP-NOW receive callback
//void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
//      int coreID = xPortGetCoreID();
//    printf("OnDataRecv Running on core %d\n", coreID);
//    if (len != sizeof(Item)) {
//        ESP_LOGE(TAG,"Received data size mismatch: %d bytes, expected %d", len, sizeof(Item));
//        return;
//    }
//    
//    Item item;
//    memcpy(&item, incomingData, sizeof(Item));
//    // Update MAC address with actual sender
//    memcpy(item.MacAddress, recv_info->src_addr, 6);
//
//    if (xQueueSend(NowUARTQueue, &item, 0) != pdTRUE) {
//        ESP_LOGE(TAG,"Queue full, dropping packet");
//    } else {
//        ESP_LOGI(TAG,"Queued item from MAC %02X:%02X:%02X:%02X:%02X:%02X, value %u",
//                 item.MacAddress[0], item.MacAddress[1], item.MacAddress[2],
//                 item.MacAddress[3], item.MacAddress[4], item.MacAddress[5],
//                 item.value);
//    }
//}
//
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    int coreID = xPortGetCoreID();
    printf("OnDataRecv Running on core %d\n", coreID);

    if (len != sizeof(Item)) {
        ESP_LOGE(TAG, "Received data size mismatch: %d bytes, expected %d", len, sizeof(Item));
        return;
    }

    Item item;
    memcpy(&item, incomingData, sizeof(Item));
    memcpy(item.MacAddress, recv_info->src_addr, 6);  // Set sender MAC

    // Log reception
    ESP_LOGI(TAG, "Received item from MAC %02X:%02X:%02X:%02X:%02X:%02X, value %u",
             item.MacAddress[0], item.MacAddress[1], item.MacAddress[2],
             item.MacAddress[3], item.MacAddress[4], item.MacAddress[5],
             item.value);

    // Send response back to sender
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, recv_info->src_addr, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Register peer only if it doesn't already exist
    if (!esp_now_is_peer_exist(recv_info->src_addr)) {
        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add peer for response");
        } else {
            ESP_LOGI(TAG, "Peer added for response");
        }
    }

    // Create response
    Item response;
    response.value = item.value + 100; // Just to differentiate the reply
    memcpy(response.MacAddress, recv_info->src_addr, 6);

    esp_err_t err = esp_now_send(recv_info->src_addr, (uint8_t *)&response, sizeof(response));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send reply: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Reply sent to MAC %02X:%02X:%02X:%02X:%02X:%02X",
                 recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
                 recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
    }

    // Optionally forward to UART
    if (xQueueSend(NowUARTQueue, &item, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Queue full, dropping packet");
    }
}

// Initialize ESP-NOW
esp_err_t init_esp_now(void) {
      int coreID = xPortGetCoreID();
    printf("init_esp_now Running on core %d\n", coreID);
    esp_err_t ret;
    
    // Initialize WiFi in station mode
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,"Failed to init WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,"Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,"Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize ESP-NOW
    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,"Failed to init ESP-NOW: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register receive callback
    ret = esp_now_register_recv_cb(OnDataRecv);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG,"Failed to register ESP-NOW recv callback: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG,"ESP-NOW initialized successfully");
    return ESP_OK;
}
