///////////////////////////////////////////// FIXED ESP32(1) code to send real CAN data
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "string.h"
#include "freertos/queue.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "nvs_flash.h"

// Pin definitions
#define UART_NUM UART_NUM_0  
#define LED_PIN GPIO_NUM_2

// Buffer size
#define BUF_SIZE 256

// Structure to receive ESP-NOW data - MUST MATCH sender exactly
typedef struct {
    unsigned short SOC;        
    float speedKmh;
    float odometerKm;
    float displaySpeed;
    uint8_t MacAddress[6];     // Sender's MAC
} Item;

Item incomingReadings;
Item receivedItem; // Item received from jetson with real CAN data
Item lastValidCanData; // Store last valid CAN data to send
QueueHandle_t NowUSBQueue;

// Flag to indicate if we have valid CAN data
bool hasValidCanData = false;

extern "C" void app_main();
void init_usb_serial();
static void usb_serial_rx_task(void *pvParameter);
static void usb_serial_tx_task(void *pvParameter);
static void sendToJetson_usb(Item *data);
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len);
esp_err_t init_esp_now(void);
void updateCanDataFromReceived(const Item *received);

void app_main()
{
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
    
    NowUSBQueue = xQueueCreate(10, sizeof(Item));
    if (NowUSBQueue == NULL) {
        return;
    }
    
    // Configure LED pin
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);
    
    // Initialize USB Serial communication
    init_usb_serial();
    
    // Initialize ESP-NOW
    ESP_ERROR_CHECK(init_esp_now());
    
    // Initialize with default values
    memset(&receivedItem, 0, sizeof(receivedItem));
    memset(&lastValidCanData, 0, sizeof(lastValidCanData));
    
    // Set default fallback values (start with zeros)
    lastValidCanData.SOC = 0;
    lastValidCanData.speedKmh = 0.0;
    lastValidCanData.displaySpeed = 0.0;
    lastValidCanData.odometerKm = 0.0;
    
    // Create communication tasks
    xTaskCreate(usb_serial_rx_task, "usb_serial_rx_task", 4096, NULL, 12, NULL);
    xTaskCreate(usb_serial_tx_task, "usb_serial_tx_task", 4096, NULL, 11, NULL);
    
    printf("ESP32(1) started - will broadcast real CAN data from Jetson\n");
    
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void init_usb_serial()
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    uart_flush(UART_NUM);
}

// Task to send data from the queue to the Jetson via USB Serial
static void usb_serial_tx_task(void *pvParameters) {
    Item item;
    
    while (1) {
        if (xQueueReceive(NowUSBQueue, &item, portMAX_DELAY) == pdTRUE) {
            sendToJetson_usb(&item);
        }
    }
}

static void sendToJetson_usb(Item *data)
{
    esp_err_t err = uart_wait_tx_done(UART_NUM, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        return;
    }
    
    const uint8_t HEADER[] = {0xAA, 0x55};
    uart_write_bytes(UART_NUM, (const char*)HEADER, sizeof(HEADER));
    uart_write_bytes(UART_NUM, (const char*)data, sizeof(Item));
}

static void usb_serial_rx_task(void *pvParameter)
{
    uint8_t header[2] = {0};
    uint8_t *data = (uint8_t *) malloc(sizeof(Item));
    
    if (data == NULL) {
        vTaskDelete(NULL);
        return;
    }
    
    printf("USB Serial RX task started - waiting for CAN data from Jetson\n");
    
    while (1) {
        // Look for header bytes first
        int headerPos = 0;
        
        while (headerPos < 2) {
            int len = uart_read_bytes(UART_NUM, &header[headerPos], 1, 100 / portTICK_PERIOD_MS);
            if (len == 1) {
                if (headerPos == 0 && header[0] == 0xAA) {
                    headerPos = 1;
                } else if (headerPos == 1 && header[1] == 0x55) {
                    headerPos = 2; // Header complete
                } else {
                    headerPos = 0; // Reset if wrong sequence
                }
            }
        }
        
        // Now read the data payload
        int totalReceived = 0;
        while (totalReceived < sizeof(Item)) {
            int len = uart_read_bytes(UART_NUM, data + totalReceived, 
                                    sizeof(Item) - totalReceived, 
                                    1000 / portTICK_PERIOD_MS);
            if (len > 0) {
                totalReceived += len;
            } else {
                break; // Timeout
            }
        }
        
        if (totalReceived == sizeof(Item)) {
            // Copy received data to Item struct
            memcpy(&receivedItem, data, sizeof(Item));
            
            // Update our CAN data store (NO VALIDATION)
            updateCanDataFromReceived(&receivedItem);
            
            // Blink LED to indicate successful reception of CAN data
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(LED_PIN, 0);
            
            printf("Received CAN data from Jetson: SOC=%u%%, Speed=%.1fkm/h, Display=%.1fkm/h, Odo=%.1fkm\n",
                   receivedItem.SOC, receivedItem.speedKmh, receivedItem.displaySpeed, receivedItem.odometerKm);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    free(data);
}

// NO VALIDATION - just use the data as-is
void updateCanDataFromReceived(const Item *received) {
    // Copy all data directly without any validation
    lastValidCanData = *received;
    hasValidCanData = true;
    
    printf("Updated CAN data from Jetson: SOC=%u%%, Speed=%.1fkm/h, Display=%.1fkm/h, Odo=%.1fkm\n",
           lastValidCanData.SOC, lastValidCanData.speedKmh, 
           lastValidCanData.displaySpeed, lastValidCanData.odometerKm);
}

// ESP-NOW callback - send REAL CAN data from Jetson
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    if (len != sizeof(Item)) {
        return;
    }
    
    Item item;
    memcpy(&item, incomingData, sizeof(Item));
    memcpy(item.MacAddress, recv_info->src_addr, 6);  // Set sender MAC
    
    // Send response back to sender
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, recv_info->src_addr, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (!esp_now_is_peer_exist(recv_info->src_addr)) {
        if (esp_now_add_peer(&peerInfo) == ESP_OK) {
            // Peer added successfully
        }
    }
    
    // Create response with REAL CAN data from Jetson
    Item response;
    
    if (hasValidCanData) {
        // Send real CAN data from Jetson
        response = lastValidCanData;
        printf("Broadcasting REAL CAN data from Jetson: SOC=%u%%, Speed=%.1fkm/h, Display=%.1fkm/h, Odo=%.1fkm\n",
               response.SOC, response.speedKmh, response.displaySpeed, response.odometerKm);
    } else {
        // Send zeros if no CAN data available yet
        response.SOC = 0;
        response.speedKmh = 0.0;
        response.displaySpeed = 0.0;
        response.odometerKm = 0.0;
        printf("No CAN data from Jetson yet, sending zeros\n");
    }
    
    // Set this ESP32's MAC in response
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    memcpy(response.MacAddress, mac, 6);
    
    esp_err_t err = esp_now_send(recv_info->src_addr, (uint8_t *)&response, sizeof(response));
    if (err == ESP_OK) {
        printf("CAN data sent successfully via ESP-NOW to external ESP32\n");
    } else {
        printf("Failed to send CAN data via ESP-NOW: %s\n", esp_err_to_name(err));
    }
    
    // Forward received data from external ESP32 to Jetson via USB Serial
    if (xQueueSend(NowUSBQueue, &item, 0) != pdTRUE) {
        // Queue full, drop packet silently
    }
}

// Initialize ESP-NOW
esp_err_t init_esp_now(void) {
    esp_err_t ret;
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = esp_now_init();
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = esp_now_register_recv_cb(OnDataRecv);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return ESP_OK;
}
