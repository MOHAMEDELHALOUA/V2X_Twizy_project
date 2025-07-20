/////////////////////////////////////////////code for esp32 communication esp32(2)---esp-now--->esp32(1)---usb-serial--->jetson nano
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

// Pin definitions - No longer need custom UART pins since using USB
#define UART_NUM UART_NUM_0  // Changed from UART_NUM_1 to UART_NUM_0 (USB Serial)
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
QueueHandle_t NowUSBQueue;

extern "C" void app_main();
void init_usb_serial();
static void usb_serial_rx_task(void *pvParameter);
static void usb_serial_tx_task(void *pvParameter);
static void sendToJetson_usb(Item *data);
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len);
esp_err_t init_esp_now(void);

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
    
    NowUSBQueue = xQueueCreate(10, sizeof(Item)); // Store up to 10 items
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
    
    // Create communication tasks with appropriate priorities
    xTaskCreate(usb_serial_rx_task, "usb_serial_rx_task", 4096, NULL, 12, NULL);  // Higher priority for RX
    xTaskCreate(usb_serial_tx_task, "usb_serial_tx_task", 4096, NULL, 11, NULL);  // Slightly lower priority for TX   
    
    // Main task can now do other work or simply monitor the system
    while(1) {
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
    
    // Install UART driver with appropriate buffer sizes
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    // No need to set pins for UART0 as they are fixed (GPIO1=TX, GPIO3=RX)
    
    // Flush any existing data
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
    // Send as binary data
    esp_err_t err = uart_wait_tx_done(UART_NUM, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK) {
        return;
    }
    const uint8_t HEADER[] = {0xAA, 0x55};  // 2-byte start marker 
    uart_write_bytes(UART_NUM, (const char*)HEADER, sizeof(HEADER));
    // Send exact struct size
    int bytes_written = uart_write_bytes(UART_NUM, (const char*)data, sizeof(Item));
//    if (bytes_written == sizeof(Item)) {
//        // Blink LED to indicate successful transmission
//        gpio_set_level(LED_PIN, 1);
//        vTaskDelay(pdMS_TO_TICKS(50));
//        gpio_set_level(LED_PIN, 0);
//    }
}

static void usb_serial_rx_task(void *pvParameter)
{
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    Item receivedItem;
    
    if (data == NULL) {
        vTaskDelete(NULL);
        return;
    }
     
    while (1) {
        // Receive binary data
        int len = uart_read_bytes(UART_NUM, data, sizeof(Item), 1000 / portTICK_PERIOD_MS);
        
        if (len == sizeof(Item)) {
            // Copy received data to Item struct
            memcpy(&receivedItem, data, sizeof(Item));
            
            // Blink LED to indicate successful reception
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED_PIN, 0);
            
            // Forward this data via ESP-NOW if needed
            // You can add ESP-NOW transmission code here
            
        } else if (len > 0) {
            // Clear the buffer of any partial data
            uart_flush_input(UART_NUM);
        }
        
        // Small delay to prevent overwhelming the task
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    free(data);
}

// ESP-NOW receive callback
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
    
    // Create response
    Item response;
    response.SOC = 80;
    response.speedKmh = 60;
    response.displaySpeed = 58;
    response.odometerKm = 3500;
    memcpy(response.MacAddress, recv_info->src_addr, 6);
    
    esp_err_t err = esp_now_send(recv_info->src_addr, (uint8_t *)&response, sizeof(response));
    
    // Forward to Jetson via USB Serial
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
