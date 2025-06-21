#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "driver/gpio.h"

// SPI Pins
#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS 5

// Logging tag
static const char* TAG = "ESP_NOW_SPI";

// Global SPI handle
spi_device_handle_t spi_handle;

// Structure to receive ESP-NOW data
typedef struct {
    unsigned int value;
    uint8_t MacAddress[6];  // Sender's MAC
} Item;

Item incomingReadings;

// ESP-NOW receive callback
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    // Copy received data
    memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
    
    ESP_LOGI(TAG, "Received ESP-NOW data from MAC: %02X:%02X:%02X:%02X:%02X:%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "Value received: %u", incomingReadings.value);
    
    // Send received data to STM32 via SPI
    send_to_stm32_via_spi(&incomingReadings);
}

// Function to send data to STM32 via SPI
void send_to_stm32_via_spi(Item* data) {
    esp_err_t ret;
    
    // Prepare data to send (you can modify this format as needed)
    uint8_t tx_buffer[sizeof(Item)];
    uint8_t rx_buffer[sizeof(Item)] = {0};
    
    // Copy the received ESP-NOW data to SPI buffer
    memcpy(tx_buffer, data, sizeof(Item));
    
    // SPI transaction
    spi_transaction_t trans = {
        .flags = 0,
        .cmd = 0,
        .addr = 0,
        .length = sizeof(Item) * 8,  // Length in bits
        .rxlength = 0,               // Don't expect response from STM32
        .user = NULL,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer
    };
    
    ESP_LOGI(TAG, "Sending data to STM32 via SPI...");
    ret = spi_device_transmit(spi_handle, &trans);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Data sent to STM32 successfully");
        ESP_LOGI(TAG, "Sent value: %u", data->value);
    } else {
        ESP_LOGE(TAG, "Failed to send data to STM32: %s", esp_err_to_name(ret));
    }
}

// Initialize SPI
esp_err_t init_spi(void) {
    esp_err_t ret;
    
    // SPI bus configuration
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
        .flags = 0,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        .intr_flags = 0
    };
    
    // SPI device configuration
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,                      // SPI mode 0
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .duty_cycle_pos = 128,          // 50% duty cycle
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 3,          // Keep CS low 3 cycles after transaction
        .clock_speed_hz = 1000000,      // 1 MHz
        .input_delay_ns = 0,
        .spics_io_num = GPIO_CS,        // CS pin
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL
    };
    
    // Initialize SPI bus
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Add SPI device
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "SPI initialized successfully");
    return ESP_OK;
}

// Initialize ESP-NOW
esp_err_t init_esp_now(void) {
    esp_err_t ret;
    
    // Initialize WiFi in station mode
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set WiFi mode: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize ESP-NOW
    ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init ESP-NOW: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Register receive callback
    ret = esp_now_register_recv_cb(OnDataRecv);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register ESP-NOW recv callback: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "ESP-NOW initialized successfully");
    return ESP_OK;
}

// Main application
void app_main(void) {
    esp_err_t ret;
    
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    ESP_LOGI(TAG, "Starting ESP32 ESP-NOW to SPI Bridge...");
    
    // Initialize SPI
    ret = init_spi();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI initialization failed");
        return;
    }
    
    // Initialize ESP-NOW
    ret = init_esp_now();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ESP-NOW initialization failed");
        return;
    }
    
    ESP_LOGI(TAG, "ESP32 Bridge ready - waiting for ESP-NOW data...");
    
    // Main loop - just keep the system running
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
