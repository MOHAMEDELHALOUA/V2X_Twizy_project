#include <Arduino.h>
#include "driver/spi_slave.h"

// SPI pins - must match your master
#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS 5

// Buffers must be DMA-capable (word aligned)
WORD_ALIGNED_ATTR uint8_t sendbuf[4] = {0};
WORD_ALIGNED_ATTR uint8_t recvbuf[4] = {0};

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting Built-in ESP32 SPI Slave...");

    // SPI slave configuration
    spi_slave_interface_config_t slvcfg = {};
    slvcfg.mode = 0;                    // SPI mode 0
    slvcfg.spics_io_num = GPIO_CS;      // CS pin
    slvcfg.queue_size = 1;              // Transaction queue size
    slvcfg.flags = 0;
    slvcfg.post_setup_cb = NULL;
    slvcfg.post_trans_cb = NULL;

    // SPI bus configuration
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = GPIO_MOSI;
    buscfg.miso_io_num = GPIO_MISO;
    buscfg.sclk_io_num = GPIO_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4;

    // Initialize SPI slave
    esp_err_t ret = spi_slave_initialize(VSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        Serial.println("SPI slave initialization FAILED!");
        return;
    }
    
    Serial.println("SPI Slave initialized successfully!");
    
    // Prepare response data
    sendbuf[0] = 0xAA; // ACK byte
}

void loop() {
    // Clear receive buffer
    memset(recvbuf, 0, sizeof(recvbuf));
    
    // Set up transaction
    spi_slave_transaction_t t = {};
    t.length = 8;  // 1 byte = 8 bits
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;

    // Wait for master to send data
    esp_err_t ret = spi_slave_transmit(VSPI_HOST, &t, portMAX_DELAY);
    
    if (ret == ESP_OK) {
        if (t.trans_len > 0) {
            Serial.print("✅ Received: ");
            Serial.print(recvbuf[0]);
            Serial.print(" (");
            Serial.print(t.trans_len);
            Serial.println(" bits)");
            
            // Update response for next transaction
            sendbuf[0] = recvbuf[0] + 100; // Send back received value + 100
        } else {
            Serial.println("❌ Transaction completed but no data");
        }
    } else {
        Serial.print("❌ SPI error: ");
        Serial.println(ret);
        delay(100);
    }
}
