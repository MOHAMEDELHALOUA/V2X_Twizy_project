
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "soc/rtc_periph.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "spi_flash_mmap.h"

#include "driver/gpio.h"
#include "esp_intr_alloc.h"

// Pins in use
#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS 5

#define TAG "SPI"

// Main application
extern "C" void app_main(void)
{
    esp_err_t ret;

    // Configuration for the SPI bus
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = GPIO_MOSI;
    buscfg.miso_io_num = GPIO_MISO;
    buscfg.sclk_io_num = GPIO_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = 4096;
    // Configuration for the SPI device on the other side of the bus
    spi_device_interface_config_t devcfg = {}; 
    devcfg.command_bits = 0;
    devcfg.address_bits = 0;
    devcfg.dummy_bits = 0;
    devcfg.clock_speed_hz = 1000000;  // 1 MHz
    devcfg.duty_cycle_pos = 128;      // 50% duty cycle
    devcfg.mode = 0;                  // SPI mode 0
    devcfg.spics_io_num = GPIO_CS;    // CS pin
    devcfg.cs_ena_posttrans = 3;      // Keep the CS low 3 cycles after transaction
    devcfg.queue_size = 1;
    // Initialize the SPI bus
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    printf("spi_bus_initialize %d\n", ret);     // ESP_OK = 0

    spi_device_handle_t spi_handle;
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spi_handle);
    printf("spi_bus_add_device %d\n", ret);     // ESP_OK = 0

    uint8_t tx_data = 'C';
    uint8_t rx_data = 'D';
    printf("tx_data: %c\n", tx_data);
    printf("rx_data: %c\n", rx_data);
    
    spi_transaction_t t = {};
    t.length = 8 * sizeof(tx_data);          // bits to transmit
    t.rxlength = 8 * sizeof(tx_data);        // bits to receive
    t.tx_buffer = &tx_data;
    t.rx_buffer = &rx_data;

    printf("Master received:\n");
    while (1)
    {
        ret = spi_device_transmit(spi_handle, &t);
        printf("Transmited: %c\n", *((char *)t.tx_buffer));
        printf("Received: %c\n", *((char *)t.rx_buffer));
        vTaskDelay(pdMS_TO_TICKS(1500));        // Delay 5 second
    }
}
