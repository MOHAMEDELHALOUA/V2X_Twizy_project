///////////////////////////////////////////// FIXED ESP32(1) code to send real CAN data
///
///////////////////////////////////////////// FIXED ESP32(1) code to send real CAN data
///
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
#include <stdarg.h>  // Added for va_list functions

// Pin definitions
#define UART_NUM UART_NUM_0  
#define LED_PIN GPIO_NUM_2

// Buffer size
#define BUF_SIZE 256

// V2G Communication Interval
#define V2G_SEND_INTERVAL_MS 5000  // Send V2G data every 5 seconds

// Debug control - DISABLED for clean USB serial communication with Jetson
#define DEBUG_PRINTS false

// Add these defines at the top
#define V2G_COMMUNICATION_TIMEOUT_MS 20000  // 20 seconds without EVCS response = disconnect
#define CHARGING_STOP_DELAY_MS 3000         // 3 seconds after charging stops, stop V2G

// Add these global variables
unsigned long last_evcs_received = 0;       // Last time we received data from EVCS
bool charging_recently_stopped = false;     // Flag to track recent charging stop
unsigned long charging_stop_time = 0;       // When charging stopped

// ===== UPDATED CAN DATA STRUCTURE (for Jetson communication with real battery data) =====
typedef struct {
    unsigned short SOC;        // From CAN 0x155
    float speedKmh;           // From CAN 0x19F
    float odometerKm;         // From CAN 0x5D7
    float displaySpeed;       // From GPS
    
    // NEW: Real battery data from CAN
    float battery_voltage;    // From CAN 0x425 - Real battery voltage
    float battery_current;    // From CAN 0x155 - Real battery current
    float available_energy;   // From CAN 0x425 - Real available energy
    uint8_t charging_status;  // From CAN 0x425 - Real charging status
    
    uint8_t MacAddress[6];     // Sender's MAC
} Item;

// ===== V2G DATA STRUCTURES (for EVCS communication) =====
// Data to receive from EVCS (EVCS -> Vehicle)
typedef struct {
    uint8_t evcs_mac[6];           // EVCS MAC address
    float ac_voltage;              // AC voltage available (V)
    float max_current;             // Maximum current available (A)
    float max_power;               // Maximum power available (W)
    float cost_per_kwh;            // Cost per kWh
    float current_energy_delivered; // Total energy delivered this session (kWh)
    float current_cost;            // Current session cost
    bool charging_available;       // Slot available for charging
    bool session_active;           // Added: Session active flag
    uint32_t session_id;           // Unique session identifier
    unsigned long timestamp;       // Message timestamp
} evcs_to_vehicle_t;

// Data to send to EVCS (Vehicle -> EVCS)
typedef struct {
    uint8_t vehicle_mac[6];        // Vehicle MAC address
    uint16_t battery_soc;          // State of Charge (0-100%)
    float battery_voltage;         // Battery voltage (V)
    float battery_current;         // Current battery current (A)
    float battery_capacity;        // Total battery capacity (kWh)
    float desired_soc;             // Desired SOC (0-100%)
    bool ready_to_charge;          // Vehicle ready to start charging
    bool stop_charging;            // Vehicle wants to stop charging
    uint32_t session_id;           // Session ID (should match EVCS)
    unsigned long timestamp;       // Message timestamp
} vehicle_to_evcs_t;

// Global variables
Item incomingReadings;
Item receivedItem; // Item received from jetson with real CAN data
Item lastValidCanData; // Store last valid CAN data to send
QueueHandle_t NowUSBQueue;

// V2G specific variables
evcs_to_vehicle_t evcs_data;
vehicle_to_evcs_t vehicle_data;
bool evcs_connected = false;
uint8_t evcs_mac[6] = {0};
unsigned long last_v2g_send = 0;

// Vehicle configuration
#define BATTERY_CAPACITY_KWH 6.1f    // Renault Twizy battery capacity
#define BATTERY_VOLTAGE_NOMINAL 58.0f // Nominal battery voltage
#define DESIRED_SOC_DEFAULT 80.0f     // Default desired SOC

// Flag to indicate if we have valid CAN data
bool hasValidCanData = false;

// Function declarations
extern "C" void app_main();
void init_usb_serial();
static void usb_serial_rx_task(void *pvParameter);
static void usb_serial_tx_task(void *pvParameter);
static void sendToJetson_usb(Item *data);
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len);
esp_err_t init_esp_now(void);
void updateCanDataFromReceived(const Item *received);
void disconnect_from_evcs();  // Added declaration

// V2G specific functions
void init_v2g_data();
void update_v2g_data_from_can();
void send_v2g_data_to_evcs();
void process_evcs_data(const evcs_to_vehicle_t *data);
void print_evcs_data(const evcs_to_vehicle_t *data);
void print_vehicle_data(const vehicle_to_evcs_t *data);

// Debug print function - DISABLED to prevent USB serial interference
void debug_print(const char* format, ...) {
    // All debug prints disabled for clean Jetson communication
}

// Add this function to check for disconnection
void check_v2g_timeouts() {
    unsigned long currentTime = pdTICKS_TO_MS(xTaskGetTickCount());
    
    // Check if EVCS communication has timed out
    if (evcs_connected && (currentTime - last_evcs_received > V2G_COMMUNICATION_TIMEOUT_MS)) {
        disconnect_from_evcs();
    }
    
    // Check if charging stopped and we should disconnect
    if (evcs_connected && charging_recently_stopped && 
        (currentTime - charging_stop_time > CHARGING_STOP_DELAY_MS)) {
        disconnect_from_evcs();
    }
}

// Add this function to disconnect from EVCS
void disconnect_from_evcs() {
    if (evcs_connected) {
        // Remove EVCS as peer
        esp_now_del_peer(evcs_mac);
        
        // Reset V2G connection
        evcs_connected = false;
        memset(evcs_mac, 0, 6);
        last_evcs_received = 0;
        charging_recently_stopped = false;
    }
}

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
    
    // Initialize CAN data with default values
    memset(&receivedItem, 0, sizeof(receivedItem));
    memset(&lastValidCanData, 0, sizeof(lastValidCanData));
    
    // Set default fallback values (start with zeros)
    lastValidCanData.SOC = 0;
    lastValidCanData.speedKmh = 0.0;
    lastValidCanData.displaySpeed = 0.0;
    lastValidCanData.odometerKm = 0.0;
    lastValidCanData.battery_voltage = BATTERY_VOLTAGE_NOMINAL;
    lastValidCanData.battery_current = 0.0;
    lastValidCanData.available_energy = 0.0;
    lastValidCanData.charging_status = 0x00;
    
    // Initialize V2G data
    init_v2g_data();
    
    // Create communication tasks
    xTaskCreate(usb_serial_rx_task, "usb_serial_rx_task", 4096, NULL, 12, NULL);
    xTaskCreate(usb_serial_tx_task, "usb_serial_tx_task", 4096, NULL, 11, NULL);
    
    while(1) {
        unsigned long currentTime = pdTICKS_TO_MS(xTaskGetTickCount());
        
        // Check for V2G timeouts
        check_v2g_timeouts();
        
        // Send V2G data to EVCS every 5 seconds if we have an EVCS connection
        if (evcs_connected && (currentTime - last_v2g_send >= V2G_SEND_INTERVAL_MS)) {
            send_v2g_data_to_evcs();
            last_v2g_send = currentTime;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void init_v2g_data() {
    // Get this ESP32's MAC address
    esp_wifi_get_mac(WIFI_IF_STA, vehicle_data.vehicle_mac);
    
    // Initialize vehicle data with default values
    vehicle_data.battery_soc = 50;  // Start with 50% SOC
    vehicle_data.battery_voltage = BATTERY_VOLTAGE_NOMINAL;
    vehicle_data.battery_current = 0.0f;
    vehicle_data.battery_capacity = BATTERY_CAPACITY_KWH;
    vehicle_data.desired_soc = DESIRED_SOC_DEFAULT;
    vehicle_data.ready_to_charge = true;
    vehicle_data.stop_charging = false;
    vehicle_data.session_id = 0;
    vehicle_data.timestamp = 0;
}

// Update the update_v2g_data_from_can() function
void update_v2g_data_from_can() {
    if (hasValidCanData) {
        // Use REAL CAN data directly (no more estimates!)
        vehicle_data.battery_soc = lastValidCanData.SOC;
        vehicle_data.battery_voltage = lastValidCanData.battery_voltage;
        vehicle_data.battery_current = lastValidCanData.battery_current;
        
        // NEW: Detect when charging stops
        bool currently_charging = (lastValidCanData.battery_current > 0.5f);
        static bool was_charging = false;
        
        if (was_charging && !currently_charging) {
            // Charging just stopped
            charging_recently_stopped = true;
            charging_stop_time = pdTICKS_TO_MS(xTaskGetTickCount());
        } else if (currently_charging) {
            // Reset charging stop flag if charging resumed
            charging_recently_stopped = false;
        }
        
        was_charging = currently_charging;
        
        // Auto-stop charging if SOC reaches desired level
        if (vehicle_data.battery_soc >= vehicle_data.desired_soc) {
            vehicle_data.stop_charging = true;
            vehicle_data.ready_to_charge = false;
        } else if (vehicle_data.battery_soc < (vehicle_data.desired_soc - 5.0f)) {
            vehicle_data.stop_charging = false;
            vehicle_data.ready_to_charge = true;
        }
        
        vehicle_data.timestamp = pdTICKS_TO_MS(xTaskGetTickCount());
    }
}

void send_v2g_data_to_evcs() {
    if (!evcs_connected) {
        return;
    }
    
    // Update vehicle data with latest REAL CAN information
    update_v2g_data_from_can();
    
    // Send data to EVCS
    esp_err_t result = esp_now_send(evcs_mac, (uint8_t *)&vehicle_data, sizeof(vehicle_data));
    
    // Silent operation - no debug prints to interfere with USB serial
}

void process_evcs_data(const evcs_to_vehicle_t *data) {
    // Store EVCS session ID for our responses
    vehicle_data.session_id = data->session_id;
    
    // Check if we should stop charging based on cost or other factors
    if (data->current_cost > 10.0f) {  // Stop if cost exceeds 10 units
        vehicle_data.stop_charging = true;
    }
    
    // Check if charging is available
    if (!data->charging_available) {
        vehicle_data.ready_to_charge = false;
    }
}

void print_evcs_data(const evcs_to_vehicle_t *data) {
    // Function disabled to prevent USB serial interference
}

void print_vehicle_data(const vehicle_to_evcs_t *data) {
    // Function disabled to prevent USB serial interference
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
        .flags = 0
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

// CLEAN: Minimal RX task - no debug prints that interfere with Jetson
static void usb_serial_rx_task(void *pvParameter)
{
    uint8_t header[2] = {0};
    uint8_t *data = (uint8_t *) malloc(sizeof(Item));
    
    if (data == NULL) {
        vTaskDelete(NULL);
        return;
    }
    
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
            
            // Update our CAN data store with REAL battery data (SILENT)
            updateCanDataFromReceived(&receivedItem);
            
            // Blink LED to indicate successful reception of CAN data
            gpio_set_level(LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(LED_PIN, 0);
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    free(data);
}

// CLEAN: Silent update function - no prints to interfere with Jetson communication
void updateCanDataFromReceived(const Item *received) {
    // Copy all data directly including new battery fields
    lastValidCanData = *received;
    hasValidCanData = true;
    
    // Silent operation - no debug prints to interfere with USB serial
}

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    // Check if this is EVCS V2G data
    if (len == sizeof(evcs_to_vehicle_t)) {
        // Update last received time
        last_evcs_received = pdTICKS_TO_MS(xTaskGetTickCount());
        
        // This is EVCS data
        evcs_to_vehicle_t received_evcs_data;
        memcpy(&received_evcs_data, incomingData, sizeof(evcs_to_vehicle_t));
        
        // Check if EVCS session is still active
        if (!received_evcs_data.session_active) {
            disconnect_from_evcs();
            return;
        }
        
        // Store EVCS MAC if this is a new EVCS
        if (!evcs_connected) {
            memcpy(evcs_mac, recv_info->src_addr, 6);
            evcs_connected = true;
            last_evcs_received = pdTICKS_TO_MS(xTaskGetTickCount());
            
            // Add EVCS as peer
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, evcs_mac, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            
            if (!esp_now_is_peer_exist(evcs_mac)) {
                esp_now_add_peer(&peerInfo);
            }
        }
        
        // Process EVCS data
        process_evcs_data(&received_evcs_data);
        
        return;
    } 
    // Check if this is enhanced CAN data with battery info
    if (len == sizeof(Item)) {
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
            // Send real CAN data including battery info from Jetson
            response = lastValidCanData;
        } else {
            // Send zeros if no CAN data available yet
            memset(&response, 0, sizeof(response));
        }
        
        // Set this ESP32's MAC in response
        uint8_t mac[6];
        esp_wifi_get_mac(WIFI_IF_STA, mac);
        memcpy(response.MacAddress, mac, 6);
        
        esp_err_t err = esp_now_send(recv_info->src_addr, (uint8_t *)&response, sizeof(response));
        
        // Forward received data from external ESP32 to Jetson via USB Serial
        if (xQueueSend(NowUSBQueue, &item, 0) != pdTRUE) {
            // Queue full, drop packet silently
        }
        
        return;
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
