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
#include <math.h>

// Pin definitions
#define UART_NUM UART_NUM_0  
#define LED_PIN GPIO_NUM_2

// Buffer size
#define BUF_SIZE 256

// V2G/V2V Communication Interval
#define V2G_SEND_INTERVAL_MS 5000  // Send V2G data every 5 seconds
#define V2V_BROADCAST_INTERVAL_MS 2000  // Broadcast every 2 seconds
#define V2G_FALLBACK_INTERVAL_MS 10000  // Fallback broadcast every 10 seconds

unsigned long last_v2v_broadcast = 0;
unsigned long last_v2g_fallback = 0;

// Debug control - set to false for production (clean serial communication)
#define DEBUG_PRINTS false 

// Global variables to store EVCS position
float received_evcs_latitude = 0.0f;
float received_evcs_longitude = 0.0f;
bool evcs_position_known = false;

// ===== UPDATED ITEM STRUCTURE IN ESP32 CODE =====
typedef struct {
    // Battery data (V2G critical)
    unsigned short SOC;              // From CAN 0x155 - State of Charge %
    float battery_voltage;           // From CAN 0x425 - Real battery voltage (V)
    float battery_current;           // From CAN 0x155 - Real battery current (A)
    float available_energy;          // From CAN 0x425 - Real available energy (kWh)
    uint8_t charging_status;         // From CAN 0x425 - Real charging status
    
    // Vehicle dynamics (V2V critical)
    float speedKmh;                  // From CAN 0x19F - Motor speed (km/h)
    float acceleration;              // From CAN 0x59B - Accelerator percent (0-100%)
    char gear[4];                    // From CAN 0x59B - Gear position (P/R/N/D)
    uint8_t brake_status;            // From CAN 0x59B - Brake pressed (0/1)
    
    // Vehicle position (V2V/V2G critical)
    float gps_latitude;              // From GPS - Latitude (degrees)
    float gps_longitude;             // From GPS - Longitude (degrees) 
    float gps_altitude;              // From GPS - Altitude (meters)
    uint8_t gps_valid;               // GPS validity flag
    
    // V2G specific data
    bool is_charging_detected;       // Charging detection flag
    float desired_soc;               // Desired SOC for charging (%)
    bool ready_to_charge;            // Vehicle ready for charging
    
    // Communication
    uint8_t MacAddress[6];           // Sender's MAC address
    uint32_t timestamp;              // Message timestamp
} Item;

// ===== V2G DATA STRUCTURES (for EVCS communication) =====
// Data to receive from EVCS (EVCS -> Vehicle)
typedef struct {
    uint8_t evcs_mac[6];           // EVCS MAC address
    
    // EVCS Position
    float evcs_latitude;           // EVCS GPS latitude
    float evcs_longitude;          // EVCS GPS longitude
    float evcs_altitude;           // EVCS GPS altitude (optional)
    
    // Charging specifications
    float ac_voltage;              // AC voltage available (V)
    float max_current;             // Maximum current available (A)
    float max_power;               // Maximum power available (W)
    float cost_per_kwh;            // Cost per kWh
    
    // Session information
    float current_energy_delivered; // Total energy delivered this session (kWh)
    float current_cost;            // Current session cost
    bool charging_available;       // Slot available for charging
    uint32_t session_id;           // Unique session identifier
    unsigned long timestamp;       // Message timestamp
} evcs_to_vehicle_t;

// Data to send to EVCS (Vehicle -> EVCS)
typedef struct {
    uint8_t vehicle_mac[6];        // Vehicle MAC address
    
    // Vehicle Position
    float vehicle_latitude;        // Vehicle GPS latitude
    float vehicle_longitude;       // Vehicle GPS longitude
    float vehicle_altitude;        // Vehicle GPS altitude
    uint8_t gps_valid;            // GPS validity flag (0=invalid, 1=valid)
    
    // Battery information
    uint16_t battery_soc;          // State of Charge (0-100%)
    float battery_voltage;         // Battery voltage (V)
    float battery_current;         // Current battery current (A)
    float battery_capacity;        // Total battery capacity (kWh)
    float desired_soc;             // Desired SOC (0-100%)
    
    // Charging control
    bool ready_to_charge;          // Vehicle ready to start charging
    bool stop_charging;            // Vehicle wants to stop charging
    
    // Communication
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
#define DESIRED_SOC_DEFAULT 100.0f     // Default desired SOC

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

// V2G specific functions
void init_v2g_data();
void update_v2g_data_from_can();
void send_v2g_data_to_evcs();
void process_evcs_data(const evcs_to_vehicle_t *data);
void print_evcs_data(const evcs_to_vehicle_t *data);
void print_vehicle_data(const vehicle_to_evcs_t *data);

// Debug print function - only prints if DEBUG_PRINTS is enabled
void debug_print(const char* format, ...) {
    if (DEBUG_PRINTS) {
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
    }
}

// DISTANCE CALCULATION FUNCTION
float calculate_distance_between_points(float lat1, float lon1, float lat2, float lon2);
//calculate_distance_to_evcs FUNCTION
float calculate_distance_to_evcs();
void broadcast_v2v_data();

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
    lastValidCanData.battery_voltage = BATTERY_VOLTAGE_NOMINAL;
    lastValidCanData.battery_current = 0.0;
    lastValidCanData.available_energy = 0.0;
    lastValidCanData.charging_status = 0x00;
    
    // Initialize V2G data
    init_v2g_data();
    
    // Create communication tasks
    xTaskCreate(usb_serial_rx_task, "usb_serial_rx_task", 4096, NULL, 12, NULL);
    xTaskCreate(usb_serial_tx_task, "usb_serial_tx_task", 4096, NULL, 11, NULL);
    
    // Only essential startup message
    debug_print("Vehicle ESP32 V2G started - Enhanced with Real CAN Battery Data\n");
    debug_print("Vehicle MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                vehicle_data.vehicle_mac[0], vehicle_data.vehicle_mac[1],
                vehicle_data.vehicle_mac[2], vehicle_data.vehicle_mac[3],
                vehicle_data.vehicle_mac[4], vehicle_data.vehicle_mac[5]);
    
    while(1) {
        unsigned long currentTime = pdTICKS_TO_MS(xTaskGetTickCount());
        
        // Send V2G data to EVCS every 5 seconds if we have an EVCS connection
        if (evcs_connected && (currentTime - last_v2g_send >= V2G_SEND_INTERVAL_MS)) {
            send_v2g_data_to_evcs();
            last_v2g_send = currentTime;
        }
        
        // V2V broadcast (every 2 seconds)
        if (hasValidCanData && (currentTime - last_v2v_broadcast >= V2V_BROADCAST_INTERVAL_MS)) {
            broadcast_v2v_data();
            last_v2v_broadcast = currentTime;
        }
        
        // FIXED: V2G fallback communication during charging
        if (hasValidCanData && lastValidCanData.is_charging_detected && 
            (currentTime - last_v2g_fallback >= V2G_FALLBACK_INTERVAL_MS)) {
            
            debug_print("[V2G] Fallback: Broadcasting vehicle data during charging\n");
            
            // Update vehicle data with latest CAN information
            update_v2g_data_from_can();
            
            // Broadcast to find EVCS
            uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
            esp_err_t result = esp_now_send(broadcast_mac, (uint8_t *)&vehicle_data, sizeof(vehicle_data));
            
            if (result == ESP_OK) {
                debug_print("[V2G] Fallback broadcast sent successfully\n");
            }
            
            last_v2g_fallback = currentTime;
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
    
    debug_print("V2G Vehicle data initialized:\n");
    debug_print("  Battery Capacity: %.1f kWh\n", vehicle_data.battery_capacity);
    debug_print("  Battery Voltage: %.1f V\n", vehicle_data.battery_voltage);
    debug_print("  Desired SOC: %.1f%%\n", vehicle_data.desired_soc);
}

// Updated V2G data function with vehicle position
void update_v2g_data_from_can() {
    if (hasValidCanData) {
        // Battery data
        vehicle_data.battery_soc = lastValidCanData.SOC;
        vehicle_data.battery_voltage = lastValidCanData.battery_voltage;
        vehicle_data.battery_current = lastValidCanData.battery_current;
        
        // Vehicle position data
        vehicle_data.vehicle_latitude = lastValidCanData.gps_latitude;
        vehicle_data.vehicle_longitude = lastValidCanData.gps_longitude;
        vehicle_data.vehicle_altitude = lastValidCanData.gps_altitude;
        vehicle_data.gps_valid = lastValidCanData.gps_valid;
        
        // FIXED: Charging readiness logic
        if (lastValidCanData.is_charging_detected || lastValidCanData.battery_current > 0.5f) {
            vehicle_data.ready_to_charge = true;
            vehicle_data.stop_charging = false;
        } else {
            // Normal logic when not charging
            vehicle_data.ready_to_charge = (vehicle_data.battery_soc < 100);
            vehicle_data.stop_charging = false;
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
    
    if (result == ESP_OK) {
        debug_print("[V2G] Vehicle data sent to EVCS successfully\n");
        print_vehicle_data(&vehicle_data);
    } else {
        debug_print("[V2G] Failed to send vehicle data to EVCS: %s\n", esp_err_to_name(result));
    }
}

void process_evcs_data(const evcs_to_vehicle_t *data) {
    // Store EVCS position from received data
    received_evcs_latitude = data->evcs_latitude;
    received_evcs_longitude = data->evcs_longitude;
    evcs_position_known = true;
    
    // Store EVCS session ID for our responses
    vehicle_data.session_id = data->session_id;
    
    debug_print("[V2G] EVCS Position received: %.6f, %.6f\n", 
               received_evcs_latitude, received_evcs_longitude);
    
    // Calculate distance to EVCS
    if (hasValidCanData && lastValidCanData.gps_valid) {
        float distance = calculate_distance_to_evcs();
        debug_print("[V2G] Distance to EVCS: %.1f meters\n", distance);
    }
    
    print_evcs_data(data);
    
    // Charging logic
    if (data->current_cost > 10.0f) {
        vehicle_data.stop_charging = true;
        debug_print("[V2G] Stopping due to high cost: %.2f\n", data->current_cost);
    }
    
    if (!data->charging_available) {
        debug_print("[V2G] Charging slot not available\n");
        vehicle_data.ready_to_charge = false;
    }
}

void print_evcs_data(const evcs_to_vehicle_t *data) {
    debug_print("\n🔌 EVCS DATA RECEIVED 🔌\n");
    debug_print("AC Voltage: %.1f V\n", data->ac_voltage);
    debug_print("Max Current: %.1f A\n", data->max_current);
    debug_print("Max Power: %.1f W\n", data->max_power);
    debug_print("Cost per kWh: %.2f\n", data->cost_per_kwh);
    debug_print("Energy Delivered: %.3f kWh\n", data->current_energy_delivered);
    debug_print("Current Cost: %.2f\n", data->current_cost);
    debug_print("Charging Available: %s\n", data->charging_available ? "YES" : "NO");
    debug_print("Session ID: %lu\n", data->session_id);
    debug_print("==========================\n\n");
}

void print_vehicle_data(const vehicle_to_evcs_t *data) {
    debug_print("[V2G] Vehicle Data: SOC=%u%%, Voltage=%.1fV, Current=%.1fA, Ready=%s, Stop=%s\n",
                data->battery_soc, data->battery_voltage, data->battery_current,
                data->ready_to_charge ? "YES" : "NO",
                data->stop_charging ? "YES" : "NO");
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

// FIXED: Enhanced RX task with better error handling
static void usb_serial_rx_task(void *pvParameter)
{
    uint8_t header[2] = {0};
    uint8_t *data = (uint8_t *) malloc(sizeof(Item));
    
    if (data == NULL) {
        vTaskDelete(NULL);
        return;
    }
    
    // Only essential startup message
    debug_print("USB Serial RX task started - waiting for REAL CAN battery data from Jetson\n");
    
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

void updateCanDataFromReceived(const Item *received) {
    // Copy all V2V/V2G data
    lastValidCanData = *received;
    
    // Ensure GPS validity is properly handled
    lastValidCanData.gps_valid = received->gps_valid;
    
    hasValidCanData = true;
    
    debug_print("Updated V2V/V2G data: SOC=%u%%, V=%.1fV, I=%.1fA, Speed=%.1fkm/h\n",
                lastValidCanData.SOC, lastValidCanData.battery_voltage, 
                lastValidCanData.battery_current, lastValidCanData.speedKmh);
    debug_print("  Position: %.6f,%.6f, GPS Valid: %s\n",
                lastValidCanData.gps_latitude, lastValidCanData.gps_longitude,
                lastValidCanData.gps_valid ? "YES" : "NO");
}

// FIXED: Enhanced ESP-NOW data reception with proper peer management
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    // Check if this is EVCS V2G data
    if (len == sizeof(evcs_to_vehicle_t)) {
        debug_print("[V2G] Received data from EVCS\n");
        
        evcs_to_vehicle_t received_evcs_data;
        memcpy(&received_evcs_data, incomingData, sizeof(evcs_to_vehicle_t));
        
        // FIXED: Handle EVCS connection and add peer properly
        if (!evcs_connected) {
            memcpy(evcs_mac, recv_info->src_addr, 6);
            evcs_connected = true;
            
            // CRITICAL FIX: Add EVCS as peer for bidirectional communication
            esp_now_peer_info_t evcs_peer = {};
            memcpy(evcs_peer.peer_addr, evcs_mac, 6);
            evcs_peer.channel = 0;
            evcs_peer.encrypt = false;
            
            if (!esp_now_is_peer_exist(evcs_mac)) {
                esp_err_t result = esp_now_add_peer(&evcs_peer);
                debug_print("[V2G] Added EVCS peer: %s\n", 
                           result == ESP_OK ? "SUCCESS" : "FAILED");
            }
            
            debug_print("[V2G] Connected to EVCS: %02X:%02X:%02X:%02X:%02X:%02X\n",
                       evcs_mac[0], evcs_mac[1], evcs_mac[2], 
                       evcs_mac[3], evcs_mac[4], evcs_mac[5]);
        }
        
        process_evcs_data(&received_evcs_data);
        
        // CRITICAL FIX: Send immediate response to EVCS
        if (hasValidCanData) {
            debug_print("[V2G] Sending immediate response to EVCS\n");
            send_v2g_data_to_evcs();
        }
        
        return;
    }
    
    // Check if this is V2V data from another vehicle
    if (len == sizeof(Item)) {
        debug_print("[V2V] Received vehicle data from: %02X:%02X:%02X:%02X:%02X:%02X\n",
                   recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
                   recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
        
        Item received_vehicle_item;
        memcpy(&received_vehicle_item, incomingData, sizeof(Item));
        memcpy(received_vehicle_item.MacAddress, recv_info->src_addr, 6);
        
        // Process V2V data (collision avoidance, traffic info, etc.)
        debug_print("[V2V] Other vehicle: SOC=%u%%, Speed=%.1fkm/h, Pos:%.6f,%.6f\n",
                   received_vehicle_item.SOC, received_vehicle_item.speedKmh,
                   received_vehicle_item.gps_latitude, received_vehicle_item.gps_longitude);
        debug_print("[V2V] Brake:%s, Gear:%s, Charging:%s\n",
                   received_vehicle_item.brake_status ? "ON" : "OFF",
                   received_vehicle_item.gear,
                   received_vehicle_item.is_charging_detected ? "YES" : "NO");
        
        // Add peer for future communication
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, recv_info->src_addr, 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;
        
        if (!esp_now_is_peer_exist(recv_info->src_addr)) {
            esp_now_add_peer(&peerInfo);
        }
        
        // Send our current vehicle data back for V2V communication
        if (hasValidCanData) {
            Item response = lastValidCanData;
            uint8_t mac[6];
            esp_wifi_get_mac(WIFI_IF_STA, mac);
            memcpy(response.MacAddress, mac, 6);
            response.timestamp = pdTICKS_TO_MS(xTaskGetTickCount());
            
            esp_err_t err = esp_now_send(recv_info->src_addr, (uint8_t *)&response, sizeof(response));
            if (err == ESP_OK) {
                debug_print("[V2V] Sent our vehicle data back successfully\n");
            }
        }
        
        // Forward to Jetson for logging/processing
        if (xQueueSend(NowUSBQueue, &received_vehicle_item, 0) != pdTRUE) {
            // Queue full - drop packet silently
        }
        
        return;
    }
    
    debug_print("Received unknown data format, length: %d\n", len);
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

// V2V broadcasting function
void broadcast_v2v_data() {
    if (!hasValidCanData) return;
    
    Item v2v_data = lastValidCanData;
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    memcpy(v2v_data.MacAddress, mac, 6);
    v2v_data.timestamp = pdTICKS_TO_MS(xTaskGetTickCount());
    
    // Broadcast to all nearby vehicles (use broadcast MAC)
    uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_err_t result = esp_now_send(broadcast_mac, (uint8_t *)&v2v_data, sizeof(v2v_data));
    
    debug_print("[V2V] Broadcasted vehicle data to nearby vehicles\n");
}

// Distance calculation function
float calculate_distance_between_points(float lat1, float lon1, float lat2, float lon2) {
    // Convert degrees to radians
    float lat1_rad = lat1 * M_PI / 180.0f;
    float lon1_rad = lon1 * M_PI / 180.0f;
    float lat2_rad = lat2 * M_PI / 180.0f;
    float lon2_rad = lon2 * M_PI / 180.0f;
    
    // Haversine formula
    float dlat = lat2_rad - lat1_rad;
    float dlon = lon2_rad - lon1_rad;
    
    float a = sin(dlat/2) * sin(dlat/2) + 
              cos(lat1_rad) * cos(lat2_rad) * 
              sin(dlon/2) * sin(dlon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return 6371000.0f * c;  // Distance in meters
}

// Calculate distance to EVCS function
float calculate_distance_to_evcs() {
    if (!hasValidCanData || !lastValidCanData.gps_valid || !evcs_position_known) {
        return 999.0f;  // Return large distance if no GPS data
    }
    
    float lat1 = lastValidCanData.gps_latitude;
    float lon1 = lastValidCanData.gps_longitude;
    float lat2 = received_evcs_latitude;  // From EVCS message
    float lon2 = received_evcs_longitude;
    
    return calculate_distance_between_points(lat1, lon1, lat2, lon2);
}
