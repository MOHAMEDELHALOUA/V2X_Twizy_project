///////////////////////////////////////////// ESP32(1) OBU - V2G + V2V Communication
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

// Message types
#define MSG_TYPE_V2G 0x01
#define MSG_TYPE_V2V 0x02

// Timing intervals
#define V2V_BROADCAST_INTERVAL 1000    // 1 second for V2V safety data
#define V2G_RESPONSE_TIMEOUT 5000      // 5 seconds for V2G responses

// =============================================================================
// DATA STRUCTURES
// =============================================================================

// Original structure for communication with Jetson (unchanged)
typedef struct {
    unsigned short SOC;        
    float speedKmh;
    float odometerKm;
    float displaySpeed;
    uint8_t MacAddress[6];
} Item;

// V2G Structure - Battery/Charging related data (for EVCS communication)
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
    uint8_t message_type;          // 0x01 = V2G message
} vehicle_to_evcs_v2g_t;

// EVCS V2G Response Structure
typedef struct {
    uint8_t evcs_mac[6];           // EVCS MAC address
    float ac_voltage;              // AC voltage available (V)
    float max_current;             // Maximum current available (A)
    float max_power;               // Maximum power available (W)
    float cost_per_kwh;            // Cost per kWh
    float current_energy_delivered; // Total energy delivered this session (kWh)
    float current_cost;            // Current session cost
    bool charging_available;       // Slot available for charging
    uint32_t session_id;           // Unique session identifier
    unsigned long timestamp;       // Message timestamp
    uint8_t message_type;          // 0x01 = V2G message
} evcs_to_vehicle_v2g_t;

// V2V Structure - Vehicle dynamics for safety (for other vehicles)
typedef struct {
    uint8_t vehicle_mac[6];        // Vehicle MAC address
    float latitude;                // Vehicle GPS latitude
    float longitude;               // Vehicle GPS longitude
    float speed_kmh;               // Vehicle speed (km/h)
    float acceleration;            // Vehicle acceleration (m/s²)
    bool brake_status;             // Brake pedal pressed
    uint8_t gear_selection;        // Current gear (P=0, R=1, N=2, D=3)
    float heading;                 // Vehicle heading (degrees)
    unsigned long timestamp;       // Message timestamp
    uint8_t message_type;          // 0x02 = V2V message
} vehicle_v2v_data_t;

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

Item incomingReadings;
Item receivedItem; // Item received from jetson with real CAN data
Item lastValidCanData; // Store last valid CAN data

// V2G and V2V data structures
vehicle_to_evcs_v2g_t v2g_data;
evcs_to_vehicle_v2g_t evcs_response;
vehicle_v2v_data_t v2v_data;

// Communication queues and flags
QueueHandle_t NowUSBQueue;
bool hasValidCanData = false;
bool v2g_session_active = false;
bool charging_requested = false;

// Timing variables
unsigned long last_v2v_broadcast = 0;
unsigned long last_v2g_response = 0;

// Session management
uint32_t current_session_id = 0;
uint8_t evcs_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Will be updated when EVCS responds

// =============================================================================
// FUNCTION PROTOTYPES
// =============================================================================

extern "C" void app_main();
void init_usb_serial();
static void usb_serial_rx_task(void *pvParameter);
static void usb_serial_tx_task(void *pvParameter);
static void v2v_broadcast_task(void *pvParameter);
static void sendToJetson_usb(Item *data);
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len);
esp_err_t init_esp_now(void);
void updateCanDataFromReceived(const Item *received);
void prepare_v2g_data();
void prepare_v2v_data();
void send_v2g_to_evcs();
void broadcast_v2v_data();
void handle_evcs_response(const evcs_to_vehicle_v2g_t *response);
void handle_v2v_from_other_vehicle(const vehicle_v2v_data_t *v2v_msg);

// =============================================================================
// MAIN APPLICATION
// =============================================================================

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
    
    // Initialize data structures
    memset(&receivedItem, 0, sizeof(receivedItem));
    memset(&lastValidCanData, 0, sizeof(lastValidCanData));
    memset(&v2g_data, 0, sizeof(v2g_data));
    memset(&v2v_data, 0, sizeof(v2v_data));
    memset(&evcs_response, 0, sizeof(evcs_response));
    
    // Set default values
    lastValidCanData.SOC = 0;
    lastValidCanData.speedKmh = 0.0;
    lastValidCanData.displaySpeed = 0.0;
    lastValidCanData.odometerKm = 0.0;
    
    // Initialize V2G data with vehicle specs (example values - adjust as needed)
    v2g_data.battery_capacity = 50.0f;  // 50 kWh battery
    v2g_data.desired_soc = 80;          // Target 80% charge
    v2g_data.message_type = MSG_TYPE_V2G;
    
    // Initialize V2V data
    v2v_data.message_type = MSG_TYPE_V2V;
    
    // Create communication tasks
    xTaskCreate(usb_serial_rx_task, "usb_serial_rx_task", 4096, NULL, 12, NULL);
    xTaskCreate(usb_serial_tx_task, "usb_serial_tx_task", 4096, NULL, 11, NULL);
    xTaskCreate(v2v_broadcast_task, "v2v_broadcast_task", 4096, NULL, 10, NULL);
    
    printf("ESP32(1) OBU started - V2G+V2V Communication System\n");
    
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// =============================================================================
// USB SERIAL COMMUNICATION (UNCHANGED)
// =============================================================================

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
            
            // Update our CAN data store
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
    // Copy all data directly without any validation
    lastValidCanData = *received;
    hasValidCanData = true;
}

// =============================================================================
// V2V BROADCASTING TASK
// =============================================================================

static void v2v_broadcast_task(void *pvParameter)
{
    while (1) {
        unsigned long currentTime = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Broadcast V2V data every 1 second for safety
        if (hasValidCanData && (currentTime - last_v2v_broadcast >= V2V_BROADCAST_INTERVAL)) {
            last_v2v_broadcast = currentTime;
            broadcast_v2v_data();
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// =============================================================================
// DATA PREPARATION FUNCTIONS
// =============================================================================

void prepare_v2g_data() {
    if (!hasValidCanData) return;
    
    // Get this ESP32's MAC address
    esp_wifi_get_mac(WIFI_IF_STA, v2g_data.vehicle_mac);
    
    // Fill V2G data with battery-related information only
    v2g_data.battery_soc = lastValidCanData.SOC;
    v2g_data.battery_voltage = 400.0f;  // Example: 400V battery pack
    v2g_data.battery_current = 0.0f;    // Current flow (+ charging, - discharging)
    // battery_capacity and desired_soc already set in app_main
    v2g_data.ready_to_charge = true;    // Vehicle ready to charge
    v2g_data.stop_charging = false;     // Don't stop charging
    v2g_data.session_id = current_session_id;
    v2g_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void prepare_v2v_data() {
    if (!hasValidCanData) return;
    
    // Get this ESP32's MAC address
    esp_wifi_get_mac(WIFI_IF_STA, v2v_data.vehicle_mac);
    
    // Fill V2V data with vehicle dynamics for safety
    v2v_data.latitude = 33.986107f;     // GPS coordinates (should come from GPS module)
    v2v_data.longitude = -6.724805f;    
    v2v_data.speed_kmh = lastValidCanData.speedKmh;
    v2v_data.acceleration = 0.0f;       // Calculate from speed changes
    v2v_data.brake_status = false;      // Should come from brake pedal sensor
    v2v_data.gear_selection = 3;        // D=3 (should come from gear position sensor)
    v2v_data.heading = 0.0f;           // Vehicle heading in degrees
    v2v_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

// =============================================================================
// ESP-NOW COMMUNICATION FUNCTIONS
// =============================================================================

void send_v2g_to_evcs() {
    if (!hasValidCanData) return;
    
    prepare_v2g_data();
    
    // Send to specific EVCS MAC if known, otherwise broadcast
    esp_err_t result = esp_now_send(evcs_mac, (uint8_t *)&v2g_data, sizeof(v2g_data));
    
    if (result == ESP_OK) {
        printf("V2G data sent to EVCS: SOC=%u%%, Ready=%s\n",
               v2g_data.battery_soc, v2g_data.ready_to_charge ? "Yes" : "No");
    }
}

void broadcast_v2v_data() {
    if (!hasValidCanData) return;
    
    prepare_v2v_data();
    
    // Broadcast to all vehicles
    uint8_t broadcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_err_t result = esp_now_send(broadcast_mac, (uint8_t *)&v2v_data, sizeof(v2v_data));
    
    if (result == ESP_OK) {
        printf("V2V broadcast: Speed=%.1fkm/h, Gear=%u, Brake=%s\n",
               v2v_data.speed_kmh, v2v_data.gear_selection, 
               v2v_data.brake_status ? "ON" : "OFF");
    }
}

// =============================================================================
// ESP-NOW CALLBACKS
// =============================================================================

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    
    // Check message type by looking at the message_type field
    if (len >= sizeof(uint8_t)) {
        uint8_t msg_type = incomingData[len - sizeof(unsigned long) - sizeof(uint8_t)]; // message_type is before timestamp
        
        if (msg_type == MSG_TYPE_V2G && len == sizeof(evcs_to_vehicle_v2g_t)) {
            // Handle V2G response from EVCS
            evcs_to_vehicle_v2g_t evcs_msg;
            memcpy(&evcs_msg, incomingData, sizeof(evcs_msg));
            memcpy(evcs_msg.evcs_mac, recv_info->src_addr, 6);
            handle_evcs_response(&evcs_msg);
            
        } else if (msg_type == MSG_TYPE_V2V && len == sizeof(vehicle_v2v_data_t)) {
            // Handle V2V data from other vehicles
            vehicle_v2v_data_t other_v2v;
            memcpy(&other_v2v, incomingData, sizeof(other_v2v));
            memcpy(other_v2v.vehicle_mac, recv_info->src_addr, 6);
            handle_v2v_from_other_vehicle(&other_v2v);
            
        } else if (len == sizeof(Item)) {
            // Handle legacy communication (from other ESP32s using old format)
            Item item;
            memcpy(&item, incomingData, sizeof(Item));
            memcpy(item.MacAddress, recv_info->src_addr, 6);
            
            // Add peer if needed and send response
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, recv_info->src_addr, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;
            
            if (!esp_now_is_peer_exist(recv_info->src_addr)) {
                esp_now_add_peer(&peerInfo);
            }
            
            // Send our CAN data as response
            Item response;
            if (hasValidCanData) {
                response = lastValidCanData;
            } else {
                memset(&response, 0, sizeof(response));
            }
            
            // Set this ESP32's MAC in response
            esp_wifi_get_mac(WIFI_IF_STA, response.MacAddress);
            esp_now_send(recv_info->src_addr, (uint8_t *)&response, sizeof(response));
            
            // Forward to Jetson
            if (xQueueSend(NowUSBQueue, &item, 0) != pdTRUE) {
                // Queue full, drop packet silently
            }
        }
    }
}

void handle_evcs_response(const evcs_to_vehicle_v2g_t *response) {
    // Update EVCS MAC address
    memcpy(evcs_mac, response->evcs_mac, 6);
    
    printf("V2G Response from EVCS: Voltage=%.1fV, MaxPower=%.0fW, Cost=%.2f/kWh\n",
           response->ac_voltage, response->max_power, response->cost_per_kwh);
    
    if (response->charging_available && !v2g_session_active) {
        // Start V2G session
        v2g_session_active = true;
        current_session_id = response->session_id;
        printf("V2G session started with EVCS\n");
    }
    
    // Update charging status based on EVCS response
    last_v2g_response = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void handle_v2v_from_other_vehicle(const vehicle_v2v_data_t *v2v_msg) {
    printf("V2V from vehicle %02X:%02X:**: Speed=%.1fkm/h, Brake=%s\n",
           v2v_msg->vehicle_mac[4], v2v_msg->vehicle_mac[5],
           v2v_msg->speed_kmh, v2v_msg->brake_status ? "ON" : "OFF");
    
    // Here you can implement collision avoidance logic based on other vehicles' data
    // For example, check distance, relative speed, brake status, etc.
}

// =============================================================================
// ESP-NOW INITIALIZATION
// =============================================================================

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
    
    // Add broadcast peer
    esp_now_peer_info_t peerInfo = {};
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, evcs_mac, 6); // Initially broadcast
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    
    return ESP_OK;
}
