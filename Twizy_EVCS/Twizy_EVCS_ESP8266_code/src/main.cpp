// EVCS_ESP32_V2G_Production.ino - PZEM-004T + V2G with Raspberry Pi compatibility
#include <PZEM004Tv30.h>
#include <cmath>
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <stdbool.h>
 
// Define the UART2 RX and TX pins on ESP32 (Connect these to PZEM-004T)
#define PZEM_RX_PIN 16  // ESP32 RX (Connect to PZEM TX)
#define PZEM_TX_PIN 17  // ESP32 TX (Connect to PZEM RX)

// Fixed EVCS Location (Charging Station doesn't move)
#define EVCS_LATITUDE  33.986107f
#define EVCS_LONGITUDE -6.724805f

// Configuration flags
#define DEBUG_V2G true          // Set to true for V2G debugging
#define DEBUG_PZEM true         // Set to true for PZEM debugging
#define ENABLE_V2G true          // Set to false to disable V2G completely

// Charging parameters
#define CHARGING_RATE_PER_KWH 1.50f  // Cost per kWh
#define MAX_CHARGING_POWER 3000.0f   // Max 3kW (Type F)
#define CURRENT_THRESHOLD 0.5f       // Minimum current to detect vehicle

// Timing
unsigned long last_reading = 0;
unsigned long last_v2g_send = 0;
const unsigned long READING_INTERVAL = 2000;    // 2 seconds for PZEM readings
const unsigned long V2G_SEND_INTERVAL = 5000;   // 5 seconds for V2G communication

// Global variables
short charging_status = 0;
bool vehicle_connected = false;
float session_start_energy = 0.0f;
unsigned long session_start_time = 0;

// Initialize the PZEM sensor using Hardware Serial2
PZEM004Tv30 pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);

// Struct to send to Vehicle (EVCS -> Vehicle)
//typedef struct {
//    uint8_t evcs_mac[6];           // EVCS MAC address
//    float ac_voltage;              // AC voltage available (V)
//    float max_current;             // Maximum current available (A)
//    float max_power;               // Maximum power available (W)
//    float cost_per_kwh;            // Cost per kWh
//    float current_energy_delivered; // Total energy delivered this session (kWh)
//    float current_cost;            // Current session cost
//    bool charging_available;       // Slot available for charging
//    uint32_t session_id;           // Unique session identifier
//    unsigned long timestamp;       // Message timestamp
//} evcs_to_vehicle_t;

// EVCS → Vehicle struct (with EVCS position)
typedef struct {
    uint8_t evcs_mac[6];           // EVCS MAC address
    
    // EVCS Position (NEW - CRITICAL for distance calculation)
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

// Struct to receive from Vehicle (Vehicle -> EVCS)
//typedef struct {
//    uint8_t vehicle_mac[6];        // Vehicle MAC address
//    uint16_t battery_soc;          // State of Charge (0-100%)
//    float battery_voltage;         // Battery voltage (V)
//    float battery_current;         // Current battery current (A)
//    float battery_capacity;        // Total battery capacity (kWh)
//    float desired_soc;             // Desired SOC (0-100%)
//    bool ready_to_charge;          // Vehicle ready to start charging
//    bool stop_charging;            // Vehicle wants to stop charging
//    uint32_t session_id;           // Session ID (should match EVCS)
//    unsigned long timestamp;       // Message timestamp
//} vehicle_to_evcs_t;

// Vehicle → EVCS struct (with vehicle position)
typedef struct {
    uint8_t vehicle_mac[6];        // Vehicle MAC address
    
    // Vehicle Position (NEW - for EVCS to know vehicle location)
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

// Global data structures
evcs_to_vehicle_t evcs_data;
vehicle_to_evcs_t vehicle_data;
bool vehicle_data_received = false;

// Vehicle MAC address (will be updated when vehicle connects)
uint8_t vehicle_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Broadcast initially
esp_now_peer_info_t peerInfo;

// Current session data
uint32_t current_session_id = 0;
bool session_active = false;

// Function prototypes
void init_esp_now();
void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status);
void on_data_received(const uint8_t *mac, const uint8_t *incoming_data, int len);
void send_v2g_data();
void start_charging_session();
void end_charging_session();
void handle_charging_session();
void update_evcs_data();

// Debug print functions (only print when DEBUG flags are enabled)
void debug_v2g(const char* format, ...) {
    if (DEBUG_V2G) {
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        Serial2.println(buffer);  // Send to Serial2 (different from Serial)
        va_end(args);
    }
}

void debug_pzem(const char* format, ...) {
    if (DEBUG_PZEM) {
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        Serial2.println(buffer);  // Send to Serial2 (different from Serial)
        va_end(args);
    }
}
// DISTANCE CALCULATION FUNCTION
float calculate_distance_between_points(float lat1, float lon1, float lat2, float lon2);

//void setup() {
//    Serial.begin(115200);
//    
//    // CRITICAL: Only send essential startup message to maintain Raspberry Pi compatibility
//    if (DEBUG_V2G) {
//        Serial.println("HELECAR EVCS - V2G Communication System");
//        Serial.println("=======================================");
//    }
//    
//    if (ENABLE_V2G) {
//        // Initialize WiFi in Station mode for ESP-NOW
//        WiFi.mode(WIFI_STA);
//        
//        // Get this ESP32's MAC address
//        WiFi.macAddress(evcs_data.evcs_mac);
//        
//        if (DEBUG_V2G) {
//            Serial.print("EVCS MAC Address: ");
//            for (int i = 0; i < 6; i++) {
//                Serial.printf("%02X", evcs_data.evcs_mac[i]);
//                if (i < 5) Serial.print(":");
//            }
//            Serial.println();
//        }
//        
//        // Initialize ESP-NOW
//        init_esp_now();
//        
//        // Initialize EVCS data with default values
//        evcs_data.max_current = 13.0f;        // 13A max for Type F
//        evcs_data.max_power = MAX_CHARGING_POWER;
//        evcs_data.cost_per_kwh = CHARGING_RATE_PER_KWH;
//        evcs_data.charging_available = true;
//        evcs_data.current_energy_delivered = 0.0f;
//        evcs_data.current_cost = 0.0f;
//        
//        // Generate initial session ID
//        current_session_id = millis();
//        evcs_data.session_id = current_session_id;
//    }
//    
//    delay(1000);
//}

void setup() {
    Serial.begin(115200);
    
    if (ENABLE_V2G) {
        WiFi.mode(WIFI_STA);
        WiFi.macAddress(evcs_data.evcs_mac);
        
        // Initialize EVCS data with POSITION (NEW)
        evcs_data.evcs_latitude = EVCS_LATITUDE;      // 33.986107f
        evcs_data.evcs_longitude = EVCS_LONGITUDE;    // -6.724805f
        evcs_data.evcs_altitude = 0.0f;               // Set if known
        
        evcs_data.max_current = 13.0f;
        evcs_data.max_power = MAX_CHARGING_POWER;
        evcs_data.cost_per_kwh = CHARGING_RATE_PER_KWH;
        evcs_data.charging_available = true;
        evcs_data.current_energy_delivered = 0.0f;
        evcs_data.current_cost = 0.0f;
        
        current_session_id = millis();
        evcs_data.session_id = current_session_id;
        
        init_esp_now();
        
        debug_v2g("EVCS Position: %.6f, %.6f", 
                 evcs_data.evcs_latitude, evcs_data.evcs_longitude);
    }
    
    delay(1000);
}

void loop() {
    unsigned long currentTime = millis();
    
    // Read PZEM data every 2 seconds
    if (currentTime - last_reading >= READING_INTERVAL) {
        last_reading = currentTime;
        
        // Read data from the PZEM sensor
        float voltage = pzem.voltage();
        float current = pzem.current();
        float power = pzem.power();
        float energy = pzem.energy();
        float frequency = pzem.frequency();
        float pf = pzem.pf();
        
        // Error handling
        if (isnan(voltage) || isnan(current) || isnan(power) || 
            isnan(energy) || isnan(frequency) || isnan(pf)) {
            debug_pzem("ERROR: PZEM sensor reading failed");
            return;
        }
        
        // Update EVCS data with current measurements (for V2G)
        if (ENABLE_V2G) {
            evcs_data.ac_voltage = voltage;
            evcs_data.timestamp = currentTime;
        }
        
        // Determine charging status based on current
        bool currently_charging = (current > CURRENT_THRESHOLD);
        
        if (ENABLE_V2G) {
            if (currently_charging && !vehicle_connected) {
                // Vehicle just connected
                vehicle_connected = true;
                start_charging_session();
            } else if (!currently_charging && vehicle_connected) {
                // Vehicle disconnected
                vehicle_connected = false;
                end_charging_session();
            }
            
            // Handle charging session
            if (session_active) {
                handle_charging_session();
            }
        }
        
        // Update charging status
        charging_status = currently_charging ? 1 : 0;
        
        // CRITICAL: Send data to Raspberry Pi in EXACT original format
        // This MUST remain unchanged for Raspberry Pi compatibility
        Serial.print("EVSE");
        Serial.print(charging_status);
        Serial.print(" ");
        Serial.print(EVCS_LATITUDE, 6);
        Serial.print(" ");
        Serial.print(EVCS_LONGITUDE, 6);
        Serial.print(" ");
        Serial.print(voltage, 1);
        Serial.print(" ");
        Serial.print(current, 2);
        Serial.print(" ");
        Serial.print(power, 1);
        Serial.print(" ");
        Serial.print(energy, 3);
        Serial.print(" ");
        Serial.print((int)frequency);
        Serial.print(" ");
        Serial.print(pf, 2);
        Serial.println();
    }
    
    // Send V2G data every 5 seconds when vehicle is connected (only if V2G enabled)
    if (ENABLE_V2G && vehicle_connected && (currentTime - last_v2g_send >= V2G_SEND_INTERVAL)) {
        last_v2g_send = currentTime;
        send_v2g_data();
    }
    
    delay(100);
}

void init_esp_now() {
    if (!ENABLE_V2G) return;
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        debug_v2g("ERROR: ESP-NOW initialization failed");
        return;
    }
    
    // Register callbacks
    esp_now_register_send_cb(on_data_sent);
    esp_now_register_recv_cb(on_data_received);
    
    // Add broadcast peer initially
    memcpy(peerInfo.peer_addr, vehicle_mac, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        debug_v2g("ERROR: Failed to add broadcast peer");
        return;
    }
    
    debug_v2g("ESP-NOW V2G communication initialized");
}

void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        debug_v2g("[V2G] Data sent to vehicle successfully");
    } else {
        debug_v2g("[V2G] Failed to send data to vehicle");
    }
}


void send_v2g_data() {
    if (!ENABLE_V2G) return;
    
    update_evcs_data();
    
    esp_err_t result = esp_now_send(vehicle_mac, (uint8_t *)&evcs_data, sizeof(evcs_data));
    
    if (result == ESP_OK) {
        debug_v2g("[V2G] EVCS data sent to vehicle");
        if (DEBUG_V2G && session_active) {
            debug_v2g("[V2G] Session Energy: %.3f kWh | Cost: %.2f",
                     evcs_data.current_energy_delivered, evcs_data.current_cost);
        }
    } else {
        debug_v2g("[V2G] ERROR: Failed to send data (error: %d)", result);
    }
}

void start_charging_session() {
    if (!ENABLE_V2G || session_active) return;
    
    session_active = true;
    session_start_time = millis();
    session_start_energy = pzem.energy();
    current_session_id++;
    evcs_data.session_id = current_session_id;
    
    debug_v2g("[V2G] CHARGING SESSION STARTED - ID: %u", current_session_id);
}

void end_charging_session() {
    if (!ENABLE_V2G || !session_active) return;
    
    session_active = false;
    float final_energy = pzem.energy();
    float energy_delivered = final_energy - session_start_energy;
    float total_cost = energy_delivered * CHARGING_RATE_PER_KWH;
    unsigned long session_duration = millis() - session_start_time;
    
    debug_v2g("[V2G] CHARGING SESSION COMPLETED");
    debug_v2g("[V2G] Energy: %.3f kWh, Cost: %.2f, Duration: %lu min",
             energy_delivered, total_cost, session_duration / 60000);
    
    // Reset session data
    evcs_data.current_energy_delivered = 0.0f;
    evcs_data.current_cost = 0.0f;
}

void handle_charging_session() {
    if (!ENABLE_V2G || !session_active) return;
    
    float current_energy = pzem.energy();
    evcs_data.current_energy_delivered = current_energy - session_start_energy;
    evcs_data.current_cost = evcs_data.current_energy_delivered * CHARGING_RATE_PER_KWH;
}

//void update_evcs_data() {
//    if (!ENABLE_V2G) return;
//    
//    evcs_data.charging_available = !session_active;
//    evcs_data.timestamp = millis();
//}
//
void update_evcs_data() {
    if (!ENABLE_V2G) return;
    
    evcs_data.charging_available = !session_active;
    evcs_data.timestamp = millis();
    
    // Position data always included (EVCS doesn't move)
    evcs_data.evcs_latitude = EVCS_LATITUDE;
    evcs_data.evcs_longitude = EVCS_LONGITUDE;
    evcs_data.evcs_altitude = 0.0f;
}

void on_data_received(const uint8_t *mac, const uint8_t *incoming_data, int len) {
    if (!ENABLE_V2G) return;
    
    if (len != sizeof(vehicle_to_evcs_t)) {
        debug_v2g("[V2G] ERROR: Unexpected data length: %d (expected %d)", 
                 len, sizeof(vehicle_to_evcs_t));
        return;
    }
    
    // Copy received data
    memcpy(&vehicle_data, incoming_data, sizeof(vehicle_to_evcs_t));
    memcpy(vehicle_data.vehicle_mac, mac, 6);
    vehicle_data_received = true;
    
    // Print received vehicle data with POSITION (NEW)
    if (DEBUG_V2G) {
        debug_v2g("[V2G] Vehicle MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 vehicle_data.vehicle_mac[0], vehicle_data.vehicle_mac[1],
                 vehicle_data.vehicle_mac[2], vehicle_data.vehicle_mac[3],
                 vehicle_data.vehicle_mac[4], vehicle_data.vehicle_mac[5]);
        debug_v2g("[V2G] Vehicle Position: %.6f, %.6f (GPS: %s)",
                 vehicle_data.vehicle_latitude, vehicle_data.vehicle_longitude,
                 vehicle_data.gps_valid ? "Valid" : "Invalid");
        debug_v2g("[V2G] Battery: SOC=%u%%, Voltage=%.1fV, Current=%.1fA",
                 vehicle_data.battery_soc, vehicle_data.battery_voltage, vehicle_data.battery_current);
        
        // Calculate distance to vehicle (NEW FEATURE)
        if (vehicle_data.gps_valid) {
            float distance = calculate_distance_between_points(
                EVCS_LATITUDE, EVCS_LONGITUDE,
                vehicle_data.vehicle_latitude, vehicle_data.vehicle_longitude
            );
            debug_v2g("[V2G] Distance to vehicle: %.1f meters", distance);
        }
    }
    
    // Update vehicle MAC if it's a new vehicle
    bool is_new_vehicle = false;
    for (int i = 0; i < 6; i++) {
        if (vehicle_mac[i] != mac[i]) {
            is_new_vehicle = true;
            break;
        }
    }
    
    if (is_new_vehicle) {
        debug_v2g("[V2G] New vehicle detected, updating peer MAC");
        memcpy(vehicle_mac, mac, 6);
        
        esp_now_del_peer(peerInfo.peer_addr);
        memcpy(peerInfo.peer_addr, vehicle_mac, 6);
        esp_now_add_peer(&peerInfo);
    }
    
    // Handle vehicle requests
    if (vehicle_data.stop_charging && session_active) {
        debug_v2g("[V2G] Vehicle requested to stop charging");
        end_charging_session();
    }
}
// DISTANCE CALCULATION FUNCTION
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

