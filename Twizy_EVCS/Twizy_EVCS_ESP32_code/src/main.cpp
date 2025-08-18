// EVCS_ESP32_V2G_Enhanced.ino - PZEM-004T + V2G with Session Management
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

// Charging parameters
#define CHARGING_RATE_PER_KWH 1.50f  // Cost per kWh
#define MAX_CHARGING_POWER 3000.0f   // Max 3kW (Type F)
#define CURRENT_THRESHOLD 0.5f       // Minimum current to detect vehicle

// V2G Session Management
#define V2G_SESSION_TIMEOUT_MS 30000     // 30 seconds without charging = end session
#define V2G_COMMUNICATION_TIMEOUT_MS 15000  // 15 seconds without V2G messages = disconnect
#define SESSION_END_DELAY_MS 5000        // 5 seconds delay after charging stops before ending session

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

// V2G Session Management Variables
unsigned long last_charging_time = 0;          // Last time current > threshold
unsigned long last_v2g_received = 0;           // Last time we received V2G data
unsigned long charging_stopped_time = 0;       // When charging stopped
bool session_ending = false;                   // Flag to indicate session is ending

// Initialize the PZEM sensor using Hardware Serial2
PZEM004Tv30 pzem(Serial2, PZEM_RX_PIN, PZEM_TX_PIN);

// Struct to send to Vehicle (EVCS -> Vehicle)
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
    bool session_active;           // NEW: Indicates if session is still active
} evcs_to_vehicle_t;

// Struct to receive from Vehicle (Vehicle -> EVCS)
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
bool v2g_connected = false;

// Function prototypes
void init_esp_now();
void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status);
void on_data_received(const uint8_t *mac, const uint8_t *incoming_data, int len);
void send_v2g_data();
void start_charging_session();
void end_charging_session();
void handle_charging_session();
void update_evcs_data();
void print_vehicle_data();
void print_session_info();
void check_session_timeouts();
void disconnect_v2g_session();

void setup() {
    Serial.begin(115200);
    Serial.println("HELECAR EVCS - V2G Communication System with Session Management");
    Serial.println("==============================================================");
    Serial.println("Features: PZEM-004T + ESP-NOW V2G + Auto Session Timeout");
    Serial.println();
    
    // Initialize WiFi in Station mode for ESP-NOW
    WiFi.mode(WIFI_STA);
    
    // Get this ESP32's MAC address
    WiFi.macAddress(evcs_data.evcs_mac);
    Serial.print("EVCS MAC Address: ");
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", evcs_data.evcs_mac[i]);
        if (i < 5) Serial.print(":");
    }
    Serial.println();
    
    // Initialize ESP-NOW
    init_esp_now();
    
    // Initialize EVCS data with default values
    evcs_data.max_current = 13.0f;        // 13A max for Type F
    evcs_data.max_power = MAX_CHARGING_POWER;
    evcs_data.cost_per_kwh = CHARGING_RATE_PER_KWH;
    evcs_data.charging_available = true;
    evcs_data.current_energy_delivered = 0.0f;
    evcs_data.current_cost = 0.0f;
    evcs_data.session_active = false;
    
    // Generate initial session ID
    current_session_id = millis();
    evcs_data.session_id = current_session_id;
    
    Serial.println("EVCS V2G system initialized and ready!");
    Serial.println("Session Management:");
    Serial.printf("  - V2G timeout: %d seconds\n", V2G_COMMUNICATION_TIMEOUT_MS / 1000);
    Serial.printf("  - Session timeout: %d seconds after charging stops\n", SESSION_END_DELAY_MS / 1000);
    Serial.println("Waiting for vehicle connection...");
    Serial.println();
    
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
            Serial.println("ERROR: PZEM sensor reading failed");
            return;
        }
        
        // Update EVCS data with current measurements
        evcs_data.ac_voltage = voltage;
        evcs_data.timestamp = currentTime;
        
        // Determine charging status based on current
        bool currently_charging = (current > CURRENT_THRESHOLD);
        
        // Update last charging time if currently charging
        if (currently_charging) {
            last_charging_time = currentTime;
            if (!vehicle_connected) {
                // Vehicle just connected
                vehicle_connected = true;
                start_charging_session();
            }
        } else {
            // Not charging - check if this is a recent change
            if (vehicle_connected && !session_ending) {
                // Vehicle was charging but stopped
                charging_stopped_time = currentTime;
                session_ending = true;
                Serial.println("⚠️ Charging stopped - starting session timeout");
            }
        }
        
        // Update charging status
        charging_status = currently_charging ? 1 : 0;
        
        // Handle charging session
        if (session_active) {
            handle_charging_session();
        }
        
        // Check for session timeouts
        check_session_timeouts();
        
        // Send data to Raspberry Pi (original format)
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
    
    // Send V2G data every 5 seconds when V2G is connected and session is active
    if (v2g_connected && session_active && (currentTime - last_v2g_send >= V2G_SEND_INTERVAL)) {
        last_v2g_send = currentTime;
        send_v2g_data();
    }
    
    delay(100);
}

void check_session_timeouts() {
    unsigned long currentTime = millis();
    
    // Check V2G communication timeout
    if (v2g_connected && session_active) {
        if (currentTime - last_v2g_received > V2G_COMMUNICATION_TIMEOUT_MS) {
            Serial.println("⚠️ V2G communication timeout - disconnecting vehicle");
            disconnect_v2g_session();
        }
    }
    
    // Check session end timeout after charging stops
    if (session_ending && (currentTime - charging_stopped_time > SESSION_END_DELAY_MS)) {
        Serial.println("⏰ Session timeout reached - ending session");
        end_charging_session();
    }
    
    // Check for session timeout without any charging activity
    if (session_active && (currentTime - last_charging_time > V2G_SESSION_TIMEOUT_MS)) {
        Serial.println("⚠️ Session timeout - no charging activity");
        end_charging_session();
    }
}

void disconnect_v2g_session() {
    if (v2g_connected) {
        Serial.println("🔌 DISCONNECTING V2G SESSION 🔌");
        Serial.printf("Vehicle MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                     vehicle_mac[0], vehicle_mac[1], vehicle_mac[2],
                     vehicle_mac[3], vehicle_mac[4], vehicle_mac[5]);
        
        // Remove the vehicle as a peer
        esp_now_del_peer(vehicle_mac);
        
        // Reset V2G connection
        v2g_connected = false;
        vehicle_data_received = false;
        memset(vehicle_mac, 0xFF, 6);  // Reset to broadcast
        
        // Update EVCS data to indicate session ended
        evcs_data.session_active = false;
        evcs_data.charging_available = true;
        
        Serial.println("V2G session disconnected");
        Serial.println("Ready for new vehicle connection");
        Serial.println("================================\n");
    }
}

void init_esp_now() {
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ERROR: ESP-NOW initialization failed");
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
        Serial.println("ERROR: Failed to add broadcast peer");
        return;
    }
    
    Serial.println("ESP-NOW V2G communication initialized");
}

void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("[V2G] Data sent to vehicle successfully");
    } else {
        Serial.println("[V2G] Failed to send data to vehicle");
    }
}

void on_data_received(const uint8_t *mac, const uint8_t *incoming_data, int len) {
    if (len != sizeof(vehicle_to_evcs_t)) {
        Serial.printf("[V2G] ERROR: Unexpected data length: %d (expected %d)\n", 
                     len, sizeof(vehicle_to_evcs_t));
        return;
    }
    
    // Update last V2G received time
    last_v2g_received = millis();
    
    // Copy received data
    memcpy(&vehicle_data, incoming_data, sizeof(vehicle_to_evcs_t));
    memcpy(vehicle_data.vehicle_mac, mac, 6);
    vehicle_data_received = true;
    
    // Print received vehicle data
    print_vehicle_data();
    
    // Check if this is a new vehicle or existing vehicle
    bool is_new_vehicle = !v2g_connected;
    if (v2g_connected) {
        // Check if MAC address changed
        for (int i = 0; i < 6; i++) {
            if (vehicle_mac[i] != mac[i]) {
                is_new_vehicle = true;
                break;
            }
        }
    }
    
    if (is_new_vehicle && session_active) {
        // New vehicle connecting during active session
        Serial.println("[V2G] New vehicle detected during active session");
        
        // Remove old peer if exists
        if (v2g_connected) {
            esp_now_del_peer(vehicle_mac);
        }
        
        // Update to new vehicle
        memcpy(vehicle_mac, mac, 6);
        v2g_connected = true;
        
        // Add new peer
        memcpy(peerInfo.peer_addr, vehicle_mac, 6);
        esp_now_add_peer(&peerInfo);
        
        Serial.printf("[V2G] Connected to vehicle: %02X:%02X:%02X:%02X:%02X:%02X\n",
                     vehicle_mac[0], vehicle_mac[1], vehicle_mac[2],
                     vehicle_mac[3], vehicle_mac[4], vehicle_mac[5]);
    } else if (!v2g_connected && session_active) {
        // First V2G connection for this session
        memcpy(vehicle_mac, mac, 6);
        v2g_connected = true;
        
        // Add as peer
        memcpy(peerInfo.peer_addr, vehicle_mac, 6);
        esp_now_add_peer(&peerInfo);
        
        Serial.println("[V2G] 🚗 V2G CONNECTION ESTABLISHED 🚗");
        Serial.printf("Vehicle MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                     vehicle_mac[0], vehicle_mac[1], vehicle_mac[2],
                     vehicle_mac[3], vehicle_mac[4], vehicle_mac[5]);
    }
    
    // Handle vehicle requests
    if (vehicle_data.stop_charging && session_active) {
        Serial.println("[V2G] Vehicle requested to stop charging");
        // Don't end session immediately, wait for physical disconnect
    }
}

void send_v2g_data() {
    update_evcs_data();
    
    esp_err_t result = esp_now_send(vehicle_mac, (uint8_t *)&evcs_data, sizeof(evcs_data));
    
    if (result == ESP_OK) {
        Serial.println("[V2G] EVCS data sent to vehicle");
        print_session_info();
    } else {
        Serial.printf("[V2G] ERROR: Failed to send data (error: %d)\n", result);
    }
}

void start_charging_session() {
    if (!session_active) {
        session_active = true;
        session_ending = false;
        session_start_time = millis();
        session_start_energy = pzem.energy();
        current_session_id++;
        evcs_data.session_id = current_session_id;
        evcs_data.session_active = true;
        
        Serial.println("\n🔌 CHARGING SESSION STARTED 🔌");
        Serial.printf("Session ID: %u\n", current_session_id);
        Serial.printf("Start Energy: %.3f kWh\n", session_start_energy);
        Serial.printf("Start Time: %lu ms\n", session_start_time);
        Serial.println("Waiting for V2G connection...");
        Serial.println("=====================================\n");
    }
}

void end_charging_session() {
    if (session_active) {
        session_active = false;
        session_ending = false;
        vehicle_connected = false;
        
        float final_energy = pzem.energy();
        float energy_delivered = final_energy - session_start_energy;
        float total_cost = energy_delivered * CHARGING_RATE_PER_KWH;
        unsigned long session_duration = millis() - session_start_time;
        
        Serial.println("\n🔋 CHARGING SESSION COMPLETED 🔋");
        Serial.printf("Session ID: %u\n", current_session_id);
        Serial.printf("Energy Delivered: %.3f kWh\n", energy_delivered);
        Serial.printf("Total Cost: %.2f\n", total_cost);
        Serial.printf("Duration: %lu minutes\n", session_duration / 60000);
        Serial.printf("Average Power: %.1f kW\n", 
                     energy_delivered / (session_duration / 3600000.0f));
        Serial.println("=====================================\n");
        
        // Disconnect V2G session
        disconnect_v2g_session();
        
        // Reset session data
        evcs_data.current_energy_delivered = 0.0f;
        evcs_data.current_cost = 0.0f;
        evcs_data.session_active = false;
        evcs_data.charging_available = true;
    }
}

void handle_charging_session() {
    if (session_active) {
        float current_energy = pzem.energy();
        evcs_data.current_energy_delivered = current_energy - session_start_energy;
        evcs_data.current_cost = evcs_data.current_energy_delivered * CHARGING_RATE_PER_KWH;
    }
}

void update_evcs_data() {
    evcs_data.charging_available = !session_active;
    evcs_data.session_active = session_active;
    evcs_data.timestamp = millis();
}

void print_vehicle_data() {
    Serial.println("\n📱 VEHICLE DATA RECEIVED 📱");
    Serial.printf("Vehicle MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                 vehicle_data.vehicle_mac[0], vehicle_data.vehicle_mac[1],
                 vehicle_data.vehicle_mac[2], vehicle_data.vehicle_mac[3],
                 vehicle_data.vehicle_mac[4], vehicle_data.vehicle_mac[5]);
    Serial.printf("Battery SOC: %u%%\n", vehicle_data.battery_soc);
    Serial.printf("Battery Voltage: %.1f V\n", vehicle_data.battery_voltage);
    Serial.printf("Battery Current: %.1f A\n", vehicle_data.battery_current);
    Serial.printf("Battery Capacity: %.1f kWh\n", vehicle_data.battery_capacity);
    Serial.printf("Desired SOC: %.1f%%\n", vehicle_data.desired_soc);
    Serial.printf("Ready to Charge: %s\n", vehicle_data.ready_to_charge ? "YES" : "NO");
    Serial.printf("Stop Charging: %s\n", vehicle_data.stop_charging ? "YES" : "NO");
    Serial.printf("Session ID: %lu\n", vehicle_data.session_id);
    Serial.println("===============================\n");
}

void print_session_info() {
    if (session_active) {
        Serial.printf("[V2G] Session Energy: %.3f kWh | Cost: %.2f | Duration: %lu min | V2G: %s\n",
                     evcs_data.current_energy_delivered,
                     evcs_data.current_cost,
                     (millis() - session_start_time) / 60000,
                     v2g_connected ? "Connected" : "Disconnected");
    }
}
