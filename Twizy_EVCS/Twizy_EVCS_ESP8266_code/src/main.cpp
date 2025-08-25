// EVCS_ESP8266_V2G_Production.ino
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <espnow.h>
#include <SoftwareSerial.h>
#include <PZEM004Tv30.h>
#include <cmath>

// === PIN DEFINITIONS ===
#define PZEM_RX_PIN 14  // D5 → GPIO14 → PZEM TX
#define PZEM_TX_PIN 12  // D6 → GPIO12 → PZEM RX

// === EVCS LOCATION (Fixed) ===
#define EVCS_LATITUDE  33.986107f
#define EVCS_LONGITUDE -6.724805f

// === DEBUG & FUNCTIONALITY FLAGS ===
#define DEBUG_V2G true
#define DEBUG_PZEM true
#define ENABLE_V2G true

// === CHARGING PARAMETERS ===
#define CHARGING_RATE_PER_KWH 1.50f
#define MAX_CHARGING_POWER 3000.0f
#define CURRENT_THRESHOLD 0.5f  // Minimum current to detect vehicle connection

// === TIMING ===
unsigned long last_reading = 0;
unsigned long last_v2g_send = 0;
const unsigned long READING_INTERVAL = 2000;    // Read PZEM every 2s
const unsigned long V2G_SEND_INTERVAL = 5000;   // Send V2G every 5s

// === PZEM SENSOR SETUP ===
SoftwareSerial pzemSerial(PZEM_RX_PIN, PZEM_TX_PIN);
PZEM004Tv30 pzem(pzemSerial);  // Pass by reference (NOT &pzemSerial)

// === GLOBAL VARIABLES ===
short charging_status = 0;
bool vehicle_connected = false;
float session_start_energy = 0.0f;
unsigned long session_start_time = 0;
bool session_active = false;
uint32_t current_session_id = 0;

// === V2G DATA STRUCTS ===
typedef struct {
    uint8_t evcs_mac[6];
    float evcs_latitude;
    float evcs_longitude;
    float evcs_altitude;
    float ac_voltage;
    float max_current;
    float max_power;
    float cost_per_kwh;
    float current_energy_delivered;
    float current_cost;
    bool charging_available;
    uint32_t session_id;
    unsigned long timestamp;
} evcs_to_vehicle_t;

typedef struct {
    uint8_t vehicle_mac[6];
    float vehicle_latitude;
    float vehicle_longitude;
    float vehicle_altitude;
    uint8_t gps_valid;
    uint16_t battery_soc;
    float battery_voltage;
    float battery_current;
    float battery_capacity;
    float desired_soc;
    bool ready_to_charge;
    bool stop_charging;
    uint32_t session_id;
    unsigned long timestamp;
} vehicle_to_evcs_t;

// === GLOBAL DATA INSTANCES ===
evcs_to_vehicle_t evcs_data;
vehicle_to_evcs_t vehicle_data;
bool vehicle_data_received = false;

// === VEHICLE COMMUNICATION ===
uint8_t vehicle_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // Initially broadcast
uint8_t peer_addr[6];  // Store current vehicle MAC

// === FUNCTION PROTOTYPES ===
void init_esp_now();
void on_data_sent(uint8_t *mac_addr, uint8_t sendStatus);
void on_data_received(uint8_t *mac, uint8_t *data, uint8_t len);
void send_v2g_data();
void start_charging_session();
void end_charging_session();
void handle_charging_session();
void update_evcs_data();
float calculate_distance_between_points(float lat1, float lon1, float lat2, float lon2);

// === DEBUG FUNCTIONS ===
void debug_v2g(const char* format, ...) {
    if (DEBUG_V2G) {
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        Serial.println(buffer);
        va_end(args);
    }
}

void debug_pzem(const char* format, ...) {
    if (DEBUG_PZEM) {
        va_list args;
        va_start(args, format);
        char buffer[256];
        vsnprintf(buffer, sizeof(buffer), format, args);
        Serial.println(buffer);
        va_end(args);
    }
}

// === SETUP ===
void setup() {
    Serial.begin(115200);
    delay(100);

    Serial.println("EVCS System Starting...");

    if (ENABLE_V2G) {
        WiFi.mode(WIFI_STA);
        WiFi.macAddress(evcs_data.evcs_mac);

        // Initialize EVCS data
        evcs_data.evcs_latitude = EVCS_LATITUDE;
        evcs_data.evcs_longitude = EVCS_LONGITUDE;
        evcs_data.evcs_altitude = 0.0f;

        evcs_data.max_current = 13.0f;
        evcs_data.max_power = MAX_CHARGING_POWER;
        evcs_data.cost_per_kwh = CHARGING_RATE_PER_KWH;
        evcs_data.charging_available = true;
        evcs_data.current_energy_delivered = 0.0f;
        evcs_data.current_cost = 0.0f;

        current_session_id = millis();
        evcs_data.session_id = current_session_id;

        init_esp_now();

        debug_v2g("EVCS MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                 evcs_data.evcs_mac[0], evcs_data.evcs_mac[1],
                 evcs_data.evcs_mac[2], evcs_data.evcs_mac[3],
                 evcs_data.evcs_mac[4], evcs_data.evcs_mac[5]);
        debug_v2g("EVCS Position: %.6f, %.6f", evcs_data.evcs_latitude, evcs_data.evcs_longitude);
    }

    delay(1000);
}

// === LOOP ===
void loop() {
    unsigned long currentTime = millis();

    // Read PZEM sensor every 2 seconds
    if (currentTime - last_reading >= READING_INTERVAL) {
        last_reading = currentTime;

        float voltage = pzem.voltage();
        float current = pzem.current();
        float power = pzem.power();
        float energy = pzem.energy();
        float frequency = pzem.frequency();
        float pf = pzem.pf();

        // Validate readings
        if (isnan(voltage) || isnan(current) || isnan(power) ||
            isnan(energy) || isnan(frequency) || isnan(pf)) {
            debug_pzem("ERROR: PZEM reading failed");
        } else {
            // Update EVCS data
            if (ENABLE_V2G) {
                evcs_data.ac_voltage = voltage;
                evcs_data.timestamp = currentTime;
            }

            bool currently_charging = (current > CURRENT_THRESHOLD);

            if (ENABLE_V2G) {
                if (currently_charging && !vehicle_connected) {
                    vehicle_connected = true;
                    start_charging_session();
                } else if (!currently_charging && vehicle_connected) {
                    vehicle_connected = false;
                    end_charging_session();
                }

                if (session_active) {
                    handle_charging_session();
                }
            }

            // Update status
            charging_status = currently_charging ? 1 : 0;

            // === SEND TO RASPBERRY PI (Fixed Format) ===
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
    }

    // Send V2G data every 5s if vehicle is connected
    if (ENABLE_V2G && vehicle_connected && (currentTime - last_v2g_send >= V2G_SEND_INTERVAL)) {
        last_v2g_send = currentTime;
        send_v2g_data();
    }

    delay(100);
}

// === ESP-NOW INITIALIZATION ===
void init_esp_now() {
    if (!ENABLE_V2G) return;

    if (esp_now_init() != 0) {
        debug_v2g("ERROR: ESP-NOW init failed");
        return;
    }

    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    esp_now_register_send_cb(on_data_sent);
    esp_now_register_recv_cb(on_data_received);

    // Start with broadcast
    memset(peer_addr, 0xFF, 6);  // Broadcast MAC

    esp_now_add_peer(
        peer_addr,
        ESP_NOW_ROLE_COMBO,
        1,
        nullptr,
        0
    );

    debug_v2g("ESP-NOW initialized");
}

// === DATA SENT CALLBACK ===
void on_data_sent(uint8_t *mac_addr, uint8_t sendStatus) {
    if (sendStatus == 0) {
        debug_v2g("[V2G] Data sent successfully");
    } else {
        debug_v2g("[V2G] Send failed: %d", sendStatus);
    }
}

// === DATA RECEIVED CALLBACK ===
void on_data_received(uint8_t *mac, uint8_t *data, uint8_t len) {
    if (!ENABLE_V2G) return;

    if (len != sizeof(vehicle_to_evcs_t)) {
        debug_v2g("[V2G] Bad data length: %d (expected %d)", len, sizeof(vehicle_to_evcs_t));
        return;
    }

    memcpy(&vehicle_data, data, sizeof(vehicle_to_evcs_t));
    memcpy(vehicle_data.vehicle_mac, mac, 6);
    vehicle_data_received = true;

    if (DEBUG_V2G) {
        debug_v2g("[V2G] Received from: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        debug_v2g("[V2G] SOC: %u%%, Voltage: %.1fV, Current: %.1fA",
                 vehicle_data.battery_soc, vehicle_data.battery_voltage, vehicle_data.battery_current);
        debug_v2g("[V2G] Desired SOC: %.1f%%", vehicle_data.desired_soc);

        if (vehicle_data.gps_valid) {
            float distance = calculate_distance_between_points(
                EVCS_LATITUDE, EVCS_LONGITUDE,
                vehicle_data.vehicle_latitude, vehicle_data.vehicle_longitude
            );
            debug_v2g("[V2G] Distance to vehicle: %.1f meters", distance);
        }
    }

    // Update peer if new vehicle
    if (memcmp(mac, peer_addr, 6) != 0) {
        debug_v2g("[V2G] New vehicle detected. Updating peer.");
        esp_now_del_peer(peer_addr);
        memcpy(peer_addr, mac, 6);
        memcpy(vehicle_mac, mac, 6);

        esp_now_add_peer(peer_addr, ESP_NOW_ROLE_COMBO, 1, nullptr, 0);
    }

    if (vehicle_data.stop_charging && session_active) {
        debug_v2g("[V2G] Vehicle requested to stop charging");
        end_charging_session();
    }
}

// === SEND V2G DATA TO VEHICLE ===
void send_v2g_data() {
    if (!ENABLE_V2G) return;
    update_evcs_data();

    int result = esp_now_send(peer_addr, (uint8_t*)&evcs_data, sizeof(evcs_data));
    if (result == 0) {
        debug_v2g("[V2G] Sent EVCS data to vehicle");
    } else {
        debug_v2g("[V2G] Send failed: %d", result);
    }
}

// === START CHARGING SESSION ===
void start_charging_session() {
    if (!ENABLE_V2G || session_active) return;

    session_active = true;
    session_start_time = millis();
    session_start_energy = pzem.energy();
    current_session_id++;
    evcs_data.session_id = current_session_id;

    debug_v2g("[V2G] Charging session started | ID: %u", current_session_id);
}

// === END CHARGING SESSION ===
void end_charging_session() {
    if (!ENABLE_V2G || !session_active) return;

    session_active = false;
    float final_energy = pzem.energy();
    float energy_delivered = final_energy - session_start_energy;
    float total_cost = energy_delivered * CHARGING_RATE_PER_KWH;
    unsigned long duration = millis() - session_start_time;

    debug_v2g("[V2G] Charging session ended");
    debug_v2g("[V2G] Energy: %.3f kWh | Cost: %.2f € | Duration: %lu min",
             energy_delivered, total_cost, duration / 60000);

    evcs_data.current_energy_delivered = 0.0f;
    evcs_data.current_cost = 0.0f;
}

// === UPDATE SESSION DATA ===
void handle_charging_session() {
    if (!ENABLE_V2G || !session_active) return;

    float current_energy = pzem.energy();
    evcs_data.current_energy_delivered = current_energy - session_start_energy;
    evcs_data.current_cost = evcs_data.current_energy_delivered * CHARGING_RATE_PER_KWH;
}

// === UPDATE EVCS DATA BEFORE SENDING ===
void update_evcs_data() {
    if (!ENABLE_V2G) return;

    evcs_data.charging_available = !session_active;
    evcs_data.timestamp = millis();
    evcs_data.evcs_latitude = EVCS_LATITUDE;
    evcs_data.evcs_longitude = EVCS_LONGITUDE;
    evcs_data.evcs_altitude = 0.0f;
}

// === CALCULATE DISTANCE BETWEEN TWO GPS POINTS (HAVERSINE) ===
float calculate_distance_between_points(float lat1, float lon1, float lat2, float lon2) {
    float lat1_rad = lat1 * M_PI / 180.0f;
    float lon1_rad = lon1 * M_PI / 180.0f;
    float lat2_rad = lat2 * M_PI / 180.0f;
    float lon2_rad = lon2 * M_PI / 180.0f;

    float dlat = lat2_rad - lat1_rad;
    float dlon = lon2_rad - lon1_rad;

    float a = sin(dlat / 2) * sin(dlat / 2) +
              cos(lat1_rad) * cos(lat2_rad) *
              sin(dlon / 2) * sin(dlon / 2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));

    return 6371000.0f * c;  // meters
}
