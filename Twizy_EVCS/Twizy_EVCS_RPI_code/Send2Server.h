// Send2Server.h - ThingsBoard MQTT Communication Library
#ifndef SEND2SERVER_H
#define SEND2SERVER_H

#include <stdint.h>
#include <stdbool.h>

// ThingsBoard connection configuration
typedef struct {
    char broker_ip[256];        // ThingsBoard server IP
    int broker_port;            // MQTT port (usually 1883)
    char device_token[128];     // ThingsBoard device access token
    char client_id[64];         // MQTT client ID (unique per device)
    int keepalive;              // MQTT keepalive interval in seconds
    int qos;                    // Quality of Service (0, 1, or 2)
} ThingsBoardConfig;

// Vehicle data structure for ThingsBoard (matches your exact CSV format)
// Send2Server.h - UPDATED VehicleData structure
typedef struct {
    char timestamp[32];         // timestamp
    int soc;                    // soc
    float current;              // current
    char gear[16];              // gear
    int motor_active;           // motor_active
    int accelerator;            // accelerator
    int brake;                  // brake
    float cap_voltage;          // cap_voltage
    float motor_speed;          // motor_speed
    float odometer;             // odometer
    int range;                  // range
    float battery_voltage;      // battery_voltage
    float available_energy;     // available_energy
    uint8_t charging_status;    // charging_status
    int motor_temp;             // motor_temp
    float power_request;        // power_request
    
    // GPS data from CSV - ADD THESE FIELDS:
    float latitude;             // GPS latitude (real GPS!)
    float longitude;            // GPS longitude (real GPS!)
    float gps_altitude;         // GPS altitude
    float gps_speed;            // GPS speed
    int gps_satellites;         // Number of GPS satellites
    int gps_valid;              // GPS validity flag (1=valid, 0=invalid)
    bool has_location;          // Flag indicating if location data is available
} VehicleData;
// Connection status
typedef enum {
    TB_DISCONNECTED = 0,
    TB_CONNECTING = 1,
    TB_CONNECTED = 2,
    TB_ERROR = 3
} ConnectionStatus;

// Function prototypes
/**
 * Initialize ThingsBoard connection
 * @param config ThingsBoard configuration
 * @return 0 on success, -1 on error
 */
int tb_init(const ThingsBoardConfig *config);

/**
 * Connect to ThingsBoard server
 * @return 0 on success, -1 on error
 */
int tb_connect(void);

/**
 * Disconnect from ThingsBoard server
 */
void tb_disconnect(void);

/**
 * Send vehicle data to ThingsBoard
 * @param data Vehicle data to send
 * @return 0 on success, -1 on error
 */
int tb_send_vehicle_data(const VehicleData *data);

/**
 * Send raw JSON data to ThingsBoard
 * @param json_payload JSON string to send
 * @return 0 on success, -1 on error
 */
int tb_send_raw_json(const char *json_payload);

/**
 * Get connection status
 * @return Current connection status
 */
ConnectionStatus tb_get_status(void);

/**
 * Get last error message
 * @return Error message string
 */
const char* tb_get_last_error(void);

/**
 * Set location data (optional)
 * @param lat Latitude
 * @param lon Longitude
 */
void tb_set_location(float lat, float lon);

/**
 * Cleanup and free resources
 */
void tb_cleanup(void);

#endif // SEND2SERVER_H
