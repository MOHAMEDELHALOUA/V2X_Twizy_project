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
typedef struct {
    char timestamp[32];         // timestamp (e.g., "1753442419.000006725")
    int soc;                    // soc (e.g., 72)
    float current;              // current (e.g., -1.0)
    char gear[16];              // gear (e.g., "Neutral")
    int motor_active;           // motor_active (e.g., 1)
    int accelerator;            // accelerator (e.g., 0)
    int brake;                  // brake (e.g., 0)
    float cap_voltage;          // cap_voltage (e.g., 56.0)
    float motor_speed;          // motor_speed (e.g., -0.0)
    float odometer;             // odometer (e.g., 3508.8)
    int range;                  // range (e.g., 24)
    float battery_voltage;      // battery_voltage (e.g., 57.5)
    float available_energy;     // available_energy (e.g., 4.4)
    uint8_t charging_status;    // charging_status (e.g., 0x2A)
    int motor_temp;             // motor_temp (e.g., 0)
    float power_request;        // power_request (e.g., 0.0)
    
    // Optional location data
    float latitude;
    float longitude;
    bool has_location;
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
