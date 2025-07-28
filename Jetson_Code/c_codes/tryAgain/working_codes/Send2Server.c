// Send2Server.c - ThingsBoard MQTT Communication Library Implementation
#include "Send2Server.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <MQTTClient.h>  // Eclipse Paho MQTT library

// Global variables
static MQTTClient mqtt_client = NULL;
static ThingsBoardConfig tb_config = {0};
static ConnectionStatus connection_status = TB_DISCONNECTED;
static char last_error[512] = {0};
static float device_latitude = 0.0;
static float device_longitude = 0.0;
static bool has_location_data = false;

// Internal function prototypes
static void connection_lost_callback(void *context, char *cause);
static int message_arrived_callback(void *context, char *topicName, int topicLen, MQTTClient_message *message);
static void delivery_complete_callback(void *context, MQTTClient_deliveryToken dt);
static char* create_vehicle_json(const VehicleData *data);

// Initialize ThingsBoard connection
int tb_init(const ThingsBoardConfig *config) {
    if (!config) {
        snprintf(last_error, sizeof(last_error), "Configuration is NULL");
        return -1;
    }
    
    // Copy configuration
    memcpy(&tb_config, config, sizeof(ThingsBoardConfig));
    
    // Create MQTT client URL
    char broker_url[512];
    snprintf(broker_url, sizeof(broker_url), "tcp://%s:%d", 
             tb_config.broker_ip, tb_config.broker_port);
    
    // Create MQTT client
    int result = MQTTClient_create(&mqtt_client, broker_url, tb_config.client_id,
                                   MQTTCLIENT_PERSISTENCE_NONE, NULL);
    
    if (result != MQTTCLIENT_SUCCESS) {
        snprintf(last_error, sizeof(last_error), 
                 "Failed to create MQTT client: %d", result);
        return -1;
    }
    
    // Set callbacks
    MQTTClient_setCallbacks(mqtt_client, NULL, connection_lost_callback,
                           message_arrived_callback, delivery_complete_callback);
    
    connection_status = TB_DISCONNECTED;
    printf("[Send2Server] Initialized for ThingsBoard at %s:%d\n", 
           tb_config.broker_ip, tb_config.broker_port);
    
    return 0;
}

// Connect to ThingsBoard server
int tb_connect(void) {
    if (!mqtt_client) {
        snprintf(last_error, sizeof(last_error), "MQTT client not initialized");
        return -1;
    }
    
    connection_status = TB_CONNECTING;
    
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    conn_opts.keepAliveInterval = tb_config.keepalive;
    conn_opts.cleansession = 1;
//    conn_opts.username = tb_config.device_token;  // ThingsBoard uses token as username
//    conn_opts.password = NULL;  // No password for ThingsBoard device tokens
    
    printf("[Send2Server] Connecting to ThingsBoard...\n");
    
    int result = MQTTClient_connect(mqtt_client, &conn_opts);
    
    if (result != MQTTCLIENT_SUCCESS) {
        snprintf(last_error, sizeof(last_error), 
                 "Failed to connect to ThingsBoard: %d", result);
        connection_status = TB_ERROR;
        return -1;
    }
    
    connection_status = TB_CONNECTED;
    printf("[Send2Server] Connected to ThingsBoard successfully\n");
    
    return 0;
}

// Disconnect from ThingsBoard server
void tb_disconnect(void) {
    if (mqtt_client && connection_status == TB_CONNECTED) {
        MQTTClient_disconnect(mqtt_client, 10000);  // 10 second timeout
        printf("[Send2Server] Disconnected from ThingsBoard\n");
    }
    connection_status = TB_DISCONNECTED;
}

// Send vehicle data to ThingsBoard
int tb_send_vehicle_data(const VehicleData *data) {
    if (!mqtt_client || connection_status != TB_CONNECTED) {
        snprintf(last_error, sizeof(last_error), "Not connected to ThingsBoard");
        return -1;
    }
    
    if (!data) {
        snprintf(last_error, sizeof(last_error), "Vehicle data is NULL");
        return -1;
    }
    
    // Create JSON payload
    char *json_payload = create_vehicle_json(data);
    if (!json_payload) {
        snprintf(last_error, sizeof(last_error), "Failed to create JSON payload");
        return -1;
    }
    
    // Send to ThingsBoard telemetry topic
    const char *topic = "v1/devices/me/telemetry";
    
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    pubmsg.payload = json_payload;
    pubmsg.payloadlen = strlen(json_payload);
    pubmsg.qos = tb_config.qos;
    pubmsg.retained = 0;
    
    MQTTClient_deliveryToken token;
    int result = MQTTClient_publishMessage(mqtt_client, topic, &pubmsg, &token);
    
    if (result == MQTTCLIENT_SUCCESS) {
        MQTTClient_waitForCompletion(mqtt_client, token, 5000);  // 5 second timeout
        printf("[Send2Server] Sent vehicle data to ThingsBoard\n");
    } else {
        snprintf(last_error, sizeof(last_error), 
                 "Failed to publish message: %d", result);
    }
    
    free(json_payload);
    return (result == MQTTCLIENT_SUCCESS) ? 0 : -1;
}

// Send raw JSON data to ThingsBoard
int tb_send_raw_json(const char *json_payload) {
    if (!mqtt_client || connection_status != TB_CONNECTED) {
        snprintf(last_error, sizeof(last_error), "Not connected to ThingsBoard");
        return -1;
    }
    
    if (!json_payload) {
        snprintf(last_error, sizeof(last_error), "JSON payload is NULL");
        return -1;
    }
    
    const char *topic = "v1/devices/me/telemetry";
    
    MQTTClient_message pubmsg = MQTTClient_message_initializer;
    pubmsg.payload = (void*)json_payload;
    pubmsg.payloadlen = strlen(json_payload);
    pubmsg.qos = tb_config.qos;
    pubmsg.retained = 0;
    
    MQTTClient_deliveryToken token;
    int result = MQTTClient_publishMessage(mqtt_client, topic, &pubmsg, &token);
    
    if (result == MQTTCLIENT_SUCCESS) {
        MQTTClient_waitForCompletion(mqtt_client, token, 5000);
        printf("[Send2Server] Sent raw JSON to ThingsBoard\n");
    } else {
        snprintf(last_error, sizeof(last_error), 
                 "Failed to publish raw JSON: %d", result);
    }
    
    return (result == MQTTCLIENT_SUCCESS) ? 0 : -1;
}

// Get connection status
ConnectionStatus tb_get_status(void) {
    return connection_status;
}

// Get last error message
const char* tb_get_last_error(void) {
    return last_error;
}

// Set location data
void tb_set_location(float lat, float lon) {
    device_latitude = lat;
    device_longitude = lon;
    has_location_data = true;
    printf("[Send2Server] Location set: %.6f, %.6f\n", lat, lon);
}

// Cleanup and free resources
void tb_cleanup(void) {
    if (mqtt_client) {
        tb_disconnect();
        MQTTClient_destroy(&mqtt_client);
        mqtt_client = NULL;
    }
    connection_status = TB_DISCONNECTED;
    printf("[Send2Server] Cleanup completed\n");
}

// Callback: Connection lost
static void connection_lost_callback(void *context, char *cause) {
    printf("[Send2Server] Connection lost: %s\n", cause ? cause : "Unknown reason");
    connection_status = TB_ERROR;
    snprintf(last_error, sizeof(last_error), "Connection lost: %s", 
             cause ? cause : "Unknown reason");
}

// Callback: Message arrived (not used for telemetry, but required)
static int message_arrived_callback(void *context, char *topicName, int topicLen, 
                                   MQTTClient_message *message) {
    // ThingsBoard responses (not typically used for telemetry)
    printf("[Send2Server] Message received on topic: %s\n", topicName);
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

// Callback: Delivery complete
static void delivery_complete_callback(void *context, MQTTClient_deliveryToken dt) {
    // Message successfully delivered
    // printf("[Send2Server] Message delivery confirmed (token: %d)\n", dt);
}

// Create JSON payload for vehicle data
//static char* create_vehicle_json(const VehicleData *data) {
//    char *json = malloc(2048);  // Allocate enough space for JSON
//    if (!json) return NULL;
//    
//    // Create ThingsBoard-compatible JSON payload
//    snprintf(json, 2048,
//        "{"
//        "\"timestamp\":\"%s\","
//        "\"soc\":%d,"
//        "\"current\":%.1f,"
//        "\"gear\":\"%s\","
//        "\"motor_active\":%s,"
//        "\"accelerator\":%d,"
//        "\"brake\":%s,"
//        "\"cap_voltage\":%.1f,"
//        "\"motor_speed\":%.1f,"
//        "\"odometer\":%.1f,"
//        "\"range\":%d,"
//        "\"battery_voltage\":%.1f,"
//        "\"available_energy\":%.1f,"
//        "\"charging_status\":\"%02X\","
//        "\"motor_temp\":%d,"
//        "\"power_request\":%.1f"
//        "%s%s%s"  // Optional location data
//        "}",
//        data->timestamp,
//        data->soc,
//        data->current,
//        data->gear,
//        data->motor_active ? "true" : "false",
//        data->accelerator,
//        data->brake ? "true" : "false",
//        data->cap_voltage,
//        data->motor_speed,
//        data->odometer,
//        data->range,
//        data->battery_voltage,
//        data->available_energy,
//        data->charging_status,
//        data->motor_temp,
//        data->power_request,
//        // Add location if available
//        (data->has_location || has_location_data) ? ",\"latitude\":" : "",
//        (data->has_location || has_location_data) ? 
//            (data->has_location ? 
//                (snprintf(json + strlen(json), 100, "%.6f,\"longitude\":%.6f", 
//                         data->latitude, data->longitude), "") :
//                (snprintf(json + strlen(json), 100, "%.6f,\"longitude\":%.6f", 
//                         device_latitude, device_longitude), "")) : "",
//        ""
//    );
//    
//    return json;
//}
//
static char* create_vehicle_json(const VehicleData *data) {
    char *json = malloc(4096);  // Increase buffer size
    if (!json) return NULL;
    
    // Create location string separately if needed
    char location_str[200] = "";
    if (data->has_location || has_location_data) {
        if (data->has_location) {
            snprintf(location_str, sizeof(location_str), 
                    ",\"latitude\":%.6f,\"longitude\":%.6f", 
                    data->latitude, data->longitude);
        } else {
            snprintf(location_str, sizeof(location_str), 
                    ",\"latitude\":%.6f,\"longitude\":%.6f", 
                    device_latitude, device_longitude);
        }
    }
    
    // Create main JSON payload
    snprintf(json, 4096,
        "{"
        "\"timestamp\":\"%s\","
        "\"soc\":%d,"
        "\"current\":%.1f,"
        "\"gear\":\"%s\","
        "\"motor_active\":%s,"
        "\"accelerator\":%d,"
        "\"brake\":%s,"
        "\"cap_voltage\":%.1f,"
        "\"motor_speed\":%.1f,"
        "\"odometer\":%.1f,"
        "\"range\":%d,"
        "\"battery_voltage\":%.1f,"
        "\"available_energy\":%.1f,"
        "\"charging_status\":\"%02X\","
        "\"motor_temp\":%d,"
        "\"power_request\":%.1f"
        "%s"  // Location data
        "}",
        data->timestamp,
        data->soc,
        data->current,
        data->gear,
        data->motor_active ? "true" : "false",
        data->accelerator,
        data->brake ? "true" : "false",
        data->cap_voltage,
        data->motor_speed,
        data->odometer,
        data->range,
        data->battery_voltage,
        data->available_energy,
        data->charging_status,
        data->motor_temp,
        data->power_request,
        location_str
    );
    
    return json;
}
