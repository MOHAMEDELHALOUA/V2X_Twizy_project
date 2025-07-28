// MQTTDataMonitor.c - Monitor and display received MQTT data on Raspberry Pi
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <MQTTClient.h>

#define BROKER_ADDRESS "tcp://localhost:1883"
#define CLIENT_ID "DataMonitor_Pi"
#define TOPIC "v1/devices/me/telemetry"
#define QOS 1
#define TIMEOUT 10000L

volatile int running = 1;

// Signal handler for clean shutdown
void signal_handler(int signal) {
    printf("\nShutting down data monitor...\n");
    running = 0;
}

// Parse and display JSON data
void parse_and_display_json(const char *json_data) {
    printf("\n=== Received Vehicle Data ===\n");
    printf("Raw JSON: %s\n", json_data);
    
    // Simple JSON parsing (you could use a JSON library for more robust parsing)
    char *data_copy = strdup(json_data);
    char *token = strtok(data_copy, ",{}\"");
    
    while (token != NULL) {
        if (strstr(token, "timestamp")) {
            token = strtok(NULL, ",{}\":");
            if (token) printf("Timestamp: %s\n", token);
        } else if (strstr(token, "soc")) {
            token = strtok(NULL, ",{}\":");
            if (token) printf("Battery SOC: %s%%\n", token);
        } else if (strstr(token, "motor_speed")) {
            token = strtok(NULL, ",{}\":");
            if (token) printf("Motor Speed: %s km/h\n", token);
        } else if (strstr(token, "gear")) {
            token = strtok(NULL, ",{}\":");
            if (token) printf("Gear: %s\n", token);
        } else if (strstr(token, "battery_voltage")) {
            token = strtok(NULL, ",{}\":");
            if (token) printf("Battery Voltage: %s V\n", token);
        } else if (strstr(token, "latitude")) {
            token = strtok(NULL, ",{}\":");
            if (token) printf("Latitude: %s\n", token);
        } else if (strstr(token, "longitude")) {
            token = strtok(NULL, ",{}\":");
            if (token) printf("Longitude: %s\n", token);
        }
        token = strtok(NULL, ",{}\":");
    }
    
    free(data_copy);
    printf("=============================\n");
}

// Callback for received messages
int message_arrived(void *context, char *topicName, int topicLen, MQTTClient_message *message) {
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    
    printf("\n[%02d:%02d:%02d] Message received on topic: %s\n", 
           t->tm_hour, t->tm_min, t->tm_sec, topicName);
    
    // Convert message to string
    char *payload = malloc(message->payloadlen + 1);
    memcpy(payload, message->payload, message->payloadlen);
    payload[message->payloadlen] = '\0';
    
    // Parse and display the data
    parse_and_display_json(payload);
    
    // Save to log file
    FILE *log_file = fopen("received_data.log", "a");
    if (log_file) {
        fprintf(log_file, "[%04d-%02d-%02d %02d:%02d:%02d] %s\n",
                t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                t->tm_hour, t->tm_min, t->tm_sec, payload);
        fclose(log_file);
    }
    
    free(payload);
    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    
    return 1;
}

// Callback for connection lost
void connection_lost(void *context, char *cause) {
    printf("Connection lost: %s\n", cause ? cause : "Unknown reason");
}

int main(int argc, char *argv[]) {
    printf("MQTT Data Monitor for Raspberry Pi\n");
    printf("===================================\n");
    printf("Listening for vehicle data on topic: %s\n", TOPIC);
    printf("Press Ctrl+C to stop\n\n");
    
    // Setup signal handler
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    int rc;
    
    // Create MQTT client
    if ((rc = MQTTClient_create(&client, BROKER_ADDRESS, CLIENT_ID,
                               MQTTCLIENT_PERSISTENCE_NONE, NULL)) != MQTTCLIENT_SUCCESS) {
        printf("Failed to create MQTT client, return code %d\n", rc);
        return EXIT_FAILURE;
    }
    
    // Set callbacks
    if ((rc = MQTTClient_setCallbacks(client, NULL, connection_lost,
                                     message_arrived, NULL)) != MQTTCLIENT_SUCCESS) {
        printf("Failed to set callbacks, return code %d\n", rc);
        return EXIT_FAILURE;
    }
    
    // Connect to broker
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    
    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS) {
        printf("Failed to connect to MQTT broker, return code %d\n", rc);
        printf("Make sure Mosquitto broker is running on localhost:1883\n");
        return EXIT_FAILURE;
    }
    
    printf("Connected to MQTT broker successfully!\n");
    
    // Subscribe to the topic
    if ((rc = MQTTClient_subscribe(client, TOPIC, QOS)) != MQTTCLIENT_SUCCESS) {
        printf("Failed to subscribe to topic, return code %d\n", rc);
        return EXIT_FAILURE;
    }
    
    printf("Subscribed to topic: %s\n", TOPIC);
    printf("Waiting for vehicle data...\n\n");
    
    // Main loop - wait for messages
    while (running) {
        sleep(1);
    }
    
    // Cleanup
    printf("Disconnecting from MQTT broker...\n");
    MQTTClient_unsubscribe(client, TOPIC);
    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);
    
    printf("Data monitor stopped.\n");
    return EXIT_SUCCESS;
}
