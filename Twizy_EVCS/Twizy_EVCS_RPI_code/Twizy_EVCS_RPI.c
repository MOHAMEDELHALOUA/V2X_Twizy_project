//this code should first send the EVCS data (location, Available slots and number, plug adapter type, Max charging power,
//Cost) to the server, and in the same time check continuasly if there is a vehicle is connected to the slot and started charing
//(the current will be > 0.0A and can reach 8 or 9 A when the vehicle started charging) then it will start counting the power delivered
//to the EV. When the car's plug is removed it will send a struct to esp32 via esp-no to informe it by the enery delivered and its cost.
//
// EVCS_Monitor.c - Electric Vehicle Charging Station Monitor
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/stat.h>
#include <signal.h>
#include <stdbool.h>
#include <pthread.h>
#include <fcntl.h>
#include <termios.h>
#include "Send2Server.h"

#define COPY_INTERVAL_SECONDS 1
#define MAX_LINE_LENGTH 1024
#define CONFIG_FILE "evcs_config.txt"
#define MAX_CSV_LINES 1001
#define QUEUE_SIZE 100

// EVCS Configuration for ThingsBoard
typedef struct {
    char broker_ip[256];
    int broker_port;
    char device_token[128];
    int check_interval;
    
    // EVCS Static Information
    float latitude;           // 33.986107 (fixed charging station location)
    float longitude;          // -6.724805 (fixed charging station location)
    char evcs_name[50];       // "HELECAR_ChargingStation"
    bool fast_charge;         // true if supports fast charging
    char plug_type[60];       // "Type2_AC" or "CCS_DC" etc.
    float cost_per_kwh;       // Cost in local currency per kWh
    int number_of_slots;      // Total charging slots
    bool slots_available;     // At least one slot available
} EVCSConfig;

// EVSE Frame from ESP32 (PZEM-004T data)
typedef struct {
    struct timespec timestamp;
    char timestamp_str[64];
    
    // PZEM-004T measurements
    float charging_voltage;   // AC voltage (V)
    float charging_current;   // AC current (A) 
    float charging_power;     // Power (W)
    float energy;            // Energy consumed (kWh)
    int frequency;           // Frequency (Hz)
    float power_factor;      // Power factor
    bool charging_status;    // true if current > 0 (vehicle connected)
    bool valid;             // Data validity flag
} EVSE_Frame;

// Combined data structure for ThingsBoard
typedef struct {
    char timestamp[32];
    
    // EVCS Static Info
    float evcs_latitude;
    float evcs_longitude; 
    char evcs_name[50];
    bool fast_charge_support;
    char plug_type[60];
    float cost_per_kwh;
    int total_slots;
    bool slots_available;
    
    // Real-time Charging Data
    float voltage;
    float current; 
    float power;
    float energy_delivered;
    int frequency;
    float power_factor;
    bool vehicle_connected;
    
    // Session tracking
    float session_energy;     // Energy for current session
    float session_cost;       // Cost for current session
    time_t session_start;     // When charging started
} EVCSData;

// Queue for thread-safe communication
typedef struct {
    EVSE_Frame evse_items[QUEUE_SIZE];
    int evse_front, evse_rear, evse_count;
    pthread_mutex_t lock;
    pthread_cond_t not_empty;
} DataQueue;

// Global variables
static volatile bool running = true;
static DataQueue queue = {0};
static EVCSConfig evcs_config = {0};
static EVSE_Frame latest_evse = {0};
static pthread_mutex_t evse_data_mutex = PTHREAD_MUTEX_INITIALIZER;
static volatile bool pause_requested = false;
static pthread_mutex_t pause_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t resume_condition = PTHREAD_COND_INITIALIZER;

// Charging session tracking
static bool charging_session_active = false;
static float session_start_energy = 0.0f;
static time_t session_start_time = 0;

// Function prototypes
void signal_handler(int signal);
int load_evcs_config(EVCSConfig *config);
void init_queue(DataQueue *q);
void enqueue_evse(DataQueue *q, EVSE_Frame item);
EVSE_Frame dequeue_evse(DataQueue *q);
int open_serial_port(const char *device);
int parse_evse_line(const char *line, EVSE_Frame *frame);
void *reader_thread(void *arg);
void *evse_processor_thread(void *arg);
void *send_to_server_thread(void *arg);
EVCSData create_evcs_data(const EVSE_Frame *evse);
void handle_charging_session(const EVSE_Frame *evse);

// Signal handler for clean shutdown
void signal_handler(int signal) {
    printf("\n[EVCS] Received signal %d, shutting down...\n", signal);
    running = false;
}

//// Load EVCS configuration
//int load_evcs_config(EVCSConfig *config) {
//    // Set default values
//    strcpy(config->broker_ip, "demo.thingsboard.io");
//    config->broker_port = 1883;
//    strcpy(config->device_token, "HELECAR_EVCS_Station");
//    config->check_interval = 2;
//    
//    // EVCS Fixed Properties
//    config->latitude = 33.986107f;
//    config->longitude = -6.724805f;
//    strcpy(config->evcs_name, "HELECAR_ChargingStation");
//    config->fast_charge = false;
//    strcpy(config->plug_type, "Type_F_schuko_3Kwh");
//    config->cost_per_kwh = 1.50f;  // Adjust to local currency
//    config->number_of_slots = 1;   // Single slot for now
//    config->slots_available = true;
//    
//    printf("[EVCS] Configuration loaded:\n");
//    printf("  Station: %s\n", config->evcs_name);
//    printf("  Location: %.6f, %.6f\n", config->latitude, config->longitude);
//    printf("  Plug Type: %s\n", config->plug_type);
//    printf("  Cost: %.2f per kWh\n", config->cost_per_kwh);
//    printf("  Slots: %d\n", config->number_of_slots);
//    
//    return 0;
//}

// Enhanced configuration loader function
int load_evcs_config(EVCSConfig *config) {
    FILE *file = fopen(CONFIG_FILE, "r");
    char line[256];
    char key[64], value[192];
    
    // Set default values first
    strcpy(config->broker_ip, "demo.thingsboard.io");
    config->broker_port = 1883;
    strcpy(config->device_token, "HELECAR_EVCS_Station");
    config->check_interval = 2;
    
    // EVCS Default Properties
    config->latitude = 33.986107f;
    config->longitude = -6.724805f;
    strcpy(config->evcs_name, "HELECAR_ChargingStation");
    config->fast_charge = false;
    strcpy(config->plug_type, "Type_F_schuko_3Kwh");
    config->cost_per_kwh = 1.50f;
    config->number_of_slots = 1;
    config->slots_available = true;
    
    // If config file doesn't exist, create it with defaults
    if (!file) {
        printf("[EVCS] Config file not found, creating default configuration...\n");
        file = fopen(CONFIG_FILE, "w");
        if (file) {
            fprintf(file, "# HELECAR EVCS Configuration File\n");
            fprintf(file, "# ThingsBoard MQTT Settings\n");
            fprintf(file, "broker_ip=%s\n", config->broker_ip);
            fprintf(file, "broker_port=%d\n", config->broker_port);
            fprintf(file, "device_token=%s\n", config->device_token);
            fprintf(file, "check_interval=%d\n", config->check_interval);
            fprintf(file, "\n# EVCS Station Information\n");
            fprintf(file, "evcs_name=%s\n", config->evcs_name);
            fprintf(file, "latitude=%.6f\n", config->latitude);
            fprintf(file, "longitude=%.6f\n", config->longitude);
            fprintf(file, "fast_charge=%s\n", config->fast_charge ? "true" : "false");
            fprintf(file, "plug_type=%s\n", config->plug_type);
            fprintf(file, "cost_per_kwh=%.2f\n", config->cost_per_kwh);
            fprintf(file, "number_of_slots=%d\n", config->number_of_slots);
            fprintf(file, "slots_available=%s\n", config->slots_available ? "true" : "false");
            fclose(file);
            printf("[EVCS] Default configuration file created: %s\n", CONFIG_FILE);
        }
        return 0; // Use defaults
    }
    
    // Read configuration from file
    printf("[EVCS] Loading configuration from: %s\n", CONFIG_FILE);
    
    while (fgets(line, sizeof(line), file)) {
        // Skip comments and empty lines
        if (line[0] == '#' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        
        // Parse key=value pairs
        if (sscanf(line, "%63[^=]=%191[^\r\n]", key, value) == 2) {
            // Remove any whitespace
            char *key_trim = key;
            char *value_trim = value;
            while (*key_trim == ' ' || *key_trim == '\t') key_trim++;
            while (*value_trim == ' ' || *value_trim == '\t') value_trim++;
            
            // Parse configuration values
            if (strcmp(key_trim, "broker_ip") == 0) {
                strncpy(config->broker_ip, value_trim, sizeof(config->broker_ip) - 1);
            }
            else if (strcmp(key_trim, "broker_port") == 0) {
                config->broker_port = atoi(value_trim);
            }
            else if (strcmp(key_trim, "device_token") == 0) {
                strncpy(config->device_token, value_trim, sizeof(config->device_token) - 1);
            }
            else if (strcmp(key_trim, "check_interval") == 0) {
                config->check_interval = atoi(value_trim);
            }
            else if (strcmp(key_trim, "evcs_name") == 0) {
                strncpy(config->evcs_name, value_trim, sizeof(config->evcs_name) - 1);
            }
            else if (strcmp(key_trim, "latitude") == 0) {
                config->latitude = atof(value_trim);
            }
            else if (strcmp(key_trim, "longitude") == 0) {
                config->longitude = atof(value_trim);
            }
            else if (strcmp(key_trim, "fast_charge") == 0) {
                config->fast_charge = (strcmp(value_trim, "true") == 0);
            }
            else if (strcmp(key_trim, "plug_type") == 0) {
                strncpy(config->plug_type, value_trim, sizeof(config->plug_type) - 1);
            }
            else if (strcmp(key_trim, "cost_per_kwh") == 0) {
                config->cost_per_kwh = atof(value_trim);
            }
            else if (strcmp(key_trim, "number_of_slots") == 0) {
                config->number_of_slots = atoi(value_trim);
            }
            else if (strcmp(key_trim, "slots_available") == 0) {
                config->slots_available = (strcmp(value_trim, "true") == 0);
            }
        }
    }
    
    fclose(file);
    
    // Validate critical settings
    if (strlen(config->broker_ip) == 0 || strlen(config->device_token) == 0) {
        printf("[EVCS] ERROR: Missing critical configuration (broker_ip or device_token)\n");
        return -1;
    }
    
    printf("[EVCS] Configuration loaded successfully:\n");
    printf("  Broker: %s:%d\n", config->broker_ip, config->broker_port);
    printf("  Device Token: %s\n", config->device_token);
    printf("  Station: %s\n", config->evcs_name);
    printf("  Location: %.6f, %.6f\n", config->latitude, config->longitude);
    printf("  Plug Type: %s\n", config->plug_type);
    printf("  Cost: %.2f per kWh\n", config->cost_per_kwh);
    printf("  Slots: %d\n", config->number_of_slots);
    printf("  Check Interval: %d seconds\n", config->check_interval);
    
    return 0;
}
// Initialize queue
void init_queue(DataQueue *q) {
    memset(q, 0, sizeof(DataQueue));
    pthread_mutex_init(&q->lock, NULL);
    pthread_cond_init(&q->not_empty, NULL);
}

// Enqueue EVSE data
void enqueue_evse(DataQueue *q, EVSE_Frame item) {
    pthread_mutex_lock(&q->lock);
    if (q->evse_count == QUEUE_SIZE) {
        q->evse_front = (q->evse_front + 1) % QUEUE_SIZE;
        q->evse_count--;
    }
    q->evse_items[q->evse_rear] = item;
    q->evse_rear = (q->evse_rear + 1) % QUEUE_SIZE;
    q->evse_count++;
    
    // Update latest data
    pthread_mutex_lock(&evse_data_mutex);
    latest_evse = item;
    pthread_mutex_unlock(&evse_data_mutex);
    
    pthread_cond_signal(&q->not_empty);
    pthread_mutex_unlock(&q->lock);
}

// Dequeue EVSE data
EVSE_Frame dequeue_evse(DataQueue *q) {
    EVSE_Frame frame;
    pthread_mutex_lock(&q->lock);
    while (q->evse_count == 0) {
        pthread_cond_wait(&q->not_empty, &q->lock);
    }
    
    frame = q->evse_items[q->evse_front];
    q->evse_front = (q->evse_front + 1) % QUEUE_SIZE;
    q->evse_count--;
    pthread_mutex_unlock(&q->lock);
    return frame;
}

// Open serial port
int open_serial_port(const char *device) {
    int fd = open(device, O_RDONLY | O_NOCTTY);
    if (fd == -1) {
        perror("Unable to open serial port");
        return -1;
    }
    
    struct termios options;
    tcgetattr(fd, &options);
    
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= CREAD | CLOCAL;
    
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;
    
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;
    
    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIFLUSH);
    
    return fd;
}

// Parse EVSE line from ESP32
int parse_evse_line(const char *line, EVSE_Frame *frame) {
    // Expected format: "EVSE1 33.986107 -6.724805 220.5 8.3 1826.0 12.45 50 0.95"
    //                   EVSE status lat lon voltage current power energy freq pf
    
    if (strncmp(line, "EVSE", 4) != 0) {
        return 0;
    }
    
    clock_gettime(CLOCK_REALTIME, &frame->timestamp);
    
    int status;
    float lat, lon; // These will be ignored (we use fixed station location)
    
    int parsed = sscanf(line, "EVSE%d %f %f %f %f %f %f %d %f",
                       &status,
                       &lat, &lon,  // GPS coordinates from ESP32 (ignored)
                       &frame->charging_voltage,
                       &frame->charging_current,
                       &frame->charging_power,
                       &frame->energy,
                       &frame->frequency,
                       &frame->power_factor);
    
    if (parsed >= 8) {
        frame->charging_status = (status == 1);
        frame->valid = true;
        
        // Create timestamp string
        snprintf(frame->timestamp_str, sizeof(frame->timestamp_str), 
                "%ld.%09ld", frame->timestamp.tv_sec, frame->timestamp.tv_nsec);
        
        return 1;
    }
    
    frame->valid = false;
    return 0;
}

// Handle charging session logic
void handle_charging_session(const EVSE_Frame *evse) {
    bool vehicle_connected = evse->charging_current > 0.5f; // Threshold for connection
    
    if (vehicle_connected && !charging_session_active) {
        // Start new charging session
        charging_session_active = true;
        session_start_energy = evse->energy;
        session_start_time = evse->timestamp.tv_sec;
        
        printf("[EVCS] ⚡ CHARGING SESSION STARTED ⚡\n");
        printf("  Start Energy: %.3f kWh\n", session_start_energy);
        printf("  Start Time: %s", ctime(&session_start_time));
        
    } else if (!vehicle_connected && charging_session_active) {
        // End charging session
        charging_session_active = false;
        
        float session_energy = evse->energy - session_start_energy;
        float session_cost = session_energy * evcs_config.cost_per_kwh;
        time_t session_duration = evse->timestamp.tv_sec - session_start_time;
        
        printf("[EVCS] 🔌 CHARGING SESSION COMPLETED 🔌\n");
        printf("  Energy Delivered: %.3f kWh\n", session_energy);
        printf("  Session Cost: %.2f\n", session_cost);
        printf("  Duration: %ld minutes\n", session_duration / 60);
        printf("  Average Power: %.1f kW\n", (session_energy * 60.0f) / (session_duration / 60.0f));
        
        // TODO: Send session data via ESP-NOW to vehicle
    }
}

// Create combined EVCS data for ThingsBoard
EVCSData create_evcs_data(const EVSE_Frame *evse) {
    EVCSData data = {0};
    
    // Copy timestamp
    strncpy(data.timestamp, evse->timestamp_str, sizeof(data.timestamp) - 1);
    
    // Static EVCS information
    data.evcs_latitude = evcs_config.latitude;
    data.evcs_longitude = evcs_config.longitude;
    strncpy(data.evcs_name, evcs_config.evcs_name, sizeof(data.evcs_name) - 1);
    data.fast_charge_support = evcs_config.fast_charge;
    strncpy(data.plug_type, evcs_config.plug_type, sizeof(data.plug_type) - 1);
    data.cost_per_kwh = evcs_config.cost_per_kwh;
    data.total_slots = evcs_config.number_of_slots;
    data.slots_available = !charging_session_active; // Available if not charging
    
    // Real-time measurements
    data.voltage = evse->charging_voltage;
    data.current = evse->charging_current;
    data.power = evse->charging_power;
    data.energy_delivered = evse->energy;
    data.frequency = evse->frequency;
    data.power_factor = evse->power_factor;
    data.vehicle_connected = evse->charging_status;
    
    // Session data
    if (charging_session_active) {
        data.session_energy = evse->energy - session_start_energy;
        data.session_cost = data.session_energy * evcs_config.cost_per_kwh;
        data.session_start = session_start_time;
    }
    
    return data;
}

// Serial reader thread
void *reader_thread(void *arg) {
    const char *port = (const char *)arg;
    int fd = open_serial_port(port);
    if (fd < 0) return NULL;
    
    printf("[EVCS] Reading PZEM data from serial port: %s\n", port);
    
    FILE *serial_stream = fdopen(fd, "r");
    if (!serial_stream) {
        perror("fdopen failed");
        close(fd);
        return NULL;
    }
    
    char line[MAX_LINE_LENGTH];
    int frame_count = 0;
    
    while (running && fgets(line, sizeof(line), serial_stream)) {
        // Remove newline
        line[strcspn(line, "\r\n")] = 0;
        
        if (strlen(line) == 0) continue;
        
        EVSE_Frame frame;
        if (parse_evse_line(line, &frame)) {
            enqueue_evse(&queue, frame);
            frame_count++;
            
            if (frame_count % 10 == 0) {
                printf("[EVCS] Processed %d EVSE frames\n", frame_count);
            }
        } else {
            printf("[EVCS] Unknown format: %s\n", line);
        }
    }
    
    fclose(serial_stream);
    return NULL;
}

// EVSE data processor thread
void *evse_processor_thread(void *arg) {
    printf("[EVCS] EVSE processor thread started\n");
    
    while (running) {
        EVSE_Frame frame = dequeue_evse(&queue);
        
        if (frame.valid) {
            // Handle charging session logic
            handle_charging_session(&frame);
            
            // Log current status
            printf("[EVCS] V:%.1fV I:%.2fA P:%.1fkW E:%.3fkWh %s\n",
                   frame.charging_voltage, frame.charging_current, 
                   frame.charging_power / 1000.0f, frame.energy,
                   frame.charging_status ? "CHARGING" : "IDLE");
        }
    }
    
    return NULL;
}

// Send data to ThingsBoard server thread  
void *send_to_server_thread(void *arg) {
    printf("[EVCS] ThingsBoard sender thread started\n");
    
    // Initialize ThingsBoard connection
    ThingsBoardConfig tb_config = {
        .broker_port = evcs_config.broker_port,
        .keepalive = 60,
        .qos = 1
    };
    strcpy(tb_config.broker_ip, evcs_config.broker_ip);
    strcpy(tb_config.device_token, evcs_config.device_token);
    snprintf(tb_config.client_id, sizeof(tb_config.client_id), "EVCS_%ld", time(NULL));
    
    if (tb_init(&tb_config) != 0) {
        printf("[EVCS] Failed to initialize ThingsBoard: %s\n", tb_get_last_error());
        return NULL;
    }
    
    if (tb_connect() != 0) {
        printf("[EVCS] Failed to connect to ThingsBoard: %s\n", tb_get_last_error());
        return NULL;
    }
    
    printf("[EVCS] Connected to ThingsBoard successfully!\n");
    
    while (running) {
        pthread_mutex_lock(&evse_data_mutex);
        EVSE_Frame evse_copy = latest_evse;
        pthread_mutex_unlock(&evse_data_mutex);
        
        if (evse_copy.valid) {
            EVCSData evcs_data = create_evcs_data(&evse_copy);
            
            // TODO: Convert EVCSData to VehicleData format or create new send function
            // For now, create a JSON string manually
            char json_payload[2048];
            snprintf(json_payload, sizeof(json_payload),
                "{"
                "\"evcs_name\":\"%s\","
                "\"latitude\":%.6f,"
                "\"longitude\":%.6f,"
                "\"voltage\":%.1f,"
                "\"current\":%.2f,"
                "\"power\":%.1f,"
                "\"energy\":%.3f,"
                "\"frequency\":%d,"
                "\"power_factor\":%.2f,"
                "\"vehicle_connected\":%s,"
                "\"slots_available\":%s,"
                "\"cost_per_kwh\":%.2f,"
                "\"session_energy\":%.3f,"
                "\"session_cost\":%.2f"
                "}",
                evcs_data.evcs_name,
                evcs_data.evcs_latitude, evcs_data.evcs_longitude,
                evcs_data.voltage, evcs_data.current, evcs_data.power,
                evcs_data.energy_delivered, evcs_data.frequency, evcs_data.power_factor,
                evcs_data.vehicle_connected ? "true" : "false",
                evcs_data.slots_available ? "true" : "false",
                evcs_data.cost_per_kwh,
                evcs_data.session_energy, evcs_data.session_cost
            );
            
            if (tb_send_raw_json(json_payload) == 0) {
                printf("[EVCS] Data sent to ThingsBoard\n");
            } else {
                printf("[EVCS] Failed to send data: %s\n", tb_get_last_error());
            }
        }
        
        sleep(evcs_config.check_interval);
    }
    
    tb_disconnect();
    tb_cleanup();
    return NULL;
}

// Main function
int main(int argc, char *argv[]) {
    const char *serial_port = (argc >= 2) ? argv[1] : "/dev/ttyUSB0";
    
    printf("HELECAR EVCS Monitor v1.0\n");
    printf("=========================\n");
    printf("Serial Port: %s\n", serial_port);
    printf("Features: PZEM-004T monitoring, ThingsBoard integration, Session tracking\n\n");
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Load configuration
    if (load_evcs_config(&evcs_config) != 0) {
        printf("Failed to load EVCS configuration\n");
        return 1;
    }
    
    // Initialize queue
    init_queue(&queue);
    
    // Create threads
    pthread_t reader_tid, processor_tid, server_tid;
    
    pthread_create(&reader_tid, NULL, reader_thread, (void *)serial_port);
    pthread_create(&processor_tid, NULL, evse_processor_thread, NULL);
    pthread_create(&server_tid, NULL, send_to_server_thread, NULL);
    
    printf("[EVCS] System started - monitoring charging station\n");
    printf("[EVCS] Waiting for vehicle connections...\n\n");
    
    // Wait for threads
    pthread_join(reader_tid, NULL);
    pthread_join(processor_tid, NULL);
    pthread_join(server_tid, NULL);
    
    printf("[EVCS] System shutdown complete\n");
    return 0;
}
