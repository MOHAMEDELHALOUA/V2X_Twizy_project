//v7 - Dual Slot EVCS Monitor
// EVCS_Monitor_Dual_Slot.c - Handles data from both slot1 and slot2
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>
#include <stdbool.h>
#include <pthread.h>
#include <fcntl.h>
#include <termios.h>
#include "Send2Server.h"

#define MAX_LINE_LENGTH 1024
#define MAX_SLOTS 2

// Global control variables
static volatile bool running = true;
static int serial_fd[MAX_SLOTS] = {-1, -1};
static FILE *serial_stream[MAX_SLOTS] = {NULL, NULL};
static pthread_t slot_threads[MAX_SLOTS];

// EVCS Configuration
typedef struct {
    char broker_ip[256];
    int broker_port;
    char device_token[128];
    int check_interval;
    float latitude;
    float longitude;
    char evcs_name[50];
    bool fast_charge;
    char plug_type[60];
    float cost_per_kwh;
    int number_of_slots;
} EVCSConfig;

// EVSE Data Structure (per slot)
typedef struct {
    int slot_id;
    float voltage;
    float current;
    float power;
    float energy;
    int frequency;
    float power_factor;
    bool charging_status;
    bool valid;
    time_t timestamp;
    float session_start_energy;
    time_t session_start_time;
    bool session_active;
} EVSEData;

// Global data for each slot
static EVSEData slot_data[MAX_SLOTS];
static pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;

// Global config
static EVCSConfig config = {
    .latitude = 33.986242f,
    .longitude = -6.725006f,
    .cost_per_kwh = 1.50f,
    .check_interval = 2,
    .number_of_slots = 2,
    .fast_charge = false
};

void signal_handler(int signal) {
    printf("\n[EVCS] Shutting down (signal %d)...\n", signal);
    running = false;
    
    for (int i = 0; i < MAX_SLOTS; i++) {
        if (serial_stream[i]) {
            fclose(serial_stream[i]);
            serial_stream[i] = NULL;
        }
        if (serial_fd[i] >= 0) {
            close(serial_fd[i]);
            serial_fd[i] = -1;
        }
    }
    exit(0);
}

int load_config_file() {
    FILE *file = fopen("evcs_config.txt", "r");
    char line[256];
    char key[64], value[192];
    
    // Set defaults first
    strcpy(config.broker_ip, "demo.thingsboard.io");
    config.broker_port = 1883;
    strcpy(config.device_token, "HELECAR_EVCS_Station");
    strcpy(config.evcs_name, "HELECAR_ChargingStation");
    strcpy(config.plug_type, "Type_F_schuko_3Kwh");
    
    if (!file) {
        printf("[EVCS] Config file not found, using defaults\n");
        return 0;
    }
    
    printf("[EVCS] Loading config from evcs_config.txt...\n");
    
    while (fgets(line, sizeof(line), file)) {
        if (line[0] == '#' || line[0] == '/' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        
        if (sscanf(line, "%63[^=]=%191[^\r\n]", key, value) == 2) {
            char *key_trim = key;
            char *value_trim = value;
            while (*key_trim == ' ' || *key_trim == '\t') key_trim++;
            while (*value_trim == ' ' || *value_trim == '\t') value_trim++;
            
            if (strcmp(key_trim, "broker_ip") == 0) {
                strncpy(config.broker_ip, value_trim, sizeof(config.broker_ip) - 1);
            }
            else if (strcmp(key_trim, "broker_port") == 0) {
                config.broker_port = atoi(value_trim);
            }
            else if (strcmp(key_trim, "device_token") == 0) {
                strncpy(config.device_token, value_trim, sizeof(config.device_token) - 1);
            }
            else if (strcmp(key_trim, "check_interval") == 0) {
                config.check_interval = atoi(value_trim);
            }
            else if (strcmp(key_trim, "evcs_name") == 0) {
                strncpy(config.evcs_name, value_trim, sizeof(config.evcs_name) - 1);
            }
            else if (strcmp(key_trim, "latitude") == 0) {
                config.latitude = atof(value_trim);
            }
            else if (strcmp(key_trim, "longitude") == 0) {
                config.longitude = atof(value_trim);
            }
            else if (strcmp(key_trim, "fast_charge") == 0) {
                config.fast_charge = (strcmp(value_trim, "true") == 0);
            }
            else if (strcmp(key_trim, "plug_type") == 0) {
                strncpy(config.plug_type, value_trim, sizeof(config.plug_type) - 1);
            }
            else if (strcmp(key_trim, "cost_per_kwh") == 0) {
                config.cost_per_kwh = atof(value_trim);
            }
            else if (strcmp(key_trim, "number_of_slots") == 0) {
                config.number_of_slots = atoi(value_trim);
            }
        }
    }
    
    fclose(file);
    
    printf("[EVCS] Configuration loaded:\n");
    printf("  Broker: %s:%d\n", config.broker_ip, config.broker_port);
    printf("  Token: %s\n", config.device_token);
    printf("  Station: %s\n", config.evcs_name);
    printf("  Location: %.6f, %.6f\n", config.latitude, config.longitude);
    printf("  Cost: %.2f per kWh\n", config.cost_per_kwh);
    printf("  Slots: %d\n", config.number_of_slots);
    
    return 0;
}

int open_serial_port(const char *device) {
    printf("[EVCS] Attempting to open %s...\n", device);
    
    int fd = open(device, O_RDONLY | O_NOCTTY);
    if (fd == -1) {
        perror("[EVCS] Unable to open serial port");
        printf("[EVCS] Make sure:\n");
        printf("  1. ESP32 is connected to %s\n", device);
        printf("  2. You have permission: sudo chmod 666 %s\n", device);
        printf("  3. No other program is using the port\n");
        return -1;
    }
    
    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        perror("[EVCS] tcgetattr failed");
        close(fd);
        return -1;
    }
    
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= CREAD | CLOCAL;
    
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(ICRNL | INLCR);
    options.c_oflag &= ~OPOST;
    
    options.c_cc[VMIN] = 1;
    options.c_cc[VTIME] = 5;
    
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("[EVCS] tcsetattr failed");
        close(fd);
        return -1;
    }
    
    tcflush(fd, TCIOFLUSH);
    printf("[EVCS] Serial port %s configured successfully\n", device);
    return fd;
}

bool parse_evse_line(const char *line, EVSEData *data) {
    memset(data, 0, sizeof(EVSEData));
    
    int status;
    float lat, lon;
    int slot_num;
    
    // Parse both slot formats:
    // EELAB_EVSE_slot_1 or EELAB_EVSE_slot_2
    if (sscanf(line, "EELAB_EVSE_slot_%d%d %f %f %f %f %f %f %d %f",
               &slot_num, &status, &lat, &lon,
               &data->voltage, &data->current, &data->power,
               &data->energy, &data->frequency, &data->power_factor) >= 9) {
        
        data->slot_id = slot_num;
        data->charging_status = (status == 1);
        data->valid = true;
        data->timestamp = time(NULL);
        
        printf("[EVCS] Slot %d: V=%.1f I=%.2f P=%.1f E=%.3f Status=%s\n",
               data->slot_id, data->voltage, data->current, data->power, data->energy,
               data->charging_status ? "CHARGING" : "IDLE");
        
        return true;
    }
    
    return false;
}

void handle_charging_session(EVSEData *data) {
    bool vehicle_connected = (data->current > 0.1f);
    
    if (vehicle_connected && !data->session_active) {
        data->session_active = true;
        data->session_start_energy = data->energy;
        data->session_start_time = data->timestamp;
        printf("[EVCS] ⚡ SLOT %d CHARGING SESSION STARTED ⚡\n", data->slot_id);
    }
    else if (!vehicle_connected && data->session_active) {
        data->session_active = false;
        float session_energy = data->energy - data->session_start_energy;
        float session_cost = session_energy * config.cost_per_kwh;
        printf("[EVCS] 🔌 SLOT %d CHARGING SESSION ENDED 🔌\n", data->slot_id);
        printf("  Energy: %.3f kWh, Cost: %.2f\n", session_energy, session_cost);
    }
}

bool send_to_thingsboard() {
    char json_payload[4096];  // Larger buffer for dual slot data
    time_t current_time = time(NULL);
    
    // Lock data access
    pthread_mutex_lock(&data_mutex);
    
    // Calculate session data for each slot
    float session_energy[MAX_SLOTS] = {0.0f, 0.0f};
    float session_cost[MAX_SLOTS] = {0.0f, 0.0f};
    
    for (int i = 0; i < MAX_SLOTS; i++) {
        if (slot_data[i].session_active) {
            session_energy[i] = slot_data[i].energy - slot_data[i].session_start_energy;
            session_cost[i] = session_energy[i] * config.cost_per_kwh;
        }
    }
    
    // Create comprehensive JSON payload with both slots
    snprintf(json_payload, sizeof(json_payload),
        "{"
        "\"timestamp\":%ld,"
        "\"evcs_name\":\"%s\","
        "\"latitude\":%.6f,"
        "\"longitude\":%.6f,"
        "\"total_slots\":%d,"
        "\"cost_per_kwh\":%.2f,"
        "\"plug_type\":\"%s\","
        "\"fast_charge_support\":%s,"
        
        // Slot 1 data
        "\"voltage_slot1\":%.1f,"
        "\"current_slot1\":%.2f,"
        "\"power_slot1\":%.1f,"
        "\"energy_slot1\":%.3f,"
        "\"frequency_slot1\":%d,"
        "\"power_factor_slot1\":%.2f,"
        "\"vehicle_connected_slot1\":%s,"
        "\"session_energy_slot1\":%.3f,"
        "\"session_cost_slot1\":%.2f,"
        "\"slot1_available\":%s,"
        
        // Slot 2 data
        "\"voltage_slot2\":%.1f,"
        "\"current_slot2\":%.2f,"
        "\"power_slot2\":%.1f,"
        "\"energy_slot2\":%.3f,"
        "\"frequency_slot2\":%d,"
        "\"power_factor_slot2\":%.2f,"
        "\"vehicle_connected_slot2\":%s,"
        "\"session_energy_slot2\":%.3f,"
        "\"session_cost_slot2\":%.2f,"
        "\"slot2_available\":%s,"
        
        // Combined data
        "\"total_power\":%.1f,"
        "\"active_sessions\":%d,"
        "\"available_slots\":%d"
        "}",
        
        current_time,
        config.evcs_name,
        config.latitude, config.longitude,
        config.number_of_slots,
        config.cost_per_kwh,
        config.plug_type,
        config.fast_charge ? "true" : "false",
        
        // Slot 1
        slot_data[0].voltage, slot_data[0].current, slot_data[0].power,
        slot_data[0].energy, slot_data[0].frequency, slot_data[0].power_factor,
        slot_data[0].charging_status ? "true" : "false",
        session_energy[0], session_cost[0],
        slot_data[0].session_active ? "false" : "true",
        
        // Slot 2
        slot_data[1].voltage, slot_data[1].current, slot_data[1].power,
        slot_data[1].energy, slot_data[1].frequency, slot_data[1].power_factor,
        slot_data[1].charging_status ? "true" : "false",
        session_energy[1], session_cost[1],
        slot_data[1].session_active ? "false" : "true",
        
        // Combined
        slot_data[0].power + slot_data[1].power,
        (slot_data[0].session_active ? 1 : 0) + (slot_data[1].session_active ? 1 : 0),
        (slot_data[0].session_active ? 0 : 1) + (slot_data[1].session_active ? 0 : 1)
    );
    
    pthread_mutex_unlock(&data_mutex);
    
    printf("[EVCS] Sending combined data: Slot1 P=%.1f Slot2 P=%.1f Total=%.1f\n",
           slot_data[0].power, slot_data[1].power, 
           slot_data[0].power + slot_data[1].power);
    
    if (tb_send_raw_json(json_payload) == 0) {
        printf("[EVCS] ✅ Dual-slot data sent to ThingsBoard successfully\n");
        return true;
    } else {
        printf("[EVCS] ❌ Send failed: %s\n", tb_get_last_error());
        return false;
    }
}

void* slot_reader_thread(void* arg) {
    int slot_id = *(int*)arg;
    char line[MAX_LINE_LENGTH];
    int line_count = 0;
    int valid_count = 0;
    
    printf("[EVCS] Slot %d reader thread started\n", slot_id + 1);
    
    while (running) {
        if (serial_stream[slot_id] == NULL) {
            usleep(1000000); // 1 second
            continue;
        }
        
        if (fgets(line, sizeof(line), serial_stream[slot_id]) == NULL) {
            if (feof(serial_stream[slot_id])) {
                printf("[EVCS] Slot %d: Serial stream ended (EOF)\n", slot_id + 1);
                break;
            }
            if (ferror(serial_stream[slot_id])) {
                printf("[EVCS] Slot %d: Serial stream error\n", slot_id + 1);
                clearerr(serial_stream[slot_id]);
                usleep(100000);
                continue;
            }
            usleep(100000);
            continue;
        }
        
        line_count++;
        line[strcspn(line, "\r\n")] = 0;
        
        if (strlen(line) == 0 || strlen(line) < 20) continue;
        
        EVSEData temp_data;
        if (parse_evse_line(line, &temp_data)) {
            valid_count++;
            
            // Update global slot data with mutex protection
            pthread_mutex_lock(&data_mutex);
            
            // Copy data to the correct slot
            int target_slot = temp_data.slot_id - 1; // Convert to 0-based index
            if (target_slot >= 0 && target_slot < MAX_SLOTS) {
                // Preserve session state
                bool was_active = slot_data[target_slot].session_active;
                float prev_start_energy = slot_data[target_slot].session_start_energy;
                time_t prev_start_time = slot_data[target_slot].session_start_time;
                
                slot_data[target_slot] = temp_data;
                
                // Restore session state if it was active
                if (was_active) {
                    slot_data[target_slot].session_active = was_active;
                    slot_data[target_slot].session_start_energy = prev_start_energy;
                    slot_data[target_slot].session_start_time = prev_start_time;
                }
                
                // Handle session logic
                handle_charging_session(&slot_data[target_slot]);
            }
            
            pthread_mutex_unlock(&data_mutex);
            
            if (line_count % 20 == 1) {
                printf("[EVCS] Slot %d stats: %d total, %d valid\n", 
                       slot_id + 1, line_count, valid_count);
            }
        }
    }
    
    printf("[EVCS] Slot %d reader thread ended\n", slot_id + 1);
    return NULL;
}

int main(int argc, char *argv[]) {
    const char *serial_ports[] = {"/dev/ttyUSB0", "/dev/ttyUSB1"};
    
    // Allow custom serial ports via command line
    if (argc >= 3) {
        serial_ports[0] = argv[1];
        serial_ports[1] = argv[2];
    }
    
    printf("HELECAR DUAL-SLOT EVCS Monitor\n");
    printf("==============================\n");
    printf("Slot 1 Port: %s\n", serial_ports[0]);
    printf("Slot 2 Port: %s\n\n", serial_ports[1]);
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    load_config_file();
    
    // Initialize slot data
    for (int i = 0; i < MAX_SLOTS; i++) {
        memset(&slot_data[i], 0, sizeof(EVSEData));
        slot_data[i].slot_id = i + 1;
    }
    
    // Initialize ThingsBoard connection
    ThingsBoardConfig tb_config = {
        .broker_port = config.broker_port,
        .keepalive = 60,
        .qos = 1
    };
    strcpy(tb_config.broker_ip, config.broker_ip);
    strcpy(tb_config.device_token, config.device_token);
    snprintf(tb_config.client_id, sizeof(tb_config.client_id), "EVCS_DUAL_%ld", time(NULL));
    
    printf("[EVCS] Connecting to ThingsBoard...\n");
    bool connected = false;
    if (tb_init(&tb_config) == 0 && tb_connect() == 0) {
        connected = true;
        printf("[EVCS] ✅ Connected to ThingsBoard successfully!\n");
    } else {
        printf("[EVCS] ⚠️  ThingsBoard connection failed, running offline\n");
    }
    
    // Open serial ports for both slots
    for (int i = 0; i < MAX_SLOTS; i++) {
        printf("[EVCS] Opening slot %d serial port %s...\n", i + 1, serial_ports[i]);
        serial_fd[i] = open_serial_port(serial_ports[i]);
        
        if (serial_fd[i] >= 0) {
            serial_stream[i] = fdopen(serial_fd[i], "r");
            if (serial_stream[i]) {
                printf("[EVCS] ✅ Slot %d ready on %s\n", i + 1, serial_ports[i]);
                
                // Start reader thread for this slot
                int *slot_id = malloc(sizeof(int));
                *slot_id = i;
                if (pthread_create(&slot_threads[i], NULL, slot_reader_thread, slot_id) != 0) {
                    printf("[EVCS] ❌ Failed to create thread for slot %d\n", i + 1);
                    free(slot_id);
                }
            } else {
                printf("[EVCS] ❌ Failed to create stream for slot %d\n", i + 1);
                close(serial_fd[i]);
                serial_fd[i] = -1;
            }
        } else {
            printf("[EVCS] ⚠️  Slot %d not available on %s\n", i + 1, serial_ports[i]);
        }
    }
    
    printf("[EVCS] Starting main monitoring loop...\n");
    
    time_t last_send = 0;
    
    // Main monitoring loop
    while (running) {
        time_t now = time(NULL);
        
        if (connected && (now - last_send >= config.check_interval)) {
            send_to_thingsboard();
            last_send = now;
        }
        
        sleep(1);
    }
    
    // Cleanup
    printf("[EVCS] Cleaning up...\n");
    
    for (int i = 0; i < MAX_SLOTS; i++) {
        if (slot_threads[i]) {
            pthread_cancel(slot_threads[i]);
            pthread_join(slot_threads[i], NULL);
        }
        
        if (serial_stream[i]) {
            fclose(serial_stream[i]);
            serial_stream[i] = NULL;
        }
        if (serial_fd[i] >= 0) {
            close(serial_fd[i]);
            serial_fd[i] = -1;
        }
    }
    
    if (connected) {
        tb_disconnect();
        tb_cleanup();
    }
    
    printf("[EVCS] Shutdown complete\n");
    return 0;
}
