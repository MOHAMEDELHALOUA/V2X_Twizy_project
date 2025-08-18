//v6
// EVCS_Monitor_Fixed.c - Simplified EVCS Monitor without caching issues
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

// Global control variables
static volatile bool running = true;
static int serial_fd = -1;
static FILE *serial_stream = NULL;

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

// EVSE Data Structure
typedef struct {
    float voltage;
    float current;
    float power;
    float energy;
    int frequency;
    float power_factor;
    bool charging_status;
    bool valid;
    time_t timestamp;
} EVSEData;

// Session tracking
static bool charging_session_active = false;
static float session_start_energy = 0.0f;
static time_t session_start_time = 0;

// Global config
static EVCSConfig config = {
    .latitude = 33.986242f,
    .longitude = -6.725006f,
    .cost_per_kwh = 1.50f,
    .check_interval = 2,
    .number_of_slots = 1,
    .fast_charge = false
};

void signal_handler(int signal) {
    printf("\n[EVCS] Shutting down (signal %d)...\n", signal);
    running = false;
    
    if (serial_stream) {
        fclose(serial_stream);
        serial_stream = NULL;
    }
    if (serial_fd >= 0) {
        close(serial_fd);
        serial_fd = -1;
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
        // Skip comments and empty lines
        if (line[0] == '#' || line[0] == '/' || line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        
        // Parse key=value pairs
        if (sscanf(line, "%63[^=]=%191[^\r\n]", key, value) == 2) {
            // Trim whitespace
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
    printf("  Check interval: %d seconds\n", config.check_interval);
    
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
    
    printf("[EVCS] Serial port opened, configuring...\n");
    
    struct termios options;
    if (tcgetattr(fd, &options) != 0) {
        perror("[EVCS] tcgetattr failed");
        close(fd);
        return -1;
    }
    
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    
    options.c_cflag &= ~PARENB;      // No parity
    options.c_cflag &= ~CSTOPB;      // 1 stop bit
    options.c_cflag &= ~CSIZE;       // Clear data size
    options.c_cflag |= CS8;          // 8 data bits
    options.c_cflag |= CREAD | CLOCAL; // Enable receiver, ignore modem control
    
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
    options.c_iflag &= ~(IXON | IXOFF | IXANY);         // No flow control
    options.c_iflag &= ~(ICRNL | INLCR);                // No CR/LF conversion
    options.c_oflag &= ~OPOST;                          // Raw output
    
    options.c_cc[VMIN] = 1;   // Wait for at least 1 character
    options.c_cc[VTIME] = 5;  // Timeout after 0.5 seconds
    
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("[EVCS] tcsetattr failed");
        close(fd);
        return -1;
    }
    
    // Clear any stale data in the buffers
    tcflush(fd, TCIOFLUSH);
    
    printf("[EVCS] Serial port configured successfully\n");
    return fd;
}

bool parse_evse_line(const char *line, EVSEData *data) {
    // Clear the data structure first
    memset(data, 0, sizeof(EVSEData));
    
    if (strncmp(line, "EVSE", 4) != 0) {
        return false;
    }
    
    int status;
    float lat, lon; // Will be ignored
    
    int parsed = sscanf(line, "EVSE%d %f %f %f %f %f %f %d %f",
                       &status, &lat, &lon,
                       &data->voltage, &data->current, &data->power,
                       &data->energy, &data->frequency, &data->power_factor);
    
    if (parsed >= 8) {
        data->charging_status = (status == 1);
        data->valid = true;
        data->timestamp = time(NULL);
        
        printf("[EVCS] Parsed: V=%.1f I=%.2f P=%.1f E=%.3f Status=%s\n",
               data->voltage, data->current, data->power, data->energy,
               data->charging_status ? "CHARGING" : "IDLE");
        
        return true;
    }
    
    return false;
}

void handle_charging_session(const EVSEData *data) {
    bool vehicle_connected = (data->current > 0.1f);
    
    if (vehicle_connected && !charging_session_active) {
        charging_session_active = true;
        session_start_energy = data->energy;
        session_start_time = data->timestamp;
        printf("[EVCS] ⚡ CHARGING SESSION STARTED ⚡\n");
    }
    else if (!vehicle_connected && charging_session_active) {
        charging_session_active = false;
        float session_energy = data->energy - session_start_energy;
        float session_cost = session_energy * config.cost_per_kwh;
        printf("[EVCS] 🔌 CHARGING SESSION ENDED 🔌\n");
        printf("  Energy: %.3f kWh, Cost: %.2f\n", session_energy, session_cost);
    }
}

bool send_to_thingsboard(const EVSEData *data) {
    // Create JSON payload with CURRENT data (no caching!)
    char json_payload[2048];
    
    float session_energy = 0.0f;
    float session_cost = 0.0f;
    
    if (charging_session_active) {
        session_energy = data->energy - session_start_energy;
        session_cost = session_energy * config.cost_per_kwh;
    }
    
    snprintf(json_payload, sizeof(json_payload),
        "{"
        "\"timestamp\":%ld,"
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
        "\"fast_charge_support\":%s,"
        "\"plug_type\":\"%s\","
        "\"cost_per_kwh\":%.2f,"
        "\"total_slots\":%d,"
        "\"session_energy\":%.3f,"
        "\"session_cost\":%.2f"
        "}",
        data->timestamp,
        config.evcs_name,
        config.latitude, config.longitude,
        data->voltage,     // DIRECT from parsed data
        data->current,     // DIRECT from parsed data  
        data->power,       // DIRECT from parsed data
        data->energy,      // DIRECT from parsed data
        data->frequency, data->power_factor,
        data->charging_status ? "true" : "false",
        charging_session_active ? "false" : "true",
        config.fast_charge ? "true" : "false",
        config.plug_type,
        config.cost_per_kwh,
        config.number_of_slots,
        session_energy, session_cost
    );
    
    printf("[EVCS] Sending: V=%.1f I=%.2f P=%.1f Status=%s\n",
           data->voltage, data->current, data->power,
           data->charging_status ? "CHARGING" : "IDLE");
    
    // Log data locally regardless of connection status
    printf("[EVCS] JSON: %s\n", json_payload);
    
    if (tb_send_raw_json(json_payload) == 0) {
        printf("[EVCS] ✅ Data sent to ThingsBoard successfully\n");
        return true;
    } else {
        printf("[EVCS] ❌ Send failed: %s (data logged locally)\n", tb_get_last_error());
        return false;
    }
}

int main(int argc, char *argv[]) {
    const char *serial_port = (argc >= 2) ? argv[1] : "/dev/ttyUSB0";
    
    printf("HELECAR EVCS Monitor (Fixed Version)\n");
    printf("====================================\n");
    printf("Serial Port: %s\n\n", serial_port);
    
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    load_config_file();
    
    // Test network connectivity first
    printf("[EVCS] Testing network connectivity to %s...\n", config.broker_ip);
    
    // Initialize ThingsBoard with retries
    ThingsBoardConfig tb_config = {
        .broker_port = config.broker_port,
        .keepalive = 60,
        .qos = 1
    };
    strcpy(tb_config.broker_ip, config.broker_ip);
    strcpy(tb_config.device_token, config.device_token);
    snprintf(tb_config.client_id, sizeof(tb_config.client_id), "EVCS_%ld", time(NULL));
    
    printf("[EVCS] Attempting ThingsBoard connection...\n");
    printf("  Broker: %s:%d\n", tb_config.broker_ip, tb_config.broker_port);
    printf("  Token: %s\n", tb_config.device_token);
    printf("  Client ID: %s\n", tb_config.client_id);
    
    int retry_count = 0;
    const int max_retries = 3;
    bool connected = false;
    
    while (retry_count < max_retries && !connected) {
        printf("[EVCS] Connection attempt %d/%d...\n", retry_count + 1, max_retries);
        
        if (tb_init(&tb_config) == 0) {
            if (tb_connect() == 0) {
                connected = true;
                printf("[EVCS] ✅ Connected to ThingsBoard successfully!\n");
            } else {
                printf("[EVCS] ❌ Connect failed: %s\n", tb_get_last_error());
                tb_cleanup();
            }
        } else {
            printf("[EVCS] ❌ Init failed: %s\n", tb_get_last_error());
        }
        
        if (!connected) {
            retry_count++;
            if (retry_count < max_retries) {
                printf("[EVCS] Retrying in 5 seconds...\n");
                sleep(5);
            }
        }
    }
    
    if (!connected) {
        printf("[EVCS] ⚠️  Failed to connect to ThingsBoard after %d attempts\n", max_retries);
        printf("[EVCS] Continuing in offline mode - data will be logged locally\n");
    }
    
    // Open serial port
    printf("[EVCS] Opening serial port %s...\n", serial_port);
    serial_fd = open_serial_port(serial_port);
    if (serial_fd < 0) {
        printf("[EVCS] ❌ Failed to open serial port %s\n", serial_port);
        printf("[EVCS] Try: sudo chmod 666 %s\n", serial_port);
        printf("[EVCS] Or run with: sudo ./evcs_monitor %s\n", serial_port);
        tb_disconnect();
        tb_cleanup();
        return 1;
    }
    
    printf("[EVCS] ✅ Serial port opened successfully (fd=%d)\n", serial_fd);
    
    serial_stream = fdopen(serial_fd, "r");
    if (!serial_stream) {
        perror("[EVCS] fdopen failed");
        close(serial_fd);
        tb_disconnect();
        tb_cleanup();
        return 1;
    }
    
    printf("[EVCS] ✅ Serial stream created\n");
    printf("[EVCS] Listening on %s...\n\n", serial_port);
    
    char line[MAX_LINE_LENGTH];
    time_t last_send = 0;
    int line_count = 0;
    int valid_count = 0;
    
    printf("[EVCS] Starting main loop...\n");
    printf("[EVCS] Press Ctrl+C to stop\n\n");
    
    // Clear any initial garbage data
    sleep(1);
    tcflush(serial_fd, TCIFLUSH);
    
    // MAIN LOOP - Simple and direct
    while (running) {
        if (fgets(line, sizeof(line), serial_stream) == NULL) {
            if (feof(serial_stream)) {
                printf("[EVCS] ⚠️  Serial stream ended (EOF)\n");
                break;
            }
            if (ferror(serial_stream)) {
                printf("[EVCS] ❌ Serial stream error\n");
                clearerr(serial_stream);  // Clear error and continue
                usleep(100000); // 100ms delay
                continue;
            }
            // Timeout or no data, continue
            usleep(100000); // 100ms delay
            continue;
        }
        
        line_count++;
        
        // Remove newline and any trailing whitespace
        line[strcspn(line, "\r\n")] = 0;
        
        // Skip empty lines
        if (strlen(line) == 0) continue;
        
        // Skip obviously corrupted lines (too short)
        if (strlen(line) < 20) {
            printf("[EVCS] Skipping short line: '%s'\n", line);
            continue;
        }
        
        // Show raw data for debugging (every 20 lines)
        if (line_count % 20 == 1) {
            printf("[EVCS] Raw[%d]: %s\n", line_count, line);
        }
        
        // Parse FRESH data from ESP32
        EVSEData current_data;
        if (parse_evse_line(line, &current_data)) {
            valid_count++;
            
            // Handle session logic
            handle_charging_session(&current_data);
            
            // Send to server every N seconds (using FRESH data)
            time_t now = time(NULL);
            if (now - last_send >= config.check_interval) {
                send_to_thingsboard(&current_data);  // DIRECT data, no cache!
                last_send = now;
                printf("[EVCS] Stats: %d total lines, %d valid frames\n", line_count, valid_count);
            }
        } else {
            // Show unparseable lines occasionally for debugging
            if (line_count % 50 == 1) {
                printf("[EVCS] Unparseable[%d]: '%s'\n", line_count, line);
            }
        }
    }
    
    printf("[EVCS] Main loop ended\n");
    
    // Cleanup
    if (serial_stream) {
        fclose(serial_stream);
        serial_stream = NULL;
    }
    if (serial_fd >= 0) {
        close(serial_fd);
        serial_fd = -1;
    }
    
    tb_disconnect();
    tb_cleanup();
    
    printf("[EVCS] Shutdown complete\n");
    return 0;
}
