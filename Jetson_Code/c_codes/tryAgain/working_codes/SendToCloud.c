// SendToCloud.c - Standalone MQTT Client for ThingsBoard
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/stat.h>
#include <signal.h>
#include <stdbool.h>
#include "Send2Server.h"
#include <dirent.h>  // Add this line for opendir/readdir functions

#define MAX_LINE_LENGTH 1024
#define CONFIG_FILE "mqtt_config.txt"

// Global variables for clean shutdown
static volatile bool running = true;
static char current_csv_file[256] = {0};

// Configuration structure
typedef struct {
    char broker_ip[256];
    int broker_port;
    char device_token[128];
    char csv_file_pattern[256];
    int check_interval;
    float latitude;
    float longitude;
    bool use_location;
} CloudConfig;

// Signal handler for clean shutdown
void signal_handler(int signal) {
    printf("\n[SendToCloud] Received signal %d, shutting down...\n", signal);
    running = false;
}

// Load configuration from file
int load_config(CloudConfig *config) {
    FILE *file = fopen(CONFIG_FILE, "r");
    if (!file) {
        printf("[SendToCloud] Config file not found, using defaults\n");
        // Set default values
        strcpy(config->broker_ip, "127.0.0.1");
        config->broker_port = 1883;
        strcpy(config->device_token, "YOUR_DEVICE_TOKEN");
        strcpy(config->csv_file_pattern, "merged_can_data_*.csv");
        config->check_interval = 4;
        config->latitude = 33.8547;   // Casablanca
        config->longitude = -7.0062;
        config->use_location = true;
        return 0;
    }
    
    char line[512];
    while (fgets(line, sizeof(line), file)) {
        // Skip comments and empty lines
        if (line[0] == '#' || line[0] == '\n') continue;
        
        char key[128], value[384];
        if (sscanf(line, "%127[^=]=%383[^\n]", key, value) == 2) {
            // Remove whitespace
            char *key_trim = key;
            while (*key_trim == ' ') key_trim++;
            char *value_trim = value;
            while (*value_trim == ' ') value_trim++;
            
            if (strcmp(key_trim, "broker_ip") == 0) {
                strcpy(config->broker_ip, value_trim);
            } else if (strcmp(key_trim, "broker_port") == 0) {
                config->broker_port = atoi(value_trim);
            } else if (strcmp(key_trim, "device_token") == 0) {
                strcpy(config->device_token, value_trim);
            } else if (strcmp(key_trim, "csv_pattern") == 0) {
                strcpy(config->csv_file_pattern, value_trim);
            } else if (strcmp(key_trim, "check_interval") == 0) {
                config->check_interval = atoi(value_trim);
            } else if (strcmp(key_trim, "latitude") == 0) {
                config->latitude = atof(value_trim);
            } else if (strcmp(key_trim, "longitude") == 0) {
                config->longitude = atof(value_trim);
            } else if (strcmp(key_trim, "use_location") == 0) {
                config->use_location = (strcmp(value_trim, "true") == 0);
            }
        }
    }
    
    fclose(file);
    return 0;
}

// Create default config file
void create_default_config() {
    FILE *file = fopen(CONFIG_FILE, "w");
    if (!file) return;
    
    fprintf(file, "# SendToCloud Configuration\n");
    fprintf(file, "# ThingsBoard MQTT Settings\n");
    fprintf(file, "broker_ip=192.168.1.100\n");
    fprintf(file, "broker_port=1883\n");
    fprintf(file, "device_token=YOUR_DEVICE_TOKEN_HERE\n");
    fprintf(file, "\n");
    fprintf(file, "# File Settings\n");
    fprintf(file, "csv_pattern=merged_can_data_*.csv\n");
    fprintf(file, "check_interval=4\n");
    fprintf(file, "\n");
    fprintf(file, "# Location Settings (optional)\n");
    fprintf(file, "use_location=true\n");
    fprintf(file, "latitude=33.8547\n");
    fprintf(file, "longitude=-7.0062\n");
    
    fclose(file);
    printf("[SendToCloud] Created default config file: %s\n", CONFIG_FILE);
}

// Find the most recent CSV file
//int find_latest_csv_file(const char *pattern, char *result) {
//    // Simple implementation - looks for merged_can_data_YYYYMMDD_HHMM.csv
//    // In a real implementation, you might use glob() or readdir()
//    
//    time_t now = time(NULL);
//    struct tm *t = localtime(&now);
//    
//    // Try current time first
//    snprintf(result, 256, "merged_can_data_%04d%02d%02d_%02d%02d.csv",
//             t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min);
//    
//    if (access(result, F_OK) == 0) {
//        return 0;  // File exists
//    }
//    
//    // Try previous minutes
//    for (int i = 1; i <= 60; i++) {
//        time_t prev_time = now - (i * 60);
//        struct tm *prev_t = localtime(&prev_time);
//        
//        snprintf(result, 256, "merged_can_data_%04d%02d%02d_%02d%02d.csv",
//                 prev_t->tm_year + 1900, prev_t->tm_mon + 1, prev_t->tm_mday, 
//                 prev_t->tm_hour, prev_t->tm_min);
//        
//        if (access(result, F_OK) == 0) {
//            return 0;  // Found file
//        }
//    }
//    
//    return -1;  // No file found
//}
//
int find_latest_csv_file(const char *pattern, char *result) {
    DIR *dir;
    struct dirent *entry;
    struct stat file_stat;
    time_t latest_time = 0;
    char latest_file[256] = {0};
    
    // Open current directory
    dir = opendir(".");
    if (!dir) {
        return -1;
    }
    
    // Look for files matching the pattern
    while ((entry = readdir(dir)) != NULL) {
        // Check if filename matches pattern "merged_can_data_*.csv"
        if (strncmp(entry->d_name, "merged_can_data_", 16) == 0 &&
            strstr(entry->d_name, ".csv") != NULL) {
            
            // Get file modification time
            if (stat(entry->d_name, &file_stat) == 0) {
                if (file_stat.st_mtime > latest_time) {
                    latest_time = file_stat.st_mtime;
                    strcpy(latest_file, entry->d_name);
                }
            }
        }
    }
    
    closedir(dir);
    
    if (strlen(latest_file) > 0) {
        strcpy(result, latest_file);
        return 0;  // Success
    }
    
    return -1;  // No matching file found
}

// Parse CSV line and create VehicleData
int parse_csv_to_vehicle_data(const char *line, VehicleData *data) {
    // CSV format: timestamp,soc,current,gear,motor_active,accelerator,brake,cap_voltage,motor_speed,odometer,range,battery_voltage,available_energy,charging_status,motor_temp,power_request
    
    char temp_gear[16];
    char temp_charging_status[8];
    
    int parsed = sscanf(line, 
        "%31[^,],%d,%f,%15[^,],%d,%d,%d,%f,%f,%f,%d,%f,%f,%7[^,],%d,%f",
        data->timestamp,
        &data->soc,
        &data->current,
        temp_gear,
        &data->motor_active,
        &data->accelerator,
        &data->brake,
        &data->cap_voltage,
        &data->motor_speed,
        &data->odometer,
        &data->range,
        &data->battery_voltage,
        &data->available_energy,
        temp_charging_status,
        &data->motor_temp,
        &data->power_request
    );
    
    if (parsed >= 15) {  // At least 15 fields successfully parsed
        strncpy(data->gear, temp_gear, sizeof(data->gear) - 1);
        data->gear[sizeof(data->gear) - 1] = '\0';
        
        // Parse charging status (remove 0x prefix if present)
        if (strncmp(temp_charging_status, "0x", 2) == 0) {
            data->charging_status = (uint8_t)strtol(temp_charging_status + 2, NULL, 16);
        } else {
            data->charging_status = (uint8_t)strtol(temp_charging_status, NULL, 16);
        }
        
        return 0;  // Success
    }
    
    return -1;  // Parse error
}

// Read latest data from CSV file
int read_latest_csv_data(const char *filename, VehicleData *data) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        return -1;
    }
    
    char line[MAX_LINE_LENGTH];
    char last_line[MAX_LINE_LENGTH] = {0};
    
    // Skip header
    if (fgets(line, sizeof(line), file)) {
        // Header read, continue to data
    }
    
    // Read all lines, keep the last one
    while (fgets(line, sizeof(line), file)) {
        if (strlen(line) > 10) {  // Valid data line
            strcpy(last_line, line);
        }
    }
    
    fclose(file);
    
    if (strlen(last_line) == 0) {
        return -1;  // No data found
    }
    
    return parse_csv_to_vehicle_data(last_line, data);
}

int main(int argc, char *argv[]) {
    printf("SendToCloud - MQTT Client for ThingsBoard\n");
    printf("==========================================\n");
    
    // Setup signal handlers for clean shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Load configuration
    CloudConfig config;
    if (load_config(&config) != 0) {
        printf("Failed to load configuration\n");
        return 1;
    }
    
    // Create default config if it doesn't exist
    if (access(CONFIG_FILE, F_OK) != 0) {
        create_default_config();
        printf("Please edit %s with your ThingsBoard settings and restart\n", CONFIG_FILE);
        return 1;
    }
    
    printf("Configuration:\n");
    printf("  ThingsBoard: %s:%d\n", config.broker_ip, config.broker_port);
    printf("  CSV Pattern: %s\n", config.csv_file_pattern);
    printf("  Check Interval: %d seconds\n", config.check_interval);
    if (config.use_location) {
        printf("  Location: %.6f, %.6f\n", config.latitude, config.longitude);
    }
    printf("==========================================\n\n");
    
    // Initialize ThingsBoard connection
    ThingsBoardConfig tb_config = {
        .broker_port = config.broker_port,
        .keepalive = 60,
        .qos = 1
    };
    strcpy(tb_config.broker_ip, config.broker_ip);
    strcpy(tb_config.device_token, config.device_token);
    strcpy(tb_config.client_id, "SendToCloud_001");
    
    if (tb_init(&tb_config) != 0) {
        printf("Failed to initialize ThingsBoard: %s\n", tb_get_last_error());
        return 1;
    }
    
    // Connect to ThingsBoard
    if (tb_connect() != 0) {
        printf("Failed to connect to ThingsBoard: %s\n", tb_get_last_error());
        tb_cleanup();
        return 1;
    }
    
    // Set location if configured
    if (config.use_location) {
        tb_set_location(config.latitude, config.longitude);
    }
    
    printf("Connected to ThingsBoard successfully!\n");
    printf("Starting CSV monitoring and data transmission...\n\n");
    
    time_t last_csv_check = 0;
    
    // Main loop
    while (running) {
        time_t current_time = time(NULL);
        
        // Check for new CSV data every N seconds
        if (current_time - last_csv_check >= config.check_interval) {
            last_csv_check = current_time;
            
            // Find latest CSV file
            char csv_filename[256];
            if (find_latest_csv_file(config.csv_file_pattern, csv_filename) == 0) {
                // Check if file changed
                if (strcmp(csv_filename, current_csv_file) != 0) {
                    strcpy(current_csv_file, csv_filename);
                    printf("[SendToCloud] Now monitoring: %s\n", csv_filename);
                }
                
                // Read latest data from CSV
                VehicleData vehicle_data = {0};
                if (read_latest_csv_data(csv_filename, &vehicle_data) == 0) {
                    // Add location data if configured
                    if (config.use_location) {
                        vehicle_data.latitude = config.latitude;
                        vehicle_data.longitude = config.longitude;
                        vehicle_data.has_location = true;
                    }
                    
                    // Send to ThingsBoard
                    if (tb_send_vehicle_data(&vehicle_data) == 0) {
                        printf("[%s] Sent: SOC=%d%%, Speed=%.1fkm/h, Gear=%s\n",
                               vehicle_data.timestamp, vehicle_data.soc, 
                               vehicle_data.motor_speed, vehicle_data.gear);
                    } else {
                        printf("[SendToCloud] Failed to send data: %s\n", tb_get_last_error());
                        
                        // Try to reconnect
                        printf("[SendToCloud] Attempting to reconnect...\n");
                        tb_disconnect();
                        sleep(2);
                        if (tb_connect() != 0) {
                            printf("[SendToCloud] Reconnection failed: %s\n", tb_get_last_error());
                        } else {
                            printf("[SendToCloud] Reconnected successfully\n");
                        }
                    }
                } else {
                    printf("[SendToCloud] Failed to read CSV data from %s\n", csv_filename);
                }
            } else {
                printf("[SendToCloud] No CSV file found matching pattern: %s\n", config.csv_file_pattern);
            }
        }
        
        // Check connection status
        if (tb_get_status() != TB_CONNECTED) {
            printf("[SendToCloud] Connection lost, attempting reconnect...\n");
            tb_disconnect();
            sleep(5);
            if (tb_connect() != 0) {
                printf("[SendToCloud] Reconnection failed: %s\n", tb_get_last_error());
            }
        }
        
        // Sleep for 1 second before next iteration
        sleep(1);
    }
    
    // Cleanup
    printf("\n[SendToCloud] Shutting down...\n");
    tb_disconnect();
    tb_cleanup();
    printf("[SendToCloud] Shutdown complete\n");
    
    return 0;
}
