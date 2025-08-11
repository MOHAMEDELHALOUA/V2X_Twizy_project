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
#include <dirent.h>

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
        // Set default values with your ThingsBoard info
        strcpy(config->broker_ip, "192.168.9.100");  // Your ThingsBoard server
        config->broker_port = 1883;
        strcpy(config->device_token, "HELECAR_TwizyData");  // Your access token
        strcpy(config->csv_file_pattern, "merged_can_data_*.csv");
        config->check_interval = 1;
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

// Create default config file with your ThingsBoard settings
void create_default_config() {
    FILE *file = fopen(CONFIG_FILE, "w");
    if (!file) return;
    
    fprintf(file, "# SendToCloud Configuration\n");
    fprintf(file, "# ThingsBoard MQTT Settings\n");
    fprintf(file, "broker_ip=192.168.9.100\n");  // Your ThingsBoard server
    fprintf(file, "broker_port=1883\n");
    fprintf(file, "device_token=HELECAR_TwizyData\n");  // Your access token
    fprintf(file, "\n");
    fprintf(file, "# File Settings\n");
    fprintf(file, "csv_pattern=merged_can_data_*.csv\n");
    fprintf(file, "check_interval=2\n");
    fprintf(file, "\n");
    fprintf(file, "# Location Settings (optional)\n");
    fprintf(file, "use_location=true\n");
    fprintf(file, "latitude=33.8547\n");
    fprintf(file, "longitude=-7.0062\n");
    
    fclose(file);
    printf("[SendToCloud] Created default config file: %s\n", CONFIG_FILE);
}

// Find the most recent CSV file
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

//// FIXED: Parse CSV line and create VehicleData
//int parse_csv_to_vehicle_data(const char *line, VehicleData *data) {
//    printf("[DEBUG] Parsing line: %s", line);
//    
//    char temp_gear[16];
//    char temp_charging_status[8];
//    
//    // Initialize all fields to zero/default values
//    memset(data, 0, sizeof(VehicleData));
//    
//    // Count commas to determine number of fields
//    int comma_count = 0;
//    for (const char *p = line; *p; p++) {
//        if (*p == ',') comma_count++;
//    }
//    
//    printf("[DEBUG] Found %d commas (expecting %d fields)\n", comma_count, comma_count + 1);
//    
//    // Use a more flexible parsing approach
//    char *line_copy = strdup(line);
//    if (!line_copy) return -1;
//    
//    char *token;
//    char *saveptr;
//    int field_num = 0;
//    
//    token = strtok_r(line_copy, ",", &saveptr);
//    
//    while (token != NULL && field_num < 18) {
//        // Trim whitespace
//        while (*token == ' ' || *token == '\t') token++;
//        char *end = token + strlen(token) - 1;
//        while (end > token && (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r')) {
//            *end = '\0';
//            end--;
//        }
//        
//        switch (field_num) {
//            case 0: strncpy(data->timestamp, token, sizeof(data->timestamp) - 1); break;
//            case 1: data->soc = atoi(token); break;
//            case 2: data->current = atof(token); break;
//            case 3: strncpy(temp_gear, token, sizeof(temp_gear) - 1); break;
//            case 4: data->motor_active = atoi(token); break;
//            case 5: data->accelerator = atoi(token); break;
//            case 6: data->brake = atoi(token); break;
//            case 7: data->cap_voltage = atof(token); break;
//            case 8: data->motor_speed = atof(token); break;
//            case 9: data->odometer = atof(token); break;
//            case 10: data->range = (int)atof(token); break;  // Convert float to int
//            case 11: data->battery_voltage = atof(token); break;
//            case 12: data->available_energy = atof(token); break;
//            case 13: /* Skip this field - seems to be motor_temp duplicate */ break;
//            case 14: data->power_request = atof(token); break;
//            case 15: strncpy(temp_charging_status, token, sizeof(temp_charging_status) - 1); break;
//            case 16: data->motor_temp = atoi(token); break;
//            case 17: /* Extra field, ignore */ break;
//        }
//        
//        token = strtok_r(NULL, ",", &saveptr);
//        field_num++;
//    }
//    
//    free(line_copy);
//    
//    if (field_num >= 16) {  // We need at least 16 fields
//        // Copy gear
//        strncpy(data->gear, temp_gear, sizeof(data->gear) - 1);
//        data->gear[sizeof(data->gear) - 1] = '\0';
//        
//        // Parse charging status (remove 0x prefix if present)
//        if (strncmp(temp_charging_status, "0x", 2) == 0) {
//            data->charging_status = (uint8_t)strtol(temp_charging_status + 2, NULL, 16);
//        } else {
//            data->charging_status = (uint8_t)strtol(temp_charging_status, NULL, 16);
//        }
//        
//        printf("[DEBUG] Successfully parsed: SOC=%d, Speed=%.1f, Gear=%s, Fields=%d\n", 
//               data->soc, data->motor_speed, data->gear, field_num);
//        return 0;  // Success
//    }
//    
//    printf("[DEBUG] Parse failed - only %d fields parsed (need at least 16)\n", field_num);
//    return -1;  // Parse error
//}

// CORRECTED: Parse CSV line and create VehicleData - REPLACE your existing function with this


int parse_csv_to_vehicle_data(const char *line, VehicleData *data) {
    printf("[DEBUG] Parsing line: %s", line);
    
    // Initialize all fields to zero/default values
    memset(data, 0, sizeof(VehicleData));
    
    // Use token-based parsing for reliability
    char *line_copy = strdup(line);
    if (!line_copy) return -1;
    
    char *token;
    char *saveptr;
    int field = 0;
    char temp_gear[32] = {0};
    char temp_charging_status[32] = {0};
    
    token = strtok_r(line_copy, ",", &saveptr);
    
    while (token != NULL && field < 22) {
        // Trim whitespace
        while (*token == ' ' || *token == '\t') token++;
        char *end = token + strlen(token) - 1;
        while (end > token && (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r')) {
            *end = '\0';
            end--;
        }
        
        switch (field) {
            case 0:  strncpy(data->timestamp, token, sizeof(data->timestamp) - 1); break;
            case 1:  data->soc = atoi(token); break;
            case 2:  data->current = atof(token); break;
            case 3:  strncpy(temp_gear, token, sizeof(temp_gear) - 1); break;
            case 4:  data->motor_active = atoi(token); break;
            case 5:  data->accelerator = atoi(token); break;
            case 6:  data->brake = atoi(token); break;
            case 7:  data->cap_voltage = atof(token); break;
            case 8:  data->motor_speed = atof(token); break;
            case 9:  data->odometer = atof(token); break;
            case 10: data->range = atoi(token); break;
            case 11: data->battery_voltage = atof(token); break;
            case 12: data->available_energy = atof(token); break;
            case 13: strncpy(temp_charging_status, token, sizeof(temp_charging_status) - 1); break;
            case 14: data->motor_temp = atoi(token); break;
            case 15: data->power_request = atof(token); break;
            case 16: data->latitude = atof(token); break;        // GPS latitude
            case 17: data->longitude = atof(token); break;       // GPS longitude  
            case 18: data->gps_altitude = atof(token); break;    // GPS altitude
            case 19: data->gps_speed = atof(token); break;       // GPS speed
            case 20: data->gps_satellites = atoi(token); break; // GPS satellites
            case 21: data->gps_valid = atoi(token); break;       // GPS valid
        }
        
        field++;
        token = strtok_r(NULL, ",", &saveptr);
    }
    
    free(line_copy);
    
    printf("[DEBUG] Successfully parsed %d fields from CSV\n", field);
    
    if (field >= 16) {  // At least basic vehicle data
        // Copy gear
        strncpy(data->gear, temp_gear, sizeof(data->gear) - 1);
        data->gear[sizeof(data->gear) - 1] = '\0';
        
        // Parse charging status
        if (strncmp(temp_charging_status, "0x", 2) == 0) {
            data->charging_status = (uint8_t)strtol(temp_charging_status + 2, NULL, 16);
        } else {
            data->charging_status = (uint8_t)strtol(temp_charging_status, NULL, 16);
        }
        
        // Check GPS data (fields 16-21)
        if (field >= 22 && data->latitude != 0.0 && data->longitude != 0.0) {
            data->has_location = true;
            printf("[DEBUG] REAL GPS DATA FOUND: Lat=%.6f, Lon=%.6f, Alt=%.1f, Speed=%.1f, Sats=%d, Valid=%d\n",
                   data->latitude, data->longitude, data->gps_altitude, 
                   data->gps_speed, data->gps_satellites, data->gps_valid);
        } else {
            data->has_location = false;
            printf("[DEBUG] No GPS data: fields=%d, lat=%.6f, lon=%.6f\n", 
                   field, data->latitude, data->longitude);
        }
        
        printf("[DEBUG] VEHICLE DATA: SOC=%d%%, Speed=%.1fkm/h, Gear=%s, GPS=%s\n", 
               data->soc, data->motor_speed, data->gear, 
               data->has_location ? "REAL" : "NONE");
        
        return 0;  // Success
    }
    
    printf("[DEBUG] Parse failed - only %d fields parsed\n", field);
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
        printf("Configuration file created with your ThingsBoard settings\n");
        printf("You can edit %s if needed and restart\n", CONFIG_FILE);
        // Don't exit, continue with default values
    }
    
    printf("Configuration:\n");
    printf("  ThingsBoard: %s:%d\n", config.broker_ip, config.broker_port);
    printf("  Device Token: %s\n", config.device_token);
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
    snprintf(tb_config.client_id, sizeof(tb_config.client_id), "HELECAR_TwizyData_%ld", time(NULL));
    
    if (tb_init(&tb_config) != 0) {
        printf("Failed to initialize ThingsBoard: %s\n", tb_get_last_error());
        return 1;
    }
    
    // Connect to ThingsBoard
    printf("Attempting to connect to ThingsBoard...\n");
    if (tb_connect() != 0) {
        printf("Failed to connect to ThingsBoard: %s\n", tb_get_last_error());
        printf("Please check:\n");
        printf("1. ThingsBoard server is running at %s:%d\n", config.broker_ip, config.broker_port);
        printf("2. Device token '%s' is correct\n", config.device_token);
        printf("3. Network connectivity\n");
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
                    // FIXED: Use REAL GPS data from CSV if available, otherwise fallback to config
                    if (!vehicle_data.has_location && config.use_location) {
                        // Only use hardcoded location if no real GPS available
                        vehicle_data.latitude = config.latitude;
                        vehicle_data.longitude = config.longitude;
                        vehicle_data.has_location = true;
                        printf("[SendToCloud] Using fallback location: %.6f, %.6f\n", 
                               config.latitude, config.longitude);
                    } else if (vehicle_data.has_location) {
                        printf("[SendToCloud] Using REAL GPS: %.6f, %.6f (Sats: %d, Valid: %s)\n", 
                               vehicle_data.latitude, vehicle_data.longitude, 
                               vehicle_data.gps_satellites, vehicle_data.gps_valid ? "YES" : "NO");
                    }
                    
                    // Send to ThingsBoard with real or fallback location
                    if (tb_send_vehicle_data(&vehicle_data) == 0) {
                        printf("[%s] Sent: SOC=%d%%, Speed=%.1fkm/h, GPS=%.6f,%.6f, Range=%dkm\n",
                               vehicle_data.timestamp, vehicle_data.soc, 
                               vehicle_data.motor_speed, vehicle_data.latitude, 
                               vehicle_data.longitude, vehicle_data.range);
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
                            if (config.use_location) {
                                tb_set_location(config.latitude, config.longitude);
                            }
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
            } else {
                // Reset location after reconnection
                if (config.use_location) {
                    tb_set_location(config.latitude, config.longitude);
                }
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
