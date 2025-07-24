#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/stat.h>
#include <time.h>
#include "parseCANFrame.h"  // Include the V3 CAN parser library

#define HEADER_BYTE_1 0xAA
#define HEADER_BYTE_2 0x55
#define HEADER_SIZE 2
#define QUEUE_SIZE 10
#define MAX_LINE_LENGTH 512
#define CAN_SNAPSHOT_FILE "can_data_snapshot.csv"
#define MAX_CAN_IDS 100  // Maximum number of unique CAN IDs to track

// Priority CAN IDs for V2V/V2G (based on your requirements)
#define PRIORITY_CAN_IDS 5
static const unsigned int priority_ids[PRIORITY_CAN_IDS] = {0x155, 0x59B, 0x599, 0x5D7, 0x425};


char parsed_filename[256];
char esp_now_filename[256];

typedef struct {
    unsigned short SOC;
    float speedKmh;
    float odometerKm;
    float displaySpeed;
    uint8_t MacAddress[6];
} Item;

// Structure to hold CAN data from CSV
typedef struct {
    char timestamp[32];
    unsigned int can_id;
    int dlc;
    char data_hex[32];
    char data_bytes[32];
    int valid;  // Flag to indicate if data is valid
} CANData;

// Structure to track unique CAN IDs and their latest data
typedef struct {
    unsigned int can_id;
    CANData latest_data;
    ParsedData parsed_data;  // V3 parsed data
    int active;  // 1 if this slot is in use, 0 if empty
    time_t last_updated;  // Track when this data was last updated
} CANIDTracker;

int serial_port;
pthread_mutex_t queue_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t queue_cond = PTHREAD_COND_INITIALIZER;
Item queue[QUEUE_SIZE];
int queue_head = 0;
int queue_tail = 0;
int queue_count = 0;

// Global array to track unique CAN IDs and their latest data
CANIDTracker can_id_tracker[MAX_CAN_IDS];
int tracked_can_ids_count = 0;
pthread_mutex_t can_tracker_mutex = PTHREAD_MUTEX_INITIALIZER;
long last_read_position = 0;  // Track file position to read only new lines

int enqueue(Item *item) {
    pthread_mutex_lock(&queue_mutex);
    if (queue_count == QUEUE_SIZE) {
        pthread_mutex_unlock(&queue_mutex);
        return -1;  // full
    }
    queue[queue_tail] = *item;
    queue_tail = (queue_tail + 1) % QUEUE_SIZE;
    queue_count++;
    pthread_cond_signal(&queue_cond);
    pthread_mutex_unlock(&queue_mutex);
    return 0;
}

int dequeue(Item *item) {
    pthread_mutex_lock(&queue_mutex);
    while (queue_count == 0) {
        pthread_cond_wait(&queue_cond, &queue_mutex);
    }
    *item = queue[queue_head];
    queue_head = (queue_head + 1) % QUEUE_SIZE;
    queue_count--;
    pthread_mutex_unlock(&queue_mutex);
    return 0;
}

void get_timestamped_filename(char *buffer, const char *prefix) {
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    strftime(buffer, 256, prefix, tm_info);
}

// Convert hex string to data array for parsing
void hex_string_to_data(const char* hex_string, CANFrame* frame) {
    char temp[3] = {0};
    int data_index = 0;
    int i = 0;
    
    // Initialize data array
    memset(frame->data, 0, 8);
    
    while (hex_string[i] != '\0' && data_index < 8) {
        if (hex_string[i] == ' ') {
            i++;
            continue;
        }
        
        temp[0] = hex_string[i];
        temp[1] = hex_string[i + 1];
        temp[2] = '\0';
        
        frame->data[data_index] = (uint8_t)strtol(temp, NULL, 16);
        data_index++;
        i += 2;
    }
}

// Parse a single CSV line into CANData structure
int parse_csv_line(const char *line, CANData *can_data) {
    if (strlen(line) == 0) {
        return -1;
    }
    
    // Parse the CSV line: timestamp,can_id,dlc,data_hex,data_bytes
    char *token;
    char *line_copy = strdup(line);
    char *saveptr;
    int field = 0;
    
    token = strtok_r(line_copy, ",", &saveptr);
    while (token != NULL && field < 5) {
        // Remove newline characters
        token[strcspn(token, "\r\n")] = 0;
        
        switch (field) {
            case 0: // timestamp
                strncpy(can_data->timestamp, token, sizeof(can_data->timestamp) - 1);
                can_data->timestamp[sizeof(can_data->timestamp) - 1] = '\0';
                break;
            case 1: // can_id
                can_data->can_id = (unsigned int)strtol(token, NULL, 16);
                break;
            case 2: // dlc
                can_data->dlc = atoi(token);
                break;
            case 3: // data_hex
                strncpy(can_data->data_hex, token, sizeof(can_data->data_hex) - 1);
                can_data->data_hex[sizeof(can_data->data_hex) - 1] = '\0';
                break;
            case 4: // data_bytes
                strncpy(can_data->data_bytes, token, sizeof(can_data->data_bytes) - 1);
                can_data->data_bytes[sizeof(can_data->data_bytes) - 1] = '\0';
                break;
        }
        field++;
        token = strtok_r(NULL, ",", &saveptr);
    }
    
    free(line_copy);
    
    // Check if we parsed all required fields
    if (field >= 5) {
        can_data->valid = 1;
        return 0;
    } else {
        can_data->valid = 0;
        return -1;
    }
}

// Find CAN ID in tracker array, return index or -1 if not found
int find_can_id_index(unsigned int can_id) {
    for (int i = 0; i < MAX_CAN_IDS; i++) {
        if (can_id_tracker[i].active && can_id_tracker[i].can_id == can_id) {
            return i;
        }
    }
    return -1;
}

// Add new CAN ID to tracker array, return index or -1 if array is full
int add_new_can_id(unsigned int can_id, const CANData *data) {
    for (int i = 0; i < MAX_CAN_IDS; i++) {
        if (!can_id_tracker[i].active) {
            can_id_tracker[i].can_id = can_id;
            can_id_tracker[i].latest_data = *data;
            can_id_tracker[i].active = 1;
            can_id_tracker[i].last_updated = time(NULL);
            tracked_can_ids_count++;
            return i;
        }
    }
    return -1; // Array is full
}

// Check if a CAN ID is in our priority list
bool is_priority_can_id(unsigned int can_id) {
    for (int i = 0; i < PRIORITY_CAN_IDS; i++) {
        if (priority_ids[i] == can_id) {
            return true;
        }
    }
    return false;
}

// NEW: Display individual frame results in your desired format
void display_individual_frame_result(unsigned int can_id, const ParsedData *parsed) {
    if (!parsed->valid) return;
    
    switch (can_id) {
        case 0x155:
            printf("Frame 0x155: SOC=%d%%, Current=%.1fA\n", 
                   parsed->SOC, parsed->batteryCurrent);
            break;
            
        case 0x59B:
            printf("Frame 0x59B: Gear=%s, Motor=%s, Accel=%d%%, Brake=%s, CapV=%.1fV\n",
                   parsed->gearPosition,
                   parsed->motorControllerActive ? "ON" : "OFF",
                   parsed->acceleratorPercent,
                   parsed->brakePressed ? "Active" : "Inactive",
                   parsed->capacitorVoltage);
            break;
            
        case 0x599:
            printf("Frame 0x599: Range=%dkm, DisplaySpeed=%.1fkm/h\n",
                   parsed->remainingRange, parsed->speedKmh);
            break;
            
        case 0x5D7:
            printf("Frame 0x5D7: Speed=%.1fkm/h, Odometer=%.1fkm\n",
                   parsed->speedKmh, parsed->odometerKm);
            break;
            
        case 0x425:
            printf("Frame 0x425: Voltage=%.1fV, Energy=%.1fkWh, ChargingStatus=0x%02X\n",
                   parsed->batteryVoltage, parsed->availableEnergy, parsed->chargingProtocolStatus);
            break;
            
        case 0x196:
            printf("Frame 0x196: MotorTemp=%d°C, PowerRequest=%.1fW\n",
                   parsed->motorTemperature, parsed->powerRequest);
            break;
            
        default:
            // For other CAN IDs, show basic info
            printf("Frame 0x%03X: [Parsed but not priority]\n", can_id);
            break;
    }
}

// NEW: Display priority frames summary
void display_priority_frames_summary() {
    pthread_mutex_lock(&can_tracker_mutex);
    
    printf("\n=== Individual Frame Results ===\n");
    
    // Display in priority order
    for (int p = 0; p < PRIORITY_CAN_IDS; p++) {
        unsigned int target_id = priority_ids[p];
        
        for (int i = 0; i < MAX_CAN_IDS; i++) {
            if (can_id_tracker[i].active && 
                can_id_tracker[i].can_id == target_id && 
                can_id_tracker[i].parsed_data.valid) {
                
                display_individual_frame_result(target_id, &can_id_tracker[i].parsed_data);
                break;
            }
        }
    }
    
    printf("==========================================\n\n");
    
    pthread_mutex_unlock(&can_tracker_mutex);
}

// Enhanced display function - NOW SAVES TO CSV INSTEAD OF PRINTING
void save_parsed_can_data_to_csv(const CANData *can_data, const ParsedData *parsed) {
    static FILE *can_csv = NULL;
    static bool csv_initialized = false;
    
    // Initialize CSV file once
    if (!csv_initialized) {
        can_csv = fopen(parsed_filename, "w");
        if (can_csv) {
            fprintf(can_csv, "timestamp,can_id,dlc,data_hex,valid,soc,current,gear,motor_active,accelerator,brake,cap_voltage,speed,odometer,range,battery_voltage,available_energy,charging_status,motor_temp,power_request\n");
            fflush(can_csv);
        }
        csv_initialized = true;
    }
    
    if (can_csv) {
        // Save all data to CSV
        fprintf(can_csv, "%s,0x%03X,%d,%s,%d,%d,%.1f,%s,%d,%d,%d,%.1f,%.1f,%.1f,%d,%.1f,%.1f,0x%02X,%d,%.1f\n",
                can_data->timestamp, can_data->can_id, can_data->dlc, can_data->data_hex,
                parsed->valid ? 1 : 0,
                parsed->SOC, parsed->batteryCurrent,
                parsed->gearPosition[0] != '\0' ? parsed->gearPosition : "Unknown",
                parsed->motorControllerActive, parsed->acceleratorPercent, 
                parsed->brakePressed ? 1 : 0, parsed->capacitorVoltage,
                parsed->speedKmh, parsed->odometerKm, parsed->remainingRange,
                parsed->batteryVoltage, parsed->availableEnergy, parsed->chargingProtocolStatus,
                parsed->motorTemperature, parsed->powerRequest);
        fflush(can_csv);
    }
}

// Update or add CAN data for a specific ID with V3 parsing - SILENT MODE
void update_can_data(const CANData *new_data) {
    pthread_mutex_lock(&can_tracker_mutex);
    
    int index = find_can_id_index(new_data->can_id);
    
    // Parse the CAN data using V3 library
    CANFrame frame;
    frame.can_id = new_data->can_id;
    frame.dlc = new_data->dlc;
    hex_string_to_data(new_data->data_hex, &frame);
    
    ParsedData parsed = parseCANFrame(&frame);
    
    if (index >= 0) {
        // Update existing CAN ID with new data
        can_id_tracker[index].latest_data = *new_data;
        can_id_tracker[index].parsed_data = parsed;
        can_id_tracker[index].last_updated = time(NULL);
        
        // Save to CSV instead of printing
        save_parsed_can_data_to_csv(new_data, &parsed);
    } else {
        // New CAN ID found
        index = add_new_can_id(new_data->can_id, new_data);
        if (index >= 0) {
            can_id_tracker[index].parsed_data = parsed;
            
            // Save to CSV instead of printing
            save_parsed_can_data_to_csv(new_data, &parsed);
        }
    }
    
    pthread_mutex_unlock(&can_tracker_mutex);
}

// Read new lines from the snapshot file starting from last read position
int read_new_can_lines() {
    FILE *file = fopen(CAN_SNAPSHOT_FILE, "r");
    if (!file) {
        return -1;
    }
    
    // Seek to the last read position
    fseek(file, last_read_position, SEEK_SET);
    
    char line[MAX_LINE_LENGTH];
    int new_lines_read = 0;
    long current_position;
    
    while (fgets(line, sizeof(line), file)) {
        current_position = ftell(file);
        
        // Skip empty lines and header
        if (strlen(line) <= 1 || strstr(line, "timestamp,can_id") != NULL) {
            last_read_position = current_position;
            continue;
        }
        
        CANData can_data;
        if (parse_csv_line(line, &can_data) == 0) {
            update_can_data(&can_data);
            new_lines_read++;
        }
        
        last_read_position = current_position;
    }
    
    fclose(file);
    return new_lines_read;
}

// Print all tracked CAN IDs and their latest parsed data
void print_all_can_data() {
    pthread_mutex_lock(&can_tracker_mutex);
    
    printf("\n=== ALL TRACKED CAN IDs WITH PARSED DATA ===\n");
    for (int i = 0; i < MAX_CAN_IDS; i++) {
        if (can_id_tracker[i].active) {
            const CANData *data = &can_id_tracker[i].latest_data;
            const ParsedData *parsed = &can_id_tracker[i].parsed_data;
            
            printf("ID: 0x%03X | Time: %s | DLC: %d | Data: %s", 
                   data->can_id, data->timestamp, data->dlc, data->data_hex);
            
            if (parsed->valid) {
                printf(" -> [Valid Parse]");
            } else {
                printf(" -> [Not Parsed]");
            }
            printf("\n");
        }
    }
    printf("=== Total: %d unique CAN IDs ===\n\n", tracked_can_ids_count);
    
    pthread_mutex_unlock(&can_tracker_mutex);
}

void print_item(const Item *item) {
    // SILENT MODE - Only save to CSV, no console output
    // Console output is removed for clean terminal
}

void* receiver_thread(void* arg) {
    uint8_t header[2];
    uint8_t buffer[sizeof(Item)];
    while (1) {
        if (read(serial_port, &header[0], 1) != 1) continue;
        if (header[0] != HEADER_BYTE_1) continue;
        if (read(serial_port, &header[1], 1) != 1) continue;
        if (header[1] != HEADER_BYTE_2) continue;
        int bytes_read = 0;
        while (bytes_read < sizeof(Item)) {
            int result = read(serial_port, buffer + bytes_read, sizeof(Item) - bytes_read);
            if (result > 0) {
                bytes_read += result;
            } else {
                perror("Read error");
                break;
            }
        }
        if (bytes_read == sizeof(Item)) {
            Item item;
            memcpy(&item, buffer, sizeof(Item));
            if (enqueue(&item) != 0) {
                fprintf(stderr, "Queue full, dropping item.\n");
            }
        }
    }
    return NULL;
}

void* printer_thread(void* arg) {
    FILE *fpt = fopen(esp_now_filename, "w+");
    if (!fpt) {
        perror("Failed to open CSV file");
        return NULL;
    }
    fprintf(fpt, "MAC,SOC,Speed,DisplaySpeed,Odometer\n");
    fflush(fpt);
    
    while (1) {
        Item item;
        dequeue(&item);
        print_item(&item);  // Console output
        
        // CSV output
        for (int i = 0; i < 6; ++i) {
            fprintf(fpt, "%02X", item.MacAddress[i]);
            if (i < 5) fprintf(fpt, ":");
        }
        fprintf(fpt, ",%u,%.1f,%.1f,%.1f\n", 
                item.SOC, item.speedKmh, item.displaySpeed, item.odometerKm);
        fflush(fpt);  // This ensures data is written immediately
    }
    fclose(fpt);
    return NULL;
}

void* sender_thread(void* arg) {
    while (1) {
        Item item = {
            .SOC = 85,
            .speedKmh = 42.0,
            .odometerKm = 1234.5,
            .displaySpeed = 41.8,
            .MacAddress = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01}
        };
        uint8_t header[2] = { HEADER_BYTE_1, HEADER_BYTE_2 };
        write(serial_port, header, 2);
        write(serial_port, &item, sizeof(Item));
        usleep(5000000); // 5 seconds
    }
    return NULL;
}

//// ENHANCED THREAD: CAN Data Reader - CLEAN TERMINAL MODE
//void* can_reader_thread(void* arg) {
//    printf("CAN Data Reader with V3 Parser - CLEAN MODE\n");
//    printf("Priority CAN IDs: 0x155, 0x59B, 0x599, 0x5D7, 0x425\n");
//    printf("All CAN data saved to: parsed_can_data.csv\n");
//    printf("Priority frames summary shown every 20 seconds\n\n");
//    
//    // Initialize the tracker array
//    memset(can_id_tracker, 0, sizeof(can_id_tracker));
//    
//    int cycle_count = 0;
//    
//    while (1) {
//        int new_lines = read_new_can_lines();
//        
//        // SILENT - no status messages about new lines
//        
//        // Every 2 cycles (4 seconds), show ONLY the priority frames summary
//        cycle_count++;
//        if (cycle_count >= 2) {
//            display_priority_frames_summary();
//            cycle_count = 0;
//        }
//        
//        // Wait 1 seconds before checking again
//        sleep(1);
//    }
//    return NULL;
//}
//
//Modified can_read_thread to avoid conflict withe the other code
//that overwrites the csv_snapshot every 5s, and to refresh data as fast as possible
//whenever the csv file is modified:

void* can_reader_thread(void* arg) {
    printf("CAN Data Reader with V3 Parser - CLEAN MODE\n");
    printf("Monitoring snapshot file every 1s for updates...\n");

    memset(can_id_tracker, 0, sizeof(can_id_tracker));

    time_t last_modified = 0;
    int cycle_count = 0;

    while (1) {
        struct stat file_stat;

        if (stat(CAN_SNAPSHOT_FILE, &file_stat) == 0) {
            if (file_stat.st_mtime != last_modified) {
                last_modified = file_stat.st_mtime;

                int new_lines = read_new_can_lines();  // Only read when modified
                // printf("New lines read: %d\n", new_lines); // Optional debug
            }
        }

        // Print priority frames every 4 cycles = ~2s now
        cycle_count++;
        if (cycle_count >= 4) {
            display_priority_frames_summary();
            cycle_count = 0;
        }

        usleep(500000);  // Check every 0.5s
    }

    return NULL;
}

// NEW THREAD: Send CAN data to ESP32 - SILENT MODE
void* can_to_esp32_sender_thread(void* arg) {
    // SILENT startup - no console message
    
    while (1) {
        pthread_mutex_lock(&can_tracker_mutex);
        
        // Find the latest V3 parsed data for the IDs we care about
        unsigned short latest_soc = 0;
        float latest_speed = 0.0;
        float latest_display_speed = 0.0;
        float latest_odometer = 0.0;
        bool found_any_data = false;
        
        for (int i = 0; i < MAX_CAN_IDS; i++) {
            if (can_id_tracker[i].active && can_id_tracker[i].parsed_data.valid) {
                const ParsedData *parsed = &can_id_tracker[i].parsed_data;
                
                switch (can_id_tracker[i].can_id) {
                    case 0x155: // SOC data
                        latest_soc = parsed->SOC;
                        found_any_data = true;
                        break;
                    case 0x5D7: // Primary speed and odometer
                        latest_speed = parsed->speedKmh;
                        latest_odometer = parsed->odometerKm;
                        found_any_data = true;
                        break;
                    case 0x599: // Display speed (backup)
                        if (latest_speed == 0.0) { // Only use if no primary speed
                            latest_display_speed = parsed->speedKmh;
                        }
                        found_any_data = true;
                        break;
                }
            }
        }
        
        pthread_mutex_unlock(&can_tracker_mutex);
        
        // Send V3 parsed data to ESP32 SILENTLY
        if (found_any_data) {
            Item can_item = {
                .SOC = latest_soc,
                .speedKmh = latest_speed,
                .odometerKm = latest_odometer,
                .displaySpeed = latest_display_speed,
                .MacAddress = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
            };
            
            // Send header + data to ESP32
            uint8_t header[2] = { HEADER_BYTE_1, HEADER_BYTE_2 };
            write(serial_port, header, 2);
            write(serial_port, &can_item, sizeof(Item));
            
            // SILENT - no console output for ESP32 sends
        }
        
        // Send every 3 seconds
        sleep(3);
    }
    return NULL;
}

int setup_serial(const char *port_path) {
    int fd = open(port_path, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("Open serial port failed");
        return -1;
    }
    struct termios tty;
    memset(&tty, 0, sizeof tty);
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr failed");
        return -1;
    }
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

int main(int argc, char *argv[]) {
    const char *esp_serial_port = (argc >= 2) ? argv[1] : "/dev/ttyUSB1";
    
    time_t now = time(NULL);
    struct tm *t = localtime(&now);

    strftime(parsed_filename, sizeof(parsed_filename), "parsed_can_data_%Y%m%d_%H%M.csv", t);
    strftime(esp_now_filename, sizeof(esp_now_filename), "ESP_now_data_%Y%m%d_%H%M.csv", t);

    printf("ESP-NOW + V2V/V2G CAN System - CLEAN TERMINAL MODE\n");
    printf("================================================\n");
    printf("ESP-NOW Serial: %s\n", esp_serial_port);
    printf("CAN Data Input: %s\n", CAN_SNAPSHOT_FILE);
    printf("ESP-NOW CSV: %s\n", esp_now_filename);
    printf("CAN Parsed CSV: %s\n", parsed_filename);
    printf("Priority Frames: 0x155, 0x59B, 0x599, 0x5D7, 0x425\n");
    printf("================================================\n\n");
    
    serial_port = setup_serial(esp_serial_port);
    if (serial_port < 0) return 1;

    pthread_t rx_tid, tx_tid, print_tid, can_reader_tid, can_sender_tid;
    
    // Start all threads
    pthread_create(&rx_tid, NULL, receiver_thread, NULL);
    pthread_create(&tx_tid, NULL, sender_thread, NULL);
    pthread_create(&print_tid, NULL, printer_thread, NULL);
    pthread_create(&can_reader_tid, NULL, can_reader_thread, NULL);
    pthread_create(&can_sender_tid, NULL, can_to_esp32_sender_thread, NULL);
    
    printf("System Active - All data logged to CSV files\n");
    printf("Priority frame summary shown every 20 seconds\n\n");
    
    // Wait for threads to complete (they run indefinitely)
    pthread_join(rx_tid, NULL);
    pthread_join(tx_tid, NULL);
    pthread_join(print_tid, NULL);
    pthread_join(can_reader_tid, NULL);
    pthread_join(can_sender_tid, NULL);
    
    close(serial_port);
    return 0;
}
