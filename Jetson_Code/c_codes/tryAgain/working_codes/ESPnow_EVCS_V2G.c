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
#include "parseCANFrame.h"

#define HEADER_BYTE_1 0xAA
#define HEADER_BYTE_2 0x55
#define HEADER_SIZE 2
#define QUEUE_SIZE 10
#define MAX_LINE_LENGTH 512
#define CAN_SNAPSHOT_FILE "can_data_snapshot.csv"
#define MAX_CAN_IDS 100
#define PRIORITY_CAN_IDS 6

// V2G Enhancement: Simple charging detection
#define CHARGING_CURRENT_THRESHOLD 0.5f  // Same as EVCS threshold

static const unsigned int priority_ids[PRIORITY_CAN_IDS] = {0x155, 0x59B, 0x19F, 0x599, 0x5D7, 0x425};

char parsed_filename[256];
char esp_now_filename[256];
char v2g_log_filename[256];

//// Enhanced Item structure with real CAN battery data
//typedef struct {
//    unsigned short SOC;        // From CAN 0x155
//    float speedKmh;           // From CAN 0x19F  
//    float odometerKm;         // From CAN 0x5D7
//    float displaySpeed;       // From GPS
//    
//    // NEW: Real battery data from CAN
//    float battery_voltage;    // From CAN 0x425 - Real battery voltage
//    float battery_current;    // From CAN 0x155 - Real battery current
//    float available_energy;   // From CAN 0x425 - Real available energy
//    uint8_t charging_status;  // From CAN 0x425 - Real charging status
//    
//    uint8_t MacAddress[6];
//} Item;
// ===== UPDATED ITEM STRUCTURE FOR V2V/V2G COMMUNICATION =====
typedef struct {
    // Battery data (V2G critical)
    unsigned short SOC;              // From CAN 0x155 - State of Charge %
    float battery_voltage;           // From CAN 0x425 - Real battery voltage (V)
    float battery_current;           // From CAN 0x155 - Real battery current (A)
    float available_energy;          // From CAN 0x425 - Real available energy (kWh)
    uint8_t charging_status;         // From CAN 0x425 - Real charging status
    
    // Vehicle dynamics (V2V critical)
    float speedKmh;                  // From CAN 0x19F - Motor speed (km/h)
    float acceleration;              // From CAN 0x59B - Accelerator percent (0-100%)
    char gear[4];                    // From CAN 0x59B - Gear position (R/N/D)
    uint8_t brake_status;            // From CAN 0x59B - Brake pressed (0/1)
    
    // Vehicle position (V2V/V2G critical)
    float gps_latitude;              // From GPS - Latitude (degrees)
    float gps_longitude;             // From GPS - Longitude (degrees) 
    float gps_altitude;              // From GPS - Altitude (meters)
    
    // V2G specific data
    bool is_charging_detected;       // Charging detection flag
    float desired_soc;               // Desired SOC for charging (%)
    bool ready_to_charge;            // Vehicle ready for charging
    
    // Communication
    uint8_t MacAddress[6];           // Sender's MAC address
    uint32_t timestamp;              // Message timestamp
} Item;

// V2G Enhancement: Simple charging session tracking
typedef struct {
    bool is_charging;
    time_t session_start;
    float start_soc;
    float start_energy;
    unsigned long session_duration;
    float energy_gained;
} ChargingSession;

// Structure to hold CAN data from CSV (with GPS)
typedef struct {
    char timestamp[32];
    unsigned int can_id;
    int dlc;
    char data_hex[32];
    char data_bytes[32];
    float gps_lat;
    float gps_lon;
    float gps_alt;
    float gps_speed;
    int gps_valid;
    int valid;
} CANData;

// Structure to track unique CAN IDs and their latest data
typedef struct {
    unsigned int can_id;
    CANData latest_data;
    ParsedData parsed_data;
    int active;
    time_t last_updated;
} CANIDTracker;

// Enhanced merged CAN data structure 
typedef struct {
    char timestamp[32];
    int soc;
    float current;
    char gear[16];
    int motor_active;
    int accelerator;
    int brake;
    float cap_voltage;
    float motor_speed;
    float odometer;
    int range;
    float battery_voltage;
    float available_energy;
    uint8_t charging_status;
    int motor_temp;
    float power_request;
    // GPS Data
    float gps_latitude;
    float gps_longitude;
    float gps_altitude;
    float gps_speed;
    int gps_satellites;
    int gps_valid;
    // V2G Enhancement: Simple charging detection
    bool is_charging_detected;  // Based on current > threshold
    bool has_data;
} MergedCANData;

// Global variables
int serial_port;
pthread_mutex_t queue_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t queue_cond = PTHREAD_COND_INITIALIZER;
Item queue[QUEUE_SIZE];
int queue_head = 0;
int queue_tail = 0;
int queue_count = 0;

CANIDTracker can_id_tracker[MAX_CAN_IDS];
int tracked_can_ids_count = 0;
pthread_mutex_t can_tracker_mutex = PTHREAD_MUTEX_INITIALIZER;

// V2G Enhancement: Global charging session tracking
ChargingSession current_session = {0};
pthread_mutex_t session_mutex = PTHREAD_MUTEX_INITIALIZER;

// Function prototypes
int enqueue(Item *item);
int dequeue(Item *item);
void hex_string_to_data(const char* hex_string, CANFrame* frame);
int parse_csv_line(const char *line, CANData *can_data);
int find_can_id_index(unsigned int can_id);
int add_new_can_id(unsigned int can_id, const CANData *data);
bool is_priority_can_id(unsigned int can_id);
void update_can_data(const CANData *new_data);
int read_entire_can_file();
MergedCANData create_merged_can_data();
void save_merged_can_data_to_csv(const MergedCANData *merged);

// V2G Enhancement: Simple function prototypes
void detect_charging_session(const MergedCANData *merged);
void log_charging_session(const ChargingSession *session, const MergedCANData *merged);
void print_charging_status(const MergedCANData *merged, const ChargingSession *session);

// V2G Enhancement: Simple charging detection based on current change
void detect_charging_session(const MergedCANData *merged) {
    pthread_mutex_lock(&session_mutex);
    
    bool currently_charging = merged->is_charging_detected;
    bool was_charging = current_session.is_charging;
    
    if (currently_charging && !was_charging) {
        // Charging session started - vehicle plugged in and current detected
        current_session.is_charging = true;
        current_session.session_start = time(NULL);
        current_session.start_soc = merged->soc;
        current_session.start_energy = merged->available_energy;
        
        printf("\n🔌 CHARGING SESSION STARTED 🔌\n");
        printf("Detected by current change: %.1fA\n", merged->current);
        printf("Session Start: SOC=%d%%, Energy=%.1fkWh\n", 
               merged->soc, merged->available_energy);
        printf("Voltage: %.1fV, Current: %.1fA\n", merged->battery_voltage, merged->current);
        printf("=====================================\n\n");
        
    } else if (!currently_charging && was_charging) {
        // Charging session ended - vehicle unplugged
        current_session.is_charging = false;
        current_session.session_duration = time(NULL) - current_session.session_start;
        current_session.energy_gained = merged->available_energy - current_session.start_energy;
        
        printf("\n🔋 CHARGING SESSION COMPLETED 🔋\n");
        printf("Duration: %lu minutes\n", current_session.session_duration / 60);
        printf("SOC: %d%% → %d%% (+%d%%)\n", 
               (int)current_session.start_soc, merged->soc, 
               merged->soc - (int)current_session.start_soc);
        printf("Energy: %.1f → %.1fkWh (+%.1fkWh)\n",
               current_session.start_energy, merged->available_energy,
               current_session.energy_gained);
        printf("=====================================\n\n");
        
        // Log completed session
        log_charging_session(&current_session, merged);
    }
    
    // Update session duration if currently charging
    if (current_session.is_charging) {
        current_session.session_duration = time(NULL) - current_session.session_start;
    }
    
    pthread_mutex_unlock(&session_mutex);
}

// V2G Enhancement: Log charging session to file
void log_charging_session(const ChargingSession *session, const MergedCANData *merged) {
    static FILE *v2g_log = NULL;
    static bool log_initialized = false;
    
    if (!log_initialized) {
        v2g_log = fopen(v2g_log_filename, "w");
        if (v2g_log) {
            fprintf(v2g_log, "timestamp,duration_min,start_soc,end_soc,soc_gained,start_energy,end_energy,energy_gained,max_current\n");
            fflush(v2g_log);
        }
        log_initialized = true;
    }
    
    if (v2g_log) {
        fprintf(v2g_log, "%s,%lu,%d,%d,%d,%.1f,%.1f,%.1f,%.1f\n",
                merged->timestamp,
                session->session_duration / 60,  // Duration in minutes
                (int)session->start_soc, merged->soc, 
                merged->soc - (int)session->start_soc,
                session->start_energy, merged->available_energy,
                session->energy_gained,
                merged->current);  // Current current as max current
        fflush(v2g_log);
    }
}

// V2G Enhancement: Print charging status in console
void print_charging_status(const MergedCANData *merged, const ChargingSession *session) {
    if (session->is_charging) {
        printf("[CHARGING] Duration: %lum | SOC: %d%% | Energy: %.1fkWh | Current: %.1fA\n",
               session->session_duration / 60, merged->soc, merged->available_energy,
               merged->current);
    }
}

// Queue functions
int enqueue(Item *item) {
    pthread_mutex_lock(&queue_mutex);
    if (queue_count == QUEUE_SIZE) {
        pthread_mutex_unlock(&queue_mutex);
        return -1;
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

void hex_string_to_data(const char* hex_string, CANFrame* frame) {
    char temp[3] = {0};
    int data_index = 0;
    int i = 0;
    
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

// Enhanced CSV parser with GPS data
int parse_csv_line(const char *line, CANData *can_data) {
    if (strlen(line) == 0) {
        return -1;
    }
    
    char *token;
    char *line_copy = strdup(line);
    char *saveptr;
    int field = 0;
    
    token = strtok_r(line_copy, ",", &saveptr);
    while (token != NULL && field < 10) {
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
            case 5: // gps_lat
                can_data->gps_lat = atof(token);
                break;
            case 6: // gps_lon
                can_data->gps_lon = atof(token);
                break;
            case 7: // gps_alt
                can_data->gps_alt = atof(token);
                break;
            case 8: // gps_speed
                can_data->gps_speed = atof(token);
                break;
            case 9: // gps_valid
                can_data->gps_valid = atoi(token);
                break;
        }
        field++;
        token = strtok_r(NULL, ",", &saveptr);
    }
    
    free(line_copy);
    
    if (field >= 5) {
        can_data->valid = 1;
        return 0;
    } else {
        can_data->valid = 0;
        return -1;
    }
}

int find_can_id_index(unsigned int can_id) {
    for (int i = 0; i < MAX_CAN_IDS; i++) {
        if (can_id_tracker[i].active && can_id_tracker[i].can_id == can_id) {
            return i;
        }
    }
    return -1;
}

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
    return -1;
}

bool is_priority_can_id(unsigned int can_id) {
    for (int i = 0; i < PRIORITY_CAN_IDS; i++) {
        if (priority_ids[i] == can_id) {
            return true;
        }
    }
    return false;
}

void update_can_data(const CANData *new_data) {
    pthread_mutex_lock(&can_tracker_mutex);
    
    int index = find_can_id_index(new_data->can_id);
    
    CANFrame frame;
    frame.can_id = new_data->can_id;
    frame.dlc = new_data->dlc;
    hex_string_to_data(new_data->data_hex, &frame);
    
    ParsedData parsed = parseCANFrame(&frame);
    
    if (index >= 0) {
        can_id_tracker[index].latest_data = *new_data;
        can_id_tracker[index].parsed_data = parsed;
        can_id_tracker[index].last_updated = time(NULL);
    } else {
        index = add_new_can_id(new_data->can_id, new_data);
        if (index >= 0) {
            can_id_tracker[index].parsed_data = parsed;
        }
    }
    
    pthread_mutex_unlock(&can_tracker_mutex);
}

int read_entire_can_file() {
    FILE *file = fopen(CAN_SNAPSHOT_FILE, "r");
    if (!file) {
        return -1;
    }
    
    char line[MAX_LINE_LENGTH];
    int lines_processed = 0;
    
    pthread_mutex_lock(&can_tracker_mutex);
    memset(can_id_tracker, 0, sizeof(can_id_tracker));
    tracked_can_ids_count = 0;
    pthread_mutex_unlock(&can_tracker_mutex);
    
    while (fgets(line, sizeof(line), file)) {
        if (strlen(line) <= 1 || strstr(line, "timestamp,can_id") != NULL) {
            continue;
        }
        
        CANData can_data;
        if (parse_csv_line(line, &can_data) == 0) {
            if (is_priority_can_id(can_data.can_id)) {
                update_can_data(&can_data);
                lines_processed++;
            }
        }
    }
    
    fclose(file);
    return lines_processed;
}

// Enhanced create_merged_can_data with simple charging detection
MergedCANData create_merged_can_data() {
    MergedCANData merged = {0};
    merged.has_data = false;
    
    pthread_mutex_lock(&can_tracker_mutex);
    
    time_t latest_time = 0;
    CANData *latest_gps_source = NULL;
    
    // Collect data from all priority CAN IDs
    for (int i = 0; i < MAX_CAN_IDS; i++) {
        if (!can_id_tracker[i].active || !can_id_tracker[i].parsed_data.valid) {
            continue;
        }
        
        const ParsedData *parsed = &can_id_tracker[i].parsed_data;
        const CANData *data = &can_id_tracker[i].latest_data;
        
        if (can_id_tracker[i].last_updated > latest_time) {
            latest_time = can_id_tracker[i].last_updated;
            strncpy(merged.timestamp, data->timestamp, sizeof(merged.timestamp) - 1);
            latest_gps_source = &can_id_tracker[i].latest_data;
        }
        
        switch (can_id_tracker[i].can_id) {
            case 0x155: // Battery data
                merged.soc = parsed->SOC;
                merged.current = parsed->batteryCurrent;
                merged.has_data = true;
                break;
                
            case 0x59B: // Vehicle control data
                strncpy(merged.gear, parsed->gearPosition, sizeof(merged.gear) - 1);
                merged.motor_active = parsed->motorControllerActive;
                merged.accelerator = parsed->acceleratorPercent;
                merged.brake = parsed->brakePressed ? 1 : 0;
                merged.cap_voltage = parsed->capacitorVoltage;
                merged.has_data = true;
                break;
                
            case 0x19F: // MOTOR speed
                merged.motor_speed = parsed->speedKmh - 1.7;
                merged.has_data = true;
                break;
                
            case 0x599: // Range        
                merged.range = parsed->remainingRange;
                merged.has_data = true;
                break;
                
            case 0x5D7: // odometer
                merged.odometer = parsed->odometerKm;
                merged.has_data = true;
                break;
                
            case 0x425: // Battery voltage and energy
                merged.battery_voltage = parsed->batteryVoltage;
                merged.available_energy = parsed->availableEnergy;
                merged.charging_status = parsed->chargingProtocolStatus;
                merged.has_data = true;
                break;
                
            case 0x196: // Motor temperature and power
                merged.motor_temp = parsed->motorTemperature;
                merged.power_request = parsed->powerRequest;
                merged.has_data = true;
                break;
        }
    }
    
    // Add GPS data
    if (latest_gps_source) {
        merged.gps_latitude = latest_gps_source->gps_lat;
        merged.gps_longitude = latest_gps_source->gps_lon;
        merged.gps_altitude = latest_gps_source->gps_alt;
        merged.gps_speed = latest_gps_source->gps_speed;
        merged.gps_satellites = 0;  // Not available in CSV
        merged.gps_valid = latest_gps_source->gps_valid;
    }
    
    // V2G Enhancement: Simple charging detection - current > threshold
    merged.is_charging_detected = (merged.current > CHARGING_CURRENT_THRESHOLD);
    
    pthread_mutex_unlock(&can_tracker_mutex);
    
    return merged;
}

// Enhanced save function with charging detection
void save_merged_can_data_to_csv(const MergedCANData *merged) {
    static FILE *can_csv = NULL;
    static bool csv_initialized = false;
    
    if (!csv_initialized) {
        can_csv = fopen(parsed_filename, "w");
        if (can_csv) {
            fprintf(can_csv, "timestamp,soc,current,gear,motor_active,accelerator,brake,cap_voltage,motor_speed,odometer,range,battery_voltage,available_energy,charging_status,motor_temp,power_request,gps_lat,gps_lon,gps_alt,gps_speed,gps_sats,gps_valid,is_charging\n");
            fflush(can_csv);
        }
        csv_initialized = true;
    }
    
    if (can_csv && merged->has_data) {
        fprintf(can_csv, "%s,%d,%.1f,%s,%d,%d,%d,%.1f,%.1f,%.1f,%d,%.1f,%.1f,0x%02X,%d,%.1f,%.6f,%.6f,%.1f,%.1f,%d,%d,%s\n",
                merged->timestamp,
                merged->soc, merged->current,
                merged->gear[0] != '\0' ? merged->gear : "Unknown",
                merged->motor_active, merged->accelerator, merged->brake, 
                merged->cap_voltage, merged->motor_speed, 
                merged->odometer, merged->range,
                merged->battery_voltage, merged->available_energy, merged->charging_status,
                merged->motor_temp, merged->power_request,
                merged->gps_latitude, merged->gps_longitude, merged->gps_altitude,
                merged->gps_speed, merged->gps_satellites, merged->gps_valid,
                // V2G Enhancement: Add charging detection column
                merged->is_charging_detected ? "YES" : "NO");
        fflush(can_csv);
    }
}

// Thread functions
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

void print_item(const Item *item) {
    // Silent mode - only CSV output
}

//void* printer_thread(void* arg) {
//    FILE *fpt = fopen(esp_now_filename, "w+");
//    if (!fpt) {
//        perror("Failed to open ESP-NOW CSV file");
//        return NULL;
//    }
//    fprintf(fpt, "MAC,SOC,Speed,DisplaySpeed,Odometer,BatteryVoltage,BatteryCurrent,AvailableEnergy,ChargingStatus\n");
//    fflush(fpt);
//    
//    while (1) {
//        Item item;
//        dequeue(&item);
//        print_item(&item);
//        
//        // Enhanced CSV output with real battery data
//        for (int i = 0; i < 6; ++i) {
//            fprintf(fpt, "%02X", item.MacAddress[i]);
//            if (i < 5) fprintf(fpt, ":");
//        }
//        fprintf(fpt, ",%u,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,0x%02X\n", 
//                item.SOC, item.speedKmh, item.displaySpeed, item.odometerKm,
//                item.battery_voltage, item.battery_current, 
//                item.available_energy, item.charging_status);
//        fflush(fpt);
//    }
//    fclose(fpt);
//    return NULL;
//}

// ===== UPDATED CSV OUTPUT FOR V2V/V2G DATA =====
void* printer_thread(void* arg) {
    FILE *fpt = fopen(esp_now_filename, "w+");
    if (!fpt) {
        perror("Failed to open ESP-NOW V2V/V2G CSV file");
        return NULL;
    }
    
    // Updated CSV header for V2V/V2G data
    fprintf(fpt, "MAC,SOC,BatteryVoltage,BatteryCurrent,AvailableEnergy,ChargingStatus,");
    fprintf(fpt, "Speed,Acceleration,Gear,BrakeStatus,");
    fprintf(fpt, "Latitude,Longitude,Altitude,GPSValid,");
    fprintf(fpt, "IsCharging,DesiredSOC,ReadyToCharge,Timestamp\n");
    fflush(fpt);
    
    while (1) {
        Item item;
        dequeue(&item);
        
        // MAC address
        for (int i = 0; i < 6; ++i) {
            fprintf(fpt, "%02X", item.MacAddress[i]);
            if (i < 5) fprintf(fpt, ":");
        }
        
        // V2V/V2G data output
        fprintf(fpt, ",%u,%.1f,%.1f,%.1f,0x%02X,", 
                item.SOC, item.battery_voltage, item.battery_current, 
                item.available_energy, item.charging_status);
        fprintf(fpt, "%.1f,%.0f,%s,%u,", 
                item.speedKmh, item.acceleration, item.gear, item.brake_status);
        fprintf(fpt, "%.6f,%.6f,%.1f,%u,", 
                item.gps_latitude, item.gps_longitude, item.gps_altitude, item.gps_valid);
        fprintf(fpt, "%s,%.0f,%s,%u\n",
                item.is_charging_detected ? "YES" : "NO", item.desired_soc,
                item.ready_to_charge ? "YES" : "NO", item.timestamp);
        fflush(fpt);
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
            .battery_voltage = 58.5,
            .battery_current = -2.1,
            .available_energy = 5.2,
            .charging_status = 0x00,
            .MacAddress = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01}
        };
        uint8_t header[2] = { HEADER_BYTE_1, HEADER_BYTE_2 };
        write(serial_port, header, 2);
        write(serial_port, &item, sizeof(Item));
        usleep(5000000); // 5 seconds
    }
    return NULL;
}

// Simple CAN reader thread with charging detection
void* can_reader_thread(void* arg) {
    printf("Simple CAN Data Reader with Charging Detection\n");
    printf("Monitoring for charging sessions based on current change...\n\n");
    
    time_t last_modified = 0;
    
    while (1) {
        struct stat file_stat;
        if (stat(CAN_SNAPSHOT_FILE, &file_stat) == 0) {
            if (file_stat.st_mtime != last_modified) {
                last_modified = file_stat.st_mtime;
                
                int lines_processed = read_entire_can_file();
                
                if (lines_processed > 0) {
                    MergedCANData merged = create_merged_can_data();
                    save_merged_can_data_to_csv(&merged);
                    
                    // V2G Enhancement: Detect charging sessions
                    detect_charging_session(&merged);
                    
                    // Print charging status if active
                    pthread_mutex_lock(&session_mutex);
                    if (current_session.is_charging) {
                        print_charging_status(&merged, &current_session);
                    }
                    pthread_mutex_unlock(&session_mutex);
                }
            }
        }
        
        sleep(1);
    }
    return NULL;
}

//// Enhanced sender thread with REAL CAN data
//void* can_to_esp32_sender_thread(void* arg) {
//    printf("Sending REAL CAN battery data to ESP32\n");
//    
//    while (1) {
//        MergedCANData merged = create_merged_can_data();
//        
//        if (merged.has_data) {
//            Item can_item = {
//                .SOC = (unsigned short)merged.soc,
//                .speedKmh = merged.motor_speed,
//                .odometerKm = merged.odometer,
//                .displaySpeed = merged.gps_speed,  // Use GPS speed as display speed
//                
//                // NEW: Send REAL battery data from CAN
//                .battery_voltage = merged.battery_voltage,      // Real voltage from CAN 0x425
//                .battery_current = merged.current,              // Real current from CAN 0x155  
//                .available_energy = merged.available_energy,    // Real energy from CAN 0x425
//                .charging_status = merged.charging_status,      // Real charging status from CAN 0x425
//                
//                .MacAddress = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
//            };
//            
//            uint8_t header[2] = { HEADER_BYTE_1, HEADER_BYTE_2 };
//            write(serial_port, header, 2);
//            write(serial_port, &can_item, sizeof(Item));
//            
//            // Debug output every 10 sends
//            static int send_count = 0;
//            send_count++;
//            if (send_count % 10 == 0) {
//                printf("Sent real CAN data [%d]: SOC=%d%%, V=%.1fV, I=%.1fA, E=%.1fkWh, Charging=%s\n",
//                       send_count, can_item.SOC, can_item.battery_voltage, 
//                       can_item.battery_current, can_item.available_energy, 
//                       merged.is_charging_detected ? "YES" : "NO");
//            }
//        }
//        
//        sleep(3);
//    }
//    return NULL;
//}

// ===== UPDATED SENDING FUNCTION IN JETSON CODE =====
void* can_to_esp32_sender_thread(void* arg) {
    printf("Sending V2V/V2G CAN data to ESP32\n");
    
    while (1) {
        MergedCANData merged = create_merged_can_data();
        
        if (merged.has_data) {
            Item v2v_v2g_item = {
                // Battery data (V2G)
                .SOC = (unsigned short)merged.soc,
                .battery_voltage = merged.battery_voltage,      // Real voltage from CAN 0x425
                .battery_current = merged.current,              // Real current from CAN 0x155  
                .available_energy = merged.available_energy,    // Real energy from CAN 0x425
                .charging_status = merged.charging_status,      // Real charging status from CAN 0x425
                
                // Vehicle dynamics (V2V)
                .speedKmh = merged.motor_speed,                 // From CAN 0x19F
                .acceleration = (float)merged.accelerator,      // From CAN 0x59B (0-100%)
                .brake_status = (uint8_t)merged.brake,          // From CAN 0x59B (0/1)
                
                // Vehicle position (V2V/V2G)
                .gps_latitude = merged.gps_latitude,            // From GPS
                .gps_longitude = merged.gps_longitude,          // From GPS
                .gps_altitude = merged.gps_altitude,            // From GPS
                .gps_valid = (uint8_t)merged.gps_valid,         // GPS validity
                
                // V2G specific
                .is_charging_detected = merged.is_charging_detected,
                .desired_soc = 80.0f,                           // Default desired SOC
                .ready_to_charge = (merged.soc < 90),           // Ready if SOC < 90%
                
                // Communication
                .MacAddress = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
                .timestamp = (uint32_t)time(NULL)
            };
            
            // Copy gear string (max 3 chars + null terminator)
            strncpy(v2v_v2g_item.gear, merged.gear, 3);
            v2v_v2g_item.gear[3] = '\0';
            
            uint8_t header[2] = { HEADER_BYTE_1, HEADER_BYTE_2 };
            write(serial_port, header, 2);
            write(serial_port, &v2v_v2g_item, sizeof(Item));
            
            // Debug output every 10 sends
            static int send_count = 0;
            send_count++;
            if (send_count % 10 == 0) {
                printf("Sent V2V/V2G data [%d]: SOC=%d%%, V=%.1fV, I=%.1fA, Speed=%.1fkm/h\n",
                       send_count, v2v_v2g_item.SOC, v2v_v2g_item.battery_voltage, 
                       v2v_v2g_item.battery_current, v2v_v2g_item.speedKmh);
                printf("  Position: %.6f,%.6f Alt:%.1fm, Accel:%.0f%%, Brake:%s, Gear:%s\n",
                       v2v_v2g_item.gps_latitude, v2v_v2g_item.gps_longitude, 
                       v2v_v2g_item.gps_altitude, v2v_v2g_item.acceleration,
                       v2v_v2g_item.brake_status ? "ON" : "OFF", v2v_v2g_item.gear);
                printf("  Charging: %s, Ready: %s, Desired SOC: %.0f%%\n",
                       v2v_v2g_item.is_charging_detected ? "YES" : "NO",
                       v2v_v2g_item.ready_to_charge ? "YES" : "NO",
                       v2v_v2g_item.desired_soc);
            }
        }
        
        sleep(3);  // Send every 3 seconds
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
    strftime(parsed_filename, sizeof(parsed_filename), "merged_can_data_%Y%m%d_%H%M.csv", t);
    strftime(esp_now_filename, sizeof(esp_now_filename), "ESP_now_data_%Y%m%d_%H%M.csv", t);
    strftime(v2g_log_filename, sizeof(v2g_log_filename), "v2g_sessions_%Y%m%d_%H%M.csv", t);
    
    printf("Simplified ESP-NOW + V2G System with Real CAN Data\n");
    printf("=================================================\n");
    printf("ESP-NOW Serial: %s\n", esp_serial_port);
    printf("CAN Data Input: %s\n", CAN_SNAPSHOT_FILE);
    printf("ESP-NOW CSV: %s\n", esp_now_filename);
    printf("CAN+GPS CSV: %s\n", parsed_filename);
    printf("V2G Sessions: %s\n", v2g_log_filename);
    printf("Features: Real CAN Battery Data + Simple Current-Based V2G Detection\n");
    printf("Charging Detection: Current > %.1fA\n", CHARGING_CURRENT_THRESHOLD);
    printf("=================================================\n\n");
    
    serial_port = setup_serial(esp_serial_port);
    if (serial_port < 0) return 1;
    
    pthread_t rx_tid, tx_tid, print_tid, can_reader_tid, can_sender_tid;
    
    pthread_create(&rx_tid, NULL, receiver_thread, NULL);
    pthread_create(&tx_tid, NULL, sender_thread, NULL);
    pthread_create(&print_tid, NULL, printer_thread, NULL);
    pthread_create(&can_reader_tid, NULL, can_reader_thread, NULL);
    pthread_create(&can_sender_tid, NULL, can_to_esp32_sender_thread, NULL);
    
    printf("Simple V2G System Active!\n");
    printf("Now sending REAL battery data from CAN:\n");
    printf("  • Voltage from CAN 0x425 (Battery Management)\n");
    printf("  • Current from CAN 0x155 (Battery Status)\n");
    printf("  • Energy from CAN 0x425 (Available Energy)\n");
    printf("  • Charging Status from CAN 0x425 (Protocol Status)\n");
    printf("V2G will start when:\n");
    printf("  1. Vehicle is plugged in (physical connection)\n");
    printf("  2. Current > %.1fA detected (charging starts)\n", CHARGING_CURRENT_THRESHOLD);
    printf("Monitoring for charging sessions...\n\n");
    
    pthread_join(rx_tid, NULL);
    pthread_join(tx_tid, NULL);
    pthread_join(print_tid, NULL);
    pthread_join(can_reader_tid, NULL);
    pthread_join(can_sender_tid, NULL);
    
    close(serial_port);
    return 0;
}
