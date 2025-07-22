#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <pthread.h>
#include <stdlib.h>

#define HEADER_BYTE_1 0xAA
#define HEADER_BYTE_2 0x55
#define HEADER_SIZE 2
#define QUEUE_SIZE 10
#define MAX_LINE_LENGTH 512
#define CAN_SNAPSHOT_FILE "can_data_snapshot.csv"
#define MAX_CAN_IDS 100  // Maximum number of unique CAN IDs to track

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
    int active;  // 1 if this slot is in use, 0 if empty
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
            tracked_can_ids_count++;
            return i;
        }
    }
    return -1; // Array is full
}

// Update or add CAN data for a specific ID
void update_can_data(const CANData *new_data) {
    pthread_mutex_lock(&can_tracker_mutex);
    
    int index = find_can_id_index(new_data->can_id);
    
    if (index >= 0) {
        // Update existing CAN ID with new data
        can_id_tracker[index].latest_data = *new_data;
        printf("[CAN UPDATE] ID: %03X | Time: %s | Data: %s\n", 
               new_data->can_id, new_data->timestamp, new_data->data_hex);
    } else {
        // New CAN ID found
        index = add_new_can_id(new_data->can_id, new_data);
        if (index >= 0) {
            printf("[NEW CAN ID] ID: %03X | Time: %s | Data: %s | Total IDs: %d\n", 
                   new_data->can_id, new_data->timestamp, new_data->data_hex, tracked_can_ids_count);
        } else {
            printf("[ERROR] Cannot track more CAN IDs (limit: %d)\n", MAX_CAN_IDS);
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

// Print all tracked CAN IDs and their latest data
void print_all_can_data() {
    pthread_mutex_lock(&can_tracker_mutex);
    
    printf("\n=== ALL TRACKED CAN IDs ===\n");
    for (int i = 0; i < MAX_CAN_IDS; i++) {
        if (can_id_tracker[i].active) {
            const CANData *data = &can_id_tracker[i].latest_data;
            printf("ID: %03X | Time: %s | DLC: %d | Data: %s\n",
                   data->can_id, data->timestamp, data->dlc, data->data_hex);
        }
    }
    printf("=== Total: %d unique CAN IDs ===\n\n", tracked_can_ids_count);
    
    pthread_mutex_unlock(&can_tracker_mutex);
}

void print_item(const Item *item) {
    printf("[ESP-NOW] MAC: %02X:%02X:%02X:%02X:%02X:%02X | SOC: %u | Speed: %.1f km/h | Display: %.1f km/h | Odo: %.1f km\n",
        item->MacAddress[0], item->MacAddress[1], item->MacAddress[2],
        item->MacAddress[3], item->MacAddress[4], item->MacAddress[5],
        item->SOC, item->speedKmh, item->displaySpeed, item->odometerKm);
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
    FILE *fpt = fopen("ESP_now_data.csv", "w+");
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

// MODIFIED THREAD: CAN Data Reader with ID tracking
void* can_reader_thread(void* arg) {
    printf("CAN Data Reader thread started - monitoring %s\n", CAN_SNAPSHOT_FILE);
    printf("Tracking unique CAN IDs and their latest data...\n");
    printf("Will check for new CAN data every 2 seconds...\n\n");
    
    // Initialize the tracker array
    memset(can_id_tracker, 0, sizeof(can_id_tracker));
    
    int cycle_count = 0;
    
    while (1) {
        int new_lines = read_new_can_lines();
        
        if (new_lines > 0) {
            printf("Read %d new CAN data lines\n", new_lines);
        } else if (new_lines == 0) {
            printf("No new CAN data since last check\n");
        } else {
            printf("Failed to read from %s (file may not exist yet)\n", CAN_SNAPSHOT_FILE);
        }
        
        // Every 10 cycles (20 seconds), print all tracked CAN IDs
        cycle_count++;
        if (cycle_count >= 10) {
            print_all_can_data();
            cycle_count = 0;
        }
        
        // Wait 2 seconds before checking again
        sleep(2);
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
    
    printf("ESP-NOW Communication System with CAN Data Integration\n");
    printf("ESP-NOW Serial Port: %s\n", esp_serial_port);
    printf("CAN Snapshot File: %s\n", CAN_SNAPSHOT_FILE);
    printf("Max Trackable CAN IDs: %d\n\n", MAX_CAN_IDS);
    
    serial_port = setup_serial(esp_serial_port);
    if (serial_port < 0) return 1;
    
    pthread_t rx_tid, tx_tid, print_tid, can_reader_tid;
    
    // Start all threads
    pthread_create(&rx_tid, NULL, receiver_thread, NULL);
    pthread_create(&tx_tid, NULL, sender_thread, NULL);
    pthread_create(&print_tid, NULL, printer_thread, NULL);
    pthread_create(&can_reader_tid, NULL, can_reader_thread, NULL);
    
    printf("All threads started successfully!\n");
    printf("- ESP-NOW receiver/sender threads active\n");
    printf("- CSV printer thread active\n");
    printf("- CAN data reader thread active (tracking unique CAN IDs)\n\n");
    
    // Wait for threads to complete (they run indefinitely)
    pthread_join(rx_tid, NULL);
    pthread_join(tx_tid, NULL);
    pthread_join(print_tid, NULL);
    pthread_join(can_reader_tid, NULL);
    
    close(serial_port);
    return 0;
}
