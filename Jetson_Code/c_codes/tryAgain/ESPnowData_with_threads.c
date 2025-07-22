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

int serial_port;
pthread_mutex_t queue_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t queue_cond = PTHREAD_COND_INITIALIZER;
Item queue[QUEUE_SIZE];
int queue_head = 0;
int queue_tail = 0;
int queue_count = 0;

// Global variable to hold the latest CAN data
CANData latest_can_data = {0};
pthread_mutex_t can_data_mutex = PTHREAD_MUTEX_INITIALIZER;

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

// Function to read the last line from CAN snapshot CSV
int read_last_can_line(CANData *can_data) {
    FILE *file = fopen(CAN_SNAPSHOT_FILE, "r");
    if (!file) {
        // File might not exist yet, return invalid data
        can_data->valid = 0;
        return -1;
    }
    
    char line[MAX_LINE_LENGTH];
    char last_line[MAX_LINE_LENGTH] = "";
    int line_count = 0;
    
    // Read through the file to find the last line
    while (fgets(line, sizeof(line), file)) {
        line_count++;
        // Skip the header line
        if (line_count > 1) {
            strcpy(last_line, line);
        }
    }
    
    fclose(file);
    
    // If we didn't find any data lines (only header or empty file)
    if (strlen(last_line) == 0) {
        can_data->valid = 0;
        return -1;
    }
    
    // Parse the CSV line: timestamp,can_id,dlc,data_hex,data_bytes
    char *token;
    char *line_copy = strdup(last_line);
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

// Function to print CAN data
void print_can_data(const CANData *can_data) {
    if (can_data->valid) {
        printf("[CAN DATA] Time: %s | ID: %03X | DLC: %d | Data: %s\n",
               can_data->timestamp, can_data->can_id, can_data->dlc, can_data->data_hex);
    } else {
        printf("[CAN DATA] No valid data available\n");
    }
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

// NEW THREAD: CAN Data Reader
void* can_reader_thread(void* arg) {
    printf("CAN Data Reader thread started - monitoring %s\n", CAN_SNAPSHOT_FILE);
    printf("Will check for new CAN data every 2 seconds...\n\n");
    
    while (1) {
        CANData temp_can_data;
        
        // Try to read the last line from the CAN snapshot file
        if (read_last_can_line(&temp_can_data) == 0) {
            // Successfully read data, update the global variable
            pthread_mutex_lock(&can_data_mutex);
            latest_can_data = temp_can_data;
            pthread_mutex_unlock(&can_data_mutex);
            
            // Print the CAN data to terminal
            print_can_data(&temp_can_data);
        } else {
            // Failed to read data
            printf("[CAN DATA] Failed to read from %s (file may not exist yet)\n", CAN_SNAPSHOT_FILE);
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

int main() {
    printf("ESP-NOW Communication System with CAN Data Integration\n");
    printf("ESP-NOW Serial Port: /dev/ttyUSB1\n");
    printf("CAN Snapshot File: %s\n\n", CAN_SNAPSHOT_FILE);
    
    serial_port = setup_serial("/dev/ttyUSB1");
    if (serial_port < 0) return 1;
    
    pthread_t rx_tid, tx_tid, print_tid, can_reader_tid;
    
    // Start all threads
    pthread_create(&rx_tid, NULL, receiver_thread, NULL);
    pthread_create(&tx_tid, NULL, sender_thread, NULL);
    pthread_create(&print_tid, NULL, printer_thread, NULL);
    pthread_create(&can_reader_tid, NULL, can_reader_thread, NULL);  // NEW THREAD
    
    printf("All threads started successfully!\n");
    printf("- ESP-NOW receiver/sender threads active\n");
    printf("- CSV printer thread active\n");
    printf("- CAN data reader thread active\n\n");
    
    // Wait for threads to complete (they run indefinitely)
    pthread_join(rx_tid, NULL);
    pthread_join(tx_tid, NULL);
    pthread_join(print_tid, NULL);
    pthread_join(can_reader_tid, NULL);  // NEW THREAD JOIN
    
    close(serial_port);
    return 0;
}
