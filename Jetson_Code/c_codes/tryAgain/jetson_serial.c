//Modified to accept command line arguments for USB ports with improved CAN parsing
//key improvements made to your dual ESP32 code on git commit
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
#define QUEUE_SIZE 1000
#define MAX_LINE_LENGTH 128

// Struct for CAN data from ESP32(2)
typedef struct {
    unsigned int can_id;
    unsigned char dlc;
    unsigned char data[8];
    struct timespec timestamp;
} CANFrame;

// Struct for ESP-NOW data from ESP32(1)
typedef struct {
    unsigned short SOC;
    float speedKmh;
    float odometerKm;
    float displaySpeed;
    uint8_t MacAddress[6];
    struct timespec timestamp;
} Item;

// Global serial port file descriptors
int serial_port1; // ESP32(1) - ESP-NOW
int serial_port2; // ESP32(2) - CAN

// Global port paths (set from command line arguments)
const char *esp_now_port = NULL;
const char *can_port = NULL;

// Queue for ESP-NOW Items
pthread_mutex_t queue_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t queue_cond = PTHREAD_COND_INITIALIZER;
Item queue[QUEUE_SIZE];
int queue_head = 0;
int queue_tail = 0;
int queue_count = 0;

// Queue for CAN Frames
typedef struct {
    CANFrame items[QUEUE_SIZE];
    int front, rear, count;
    pthread_mutex_t lock;
    pthread_cond_t not_empty;
} CANQueue;

CANQueue queueCAN;

// CAN Queue functions
void init_queueCAN(CANQueue *q) {
    q->front = q->rear = q->count = 0;
    pthread_mutex_init(&q->lock, NULL);
    pthread_cond_init(&q->not_empty, NULL);
}

void enqueueCAN(CANQueue *q, CANFrame item) {
    pthread_mutex_lock(&q->lock);
    if (q->count == QUEUE_SIZE) {
        // Simple overflow handling (overwrite oldest)
        q->front = (q->front + 1) % QUEUE_SIZE;
        q->count--;
    }
    q->items[q->rear] = item;
    q->rear = (q->rear + 1) % QUEUE_SIZE;
    q->count++;
    pthread_cond_signal(&q->not_empty);
    pthread_mutex_unlock(&q->lock);
}

CANFrame dequeueCAN(CANQueue *q) {
    pthread_mutex_lock(&q->lock);
    while (q->count == 0)
        pthread_cond_wait(&q->not_empty, &q->lock);
    CANFrame item = q->items[q->front];
    q->front = (q->front + 1) % QUEUE_SIZE;
    q->count--;
    pthread_mutex_unlock(&q->lock);
    return item;
}

// ESP-NOW Queue functions
int enqueue(Item *item) {
    pthread_mutex_lock(&queue_mutex);
    if (queue_count == QUEUE_SIZE) {
        // Overwrite oldest item instead of failing
        queue_head = (queue_head + 1) % QUEUE_SIZE;
        queue_count--;
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

// Utility functions
void print_item(const Item *item) {
    printf("[ESP-NOW] MAC: %02X:%02X:%02X:%02X:%02X:%02X | SOC: %u | Speed: %.1f km/h | Display: %.1f km/h | Odo: %.1f km\n",
        item->MacAddress[0], item->MacAddress[1], item->MacAddress[2],
        item->MacAddress[3], item->MacAddress[4], item->MacAddress[5],
        item->SOC, item->speedKmh, item->displaySpeed, item->odometerKm);
}

// Improved CAN parsing functions (from working code)
int hex_char_to_int(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

int parse_hex_byte(const char *str) {
    int high = hex_char_to_int(str[0]);
    int low = hex_char_to_int(str[1]);
    if (high == -1 || low == -1) return -1;
    return (high << 4) | low;
}

int parse_can_line(const char *line, CANFrame *frame) {
    // Expected format: "SIM 19F 8 FF FF 7D 0F 38 FF 40 FE"
    if (strncmp(line, "SIM ", 4) != 0) {
        return 0;
    }
    
    // Get timestamp
    clock_gettime(CLOCK_REALTIME, &frame->timestamp);
    
    // Parse the line manually
    char *token;
    char *line_copy = strdup(line + 4); // Skip "SIM "
    char *saveptr;
    
    // Parse CAN ID (hex)
    token = strtok_r(line_copy, " ", &saveptr);
    if (!token) {
        free(line_copy);
        return 0;
    }
    frame->can_id = (unsigned int)strtol(token, NULL, 16);
    
    // Parse DLC (decimal)
    token = strtok_r(NULL, " ", &saveptr);
    if (!token) {
        free(line_copy);
        return 0;
    }
    frame->dlc = (unsigned char)atoi(token);
    
    // Validate DLC
    if (frame->dlc > 8) {
        free(line_copy);
        return 0;
    }
    
    // Parse data bytes (always expect 8 hex bytes from ESP32)
    for (int i = 0; i < 8; i++) {
        token = strtok_r(NULL, " ", &saveptr);
        if (!token) {
            free(line_copy);
            return 0;
        }
        
        int byte_val = parse_hex_byte(token);
        if (byte_val == -1) {
            free(line_copy);
            return 0;
        }
        frame->data[i] = (unsigned char)byte_val;
    }
    
    free(line_copy);
    return 1;
}

int setup_serial_binary(const char *port_path) {
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

int setup_serial_text(const char *device) {
    int fd = open(device, O_RDONLY | O_NOCTTY);
    if (fd == -1) {
        perror("Unable to open serial port");
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    
    // Set baud rate
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    
    // Configure port settings
    options.c_cflag &= ~PARENB;        // No parity
    options.c_cflag &= ~CSTOPB;        // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;            // 8 data bits
    options.c_cflag |= CREAD | CLOCAL; // Enable receiver, local mode
    
    // Raw input mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
    options.c_oflag &= ~OPOST;         // Raw output mode
    
    // Set timeout (1 second)
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;
    
    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIFLUSH); // Flush input buffer
    
    return fd;
}

// Thread functions for ESP-NOW (binary data)
void* receiver_thread(void* arg) {
    uint8_t header[2];
    uint8_t buffer[sizeof(Item)];
    int esp_now_count = 0;
    
    printf("ESP-NOW receiver thread started\n");
    
    while (1) {
        if (read(serial_port1, &header[0], 1) != 1) continue;
        if (header[0] != HEADER_BYTE_1) continue;
        if (read(serial_port1, &header[1], 1) != 1) continue;
        if (header[1] != HEADER_BYTE_2) continue;
        
        int bytes_read = 0;
        while (bytes_read < sizeof(Item)) {
            int result = read(serial_port1, buffer + bytes_read, sizeof(Item) - bytes_read);
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
            
            // Add timestamp
            clock_gettime(CLOCK_REALTIME, &item.timestamp);
            
            enqueue(&item);
            esp_now_count++;
            
            if (esp_now_count % 50 == 0) {
                printf("Processed %d ESP-NOW messages\n", esp_now_count);
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
    
    // Write CSV header with timestamp
    fprintf(fpt, "timestamp,MAC,SOC,Speed,DisplaySpeed,Odometer\n");
    fflush(fpt);
    
    printf("Writing ESP-NOW data to ESP_now_data.csv\n");
    
    while (1) {
        Item item;
        dequeue(&item);
        
        print_item(&item);  // Console output
        
        // Format timestamp
        char timestamp_str[64];
        snprintf(timestamp_str, sizeof(timestamp_str), "%ld.%09ld", 
                item.timestamp.tv_sec, item.timestamp.tv_nsec);
        
        // CSV output with timestamp
        fprintf(fpt, "%s,", timestamp_str);
        
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

void* response_sender_thread(void* arg) {
    while (1) {
        Item received_item;
        dequeue(&received_item);  // Get data from ESP32s
        
        // Process the received data and create response
        Item response = {
            .SOC = 95,  // Command: Set SOC to 95%
            .speedKmh = 65.0,  // Command: Set speed limit to 65
            .displaySpeed = 63.0,
            .odometerKm = received_item.odometerKm + 1,  // Increment odometer
            .MacAddress = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}  // Will be set by ESP32(1)
        };
        
        // Send response back to ESP32(1)
        uint8_t header[2] = { HEADER_BYTE_1, HEADER_BYTE_2 };
        write(serial_port1, header, 2);
        write(serial_port1, &response, sizeof(Item));
        
        printf("[JETSON] Sent response: SOC=%u, Speed=%.1f\n", 
               response.SOC, response.speedKmh);
    }
    return NULL;
}

// Thread functions for CAN (text data) - IMPROVED
void *reader_thread(void *arg) {
    int fd = setup_serial_text(can_port);  // Use global can_port
    if (fd < 0) {
        fprintf(stderr, "Failed to open CAN serial port: %s\n", can_port);
        return NULL;
    }
    
    printf("Reading CAN data from: %s\n", can_port);
    printf("Waiting for CAN data...\n");
    
    FILE *serial_stream = fdopen(fd, "r");
    if (!serial_stream) {
        perror("fdopen failed for CAN port");
        close(fd);
        return NULL;
    }
    
    char line[MAX_LINE_LENGTH];
    int frame_count = 0;
    
    while (fgets(line, sizeof(line), serial_stream)) {
        // Remove newline
        line[strcspn(line, "\r\n")] = 0;
        
        // Skip empty lines and startup messages
        if (strlen(line) == 0 || strstr(line, "CAN Bus Reader") || 
            strstr(line, "Ready") || strstr(line, "---")) {
            continue;
        }
        
        CANFrame frame;
        if (parse_can_line(line, &frame)) {
            enqueueCAN(&queueCAN, frame);
            frame_count++;
            
            // Print every 100th frame for monitoring
            if (frame_count % 100 == 0) {
                printf("Processed %d CAN frames\n", frame_count);
            }
        } else {
            printf("Failed to parse CAN line: %s\n", line);
        }
    }
    
    printf("CAN serial stream ended\n");
    fclose(serial_stream);
    return NULL;
}

void *writer_thread(void *arg) {
    FILE *fpt = fopen("CAN_data.csv", "w+");
    if (!fpt) {
        perror("Failed to open CAN CSV file");
        return NULL;
    }
    
    // Write CSV header with timestamp
    fprintf(fpt, "timestamp,can_id,dlc,data_hex\n");
    fflush(fpt);
    
    printf("Writing CAN data to CAN_data.csv\n");
    
    while (1) {
        CANFrame frame = dequeueCAN(&queueCAN);
        
        // Format timestamp
        char timestamp_str[64];
        snprintf(timestamp_str, sizeof(timestamp_str), "%ld.%09ld", 
                frame.timestamp.tv_sec, frame.timestamp.tv_nsec);
        
        // Write to CSV with timestamp
        fprintf(fpt, "%s,%03X,%d,", timestamp_str, frame.can_id, frame.dlc);
        
        // Write data as hex string
        for (int i = 0; i < frame.dlc; i++) {
            fprintf(fpt, "%02X", frame.data[i]);
            if (i < frame.dlc - 1) fprintf(fpt, " ");
        }
        
        fprintf(fpt, "\n");
        fflush(fpt);
    }
    fclose(fpt);
    return NULL;
}

void print_usage(const char *program_name) {
    printf("Usage: %s <ESP-NOW_port> [CAN_port]\n", program_name);
    printf("Examples:\n");
    printf("  %s /dev/ttyUSB0 /dev/ttyUSB1     # Both ESP-NOW and CAN\n", program_name);
    printf("  %s /dev/ttyUSB0                  # ESP-NOW only\n", program_name);
    printf("\nPorts:\n");
    printf("  ESP-NOW_port: Serial port for ESP32(1) with ESP-NOW data (binary)\n");
    printf("  CAN_port:     Serial port for ESP32(2) with CAN data (text) [optional]\n");
}

// Modified main() to accept command line arguments
int main(int argc, char *argv[]) {
    // Check command line arguments
    if (argc < 2 || argc > 3) {
        print_usage(argv[0]);
        return 1;
    }
    
    // Set port paths from command line arguments
    esp_now_port = argv[1];                    // Required: ESP-NOW port
    can_port = (argc >= 3) ? argv[2] : NULL;   // Optional: CAN port
    
    printf("=== Jetson CAN/ESP-NOW Data Collector ===\n");
    printf("ESP-NOW port: %s\n", esp_now_port);
    if (can_port) {
        printf("CAN port: %s\n", can_port);
    } else {
        printf("CAN port: Not specified (ESP-NOW only mode)\n");
    }
    printf("==========================================\n\n");
    
    // Setup ESP-NOW serial port (required)
    serial_port1 = setup_serial_binary(esp_now_port);
    if (serial_port1 < 0) {
        fprintf(stderr, "Failed to setup ESP-NOW serial port: %s\n", esp_now_port);
        return 1;
    }
    
    // Test CAN port availability (optional)
    int can_available = 0;
    if (can_port != NULL) {
        int test_can_port = setup_serial_text(can_port);
        can_available = (test_can_port >= 0);
        if (test_can_port >= 0) close(test_can_port);
        if (!can_available) {
            printf("Warning: CAN port %s not available, running ESP-NOW only\n", can_port);
        }
    }
    
    init_queueCAN(&queueCAN);
    pthread_t rx_tid, response_tid, print_tid, reader_tid, writer_tid;
    
    // Always start ESP-NOW threads
    pthread_create(&rx_tid, NULL, receiver_thread, NULL);
    pthread_create(&response_tid, NULL, response_sender_thread, NULL);
    pthread_create(&print_tid, NULL, printer_thread, NULL);
    
    // Only start CAN threads if port is available
    if (can_available) {
        pthread_create(&reader_tid, NULL, reader_thread, NULL);
        pthread_create(&writer_tid, NULL, writer_thread, NULL);
        printf("Started all threads. Processing ESP-NOW and CAN data...\n");
    } else {
        printf("Started ESP-NOW threads only.\n");
    }
    
    // Join threads
    pthread_join(rx_tid, NULL);
    pthread_join(response_tid, NULL);
    pthread_join(print_tid, NULL);
    
    if (can_available) {
        pthread_join(reader_tid, NULL);
        pthread_join(writer_tid, NULL);
    }
    
    close(serial_port1);
    return 0;
}
