///save two csv files

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>
#include <time.h>

#define MAX_LINE_LENGTH 128
#define QUEUE_SIZE 1000
#define COPY_INTERVAL_SECONDS 15

typedef struct {
    unsigned int can_id;
    unsigned char dlc;
    unsigned char data[8];
    struct timespec timestamp;
} CANFrame;

// --- Queue implementation ---
typedef struct {
    CANFrame items[QUEUE_SIZE];
    int front, rear, count;
    pthread_mutex_t lock;
    pthread_cond_t not_empty;
} CANQueue;

CANQueue queue;

// Global variables for pause/copy functionality
volatile int pause_requested = 0;
volatile int copy_completed = 0;
pthread_mutex_t pause_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t resume_condition = PTHREAD_COND_INITIALIZER;

void init_queue(CANQueue *q) {
    q->front = q->rear = q->count = 0;
    pthread_mutex_init(&q->lock, NULL);
    pthread_cond_init(&q->not_empty, NULL);
}

void enqueue(CANQueue *q, CANFrame item) {
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

CANFrame dequeue(CANQueue *q) {
    pthread_mutex_lock(&q->lock);
    while (q->count == 0)
        pthread_cond_wait(&q->not_empty, &q->lock);
    CANFrame item = q->items[q->front];
    q->front = (q->front + 1) % QUEUE_SIZE;
    q->count--;
    pthread_mutex_unlock(&q->lock);
    return item;
}

// Function to copy CSV file to snapshot
int copy_csv_to_snapshot() {
    FILE *source, *dest;
    char buffer[4096];
    size_t bytes_read;
    
    source = fopen("can_data.csv", "rb");
    if (!source) {
        perror("Failed to open source CSV file");
        return -1;
    }
    
    dest = fopen("can_data_snapshot.csv", "wb");
    if (!dest) {
        perror("Failed to create snapshot CSV file");
        fclose(source);
        return -1;
    }
    
    // Copy file contents
    while ((bytes_read = fread(buffer, 1, sizeof(buffer), source)) > 0) {
        if (fwrite(buffer, 1, bytes_read, dest) != bytes_read) {
            perror("Error writing to snapshot file");
            fclose(source);
            fclose(dest);
            return -1;
        }
    }
    
    fclose(source);
    fclose(dest);
    
    return 0;
}

// --- Serial Setup & CAN Parsing ---
int open_serial_port(const char *device) {
    int fd = open(device, O_RDONLY | O_NOCTTY);
    if (fd == -1) {
        perror("Unable to open serial port");
        exit(EXIT_FAILURE);
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

// --- Thread 1: Serial Reader ---
void *reader_thread(void *arg) {
    const char *port = (const char *)arg;
    int fd = open_serial_port(port);
    
    printf("Reading from serial port: %s\n", port);
    printf("Waiting for CAN data...\n");
    
    FILE *serial_stream = fdopen(fd, "r");
    if (!serial_stream) {
        perror("fdopen failed");
        close(fd);
        return NULL;
    }
    
    char line[MAX_LINE_LENGTH];
    int frame_count = 0;
    
    while (fgets(line, sizeof(line), serial_stream)) {
        // Check for pause request
        pthread_mutex_lock(&pause_mutex);
        if (pause_requested) {
            printf("Data collection paused for snapshot creation...\n");
            // Wait until copy is completed
            while (pause_requested) {
                pthread_cond_wait(&resume_condition, &pause_mutex);
            }
            printf("Data collection resumed.\n");
        }
        pthread_mutex_unlock(&pause_mutex);
        
        // Remove newline
        line[strcspn(line, "\r\n")] = 0;
        
        // Skip empty lines and startup messages
        if (strlen(line) == 0 || strstr(line, "CAN Bus Reader") || 
            strstr(line, "Ready") || strstr(line, "---")) {
            continue;
        }
        
        CANFrame frame;
        if (parse_can_line(line, &frame)) {
            enqueue(&queue, frame);
            frame_count++;
            
            // Print every 100th frame for monitoring
            if (frame_count % 100 == 0) {
                printf("Processed %d CAN frames\n", frame_count);
            }
        } else {
            printf("Failed to parse line: %s\n", line);
        }
    }
    
    printf("Serial stream ended\n");
    fclose(serial_stream);
    return NULL;
}

// --- Thread 2: CSV Writer ---
void *writer_thread(void *arg) {
    FILE *fpt = fopen("can_data.csv", "w+");
    if (!fpt) {
        perror("Failed to open CSV file");
        return NULL;
    }
    
    // Write CSV header
    fprintf(fpt, "timestamp,can_id,dlc,data_hex,data_bytes\n");
    fflush(fpt);
    
    printf("Writing CAN data to can_data.csv\n");
    
    while (1) {
        CANFrame frame = dequeue(&queue);
        
        // Check for pause request
        pthread_mutex_lock(&pause_mutex);
        if (pause_requested) {
            printf("CSV writer paused for snapshot creation...\n");
            fflush(fpt);  // Ensure all data is written before copy
            
            // Wait until copy is completed
            while (pause_requested) {
                pthread_cond_wait(&resume_condition, &pause_mutex);
            }
            printf("CSV writer resumed.\n");
        }
        pthread_mutex_unlock(&pause_mutex);
        
        // Format timestamp
        char timestamp_str[64];
        snprintf(timestamp_str, sizeof(timestamp_str), "%ld.%09ld", 
                frame.timestamp.tv_sec, frame.timestamp.tv_nsec);
        
        // Write to CSV
        fprintf(fpt, "%s,%03X,%d,", timestamp_str, frame.can_id, frame.dlc);
        
        // Write data as hex string
        for (int i = 0; i < frame.dlc; i++) {
            fprintf(fpt, "%02X", frame.data[i]);
            if (i < frame.dlc - 1) fprintf(fpt, " ");
        }
        fprintf(fpt, ",");
        
        // Write data as individual bytes
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

// --- Thread 3: Live Monitor (Optional) ---
void *monitor_thread(void *arg) {
    printf("\n=== Live CAN Monitor (Press Ctrl+C to stop) ===\n");
    printf("Format: [Timestamp] ID:DLC Data\n\n");
    
    while (1) {
        CANFrame frame = dequeue(&queue);
        
        // Check for pause request
        pthread_mutex_lock(&pause_mutex);
        if (pause_requested) {
            // Wait until copy is completed
            while (pause_requested) {
                pthread_cond_wait(&resume_condition, &pause_mutex);
            }
        }
        pthread_mutex_unlock(&pause_mutex);
        
        // Print live data
        printf("[%ld.%03ld] %03X:%d ", 
               frame.timestamp.tv_sec, frame.timestamp.tv_nsec / 1000000,
               frame.can_id, frame.dlc);
        
        for (int i = 0; i < frame.dlc; i++) {
            printf("%02X ", frame.data[i]);
        }
        printf("\n");
    }
    
    return NULL;
}

// --- Thread 4: Automatic Copy Handler (15-second timer) ---
void *copy_handler_thread(void *arg) {
    printf("Automatic snapshot creation every %d seconds started\n", COPY_INTERVAL_SECONDS);
    
    while (1) {
        // Wait for 15 seconds
        sleep(COPY_INTERVAL_SECONDS);
        
        printf("\n--- Creating snapshot (pausing data collection) ---\n");
        
        // Request pause
        pthread_mutex_lock(&pause_mutex);
        pause_requested = 1;
        pthread_mutex_unlock(&pause_mutex);
        
        // Wait 1 second to ensure all threads are paused
        sleep(1);
        
        // Perform the copy
        printf("Copying can_data.csv -> can_data_snapshot.csv...\n");
        if (copy_csv_to_snapshot() == 0) {
            printf("Snapshot created successfully.\n");
        } else {
            printf("Snapshot creation failed.\n");
        }
        
        // Resume operations
        pthread_mutex_lock(&pause_mutex);
        pause_requested = 0;
        pthread_cond_broadcast(&resume_condition);
        pthread_mutex_unlock(&pause_mutex);
        
        printf("--- Snapshot complete, resuming data collection ---\n\n");
    }
    
    return NULL;
}

// --- Main ---
int main(int argc, char *argv[]) {
    const char *serial_port = (argc >= 2) ? argv[1] : "/dev/ttyUSB0";
    
    printf("CAN Data Capture Tool with Automatic Snapshots\n");
    printf("Serial Port: %s\n", serial_port);
    printf("Main CSV: can_data.csv (continuous)\n");
    printf("Snapshot CSV: can_data_snapshot.csv (updated every %d seconds)\n\n", COPY_INTERVAL_SECONDS);
    
    init_queue(&queue);
    
    pthread_t reader_tid, writer_tid, monitor_tid, copy_handler_tid;
    
    // Start threads
    pthread_create(&reader_tid, NULL, reader_thread, (void *)serial_port);
    pthread_create(&writer_tid, NULL, writer_thread, NULL);
    pthread_create(&copy_handler_tid, NULL, copy_handler_thread, NULL);
    
    // Optionally start monitor thread (comment out if not needed)
    // pthread_create(&monitor_tid, NULL, monitor_thread, NULL);
    
    // Wait for reader thread (main control)
    pthread_join(reader_tid, NULL);
    
    // Note: writer and copy handler threads will continue running
    // In a real application, you'd want proper shutdown handling
    
    return 0;
}
