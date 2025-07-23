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
#define MAX_CSV_LINES 1000  // Maximum lines in CSV before overwriting

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

// Circular buffer management
typedef struct {
    char lines[MAX_CSV_LINES][256];  // Store CSV lines
    int current_line;
    int total_lines;
    int is_full;
    pthread_mutex_t csv_mutex;
} CSVBuffer;

CSVBuffer csv_buffer = {0};

void init_csv_buffer() {
    csv_buffer.current_line = 0;
    csv_buffer.total_lines = 0;
    csv_buffer.is_full = 0;
    pthread_mutex_init(&csv_buffer.csv_mutex, NULL);
}

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

// Add a line to the circular CSV buffer
void add_csv_line(const char *line) {
    pthread_mutex_lock(&csv_buffer.csv_mutex);
    
    // Copy the line to the current position
    strncpy(csv_buffer.lines[csv_buffer.current_line], line, 255);
    csv_buffer.lines[csv_buffer.current_line][255] = '\0';
    
    // Move to next position
    csv_buffer.current_line = (csv_buffer.current_line + 1) % MAX_CSV_LINES;
    
    // Update counters
    if (!csv_buffer.is_full) {
        csv_buffer.total_lines++;
        if (csv_buffer.total_lines >= MAX_CSV_LINES) {
            csv_buffer.is_full = 1;
        }
    }
    
    pthread_mutex_unlock(&csv_buffer.csv_mutex);
}

// Write the CSV buffer to file
int write_csv_buffer_to_file(const char *filename) {
    FILE *fpt = fopen(filename, "w");
    if (!fpt) {
        perror("Failed to open CSV file for writing");
        return -1;
    }
    
    // Write header
    fprintf(fpt, "timestamp,can_id,dlc,data_hex,data_bytes\n");
    
    pthread_mutex_lock(&csv_buffer.csv_mutex);
    
    if (csv_buffer.is_full) {
        // Buffer is full, write from current position (oldest) to end
        for (int i = csv_buffer.current_line; i < MAX_CSV_LINES; i++) {
            fprintf(fpt, "%s\n", csv_buffer.lines[i]);
        }
        // Then write from beginning to current position (newest)
        for (int i = 0; i < csv_buffer.current_line; i++) {
            fprintf(fpt, "%s\n", csv_buffer.lines[i]);
        }
    } else {
        // Buffer not full yet, write from 0 to total_lines
        for (int i = 0; i < csv_buffer.total_lines; i++) {
            fprintf(fpt, "%s\n", csv_buffer.lines[i]);
        }
    }
    
    int lines_written = csv_buffer.is_full ? MAX_CSV_LINES : csv_buffer.total_lines;
    
    pthread_mutex_unlock(&csv_buffer.csv_mutex);
    
    fclose(fpt);
    
    printf("Written %d lines to %s (circular buffer: %s)\n", 
           lines_written, filename, 
           csv_buffer.is_full ? "FULL" : "FILLING");
    
    return 0;
}

// Function to copy CSV file to snapshot
int copy_csv_to_snapshot() {
    return write_csv_buffer_to_file("can_data_snapshot.csv");
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

//int parse_can_line(const char *line, CANFrame *frame) {
//    // Expected format: "SIM 19F 8 FF FF 7D 0F 38 FF 40 FE"
//    if (strncmp(line, "SIM ", 4) != 0) {
//        return 0;
//    }
//    
//    // Get timestamp
//    clock_gettime(CLOCK_REALTIME, &frame->timestamp);
//    
//    // Parse the line manually
//    char *token;
//    char *line_copy = strdup(line + 4); // Skip "SIM "
//    char *saveptr;
//    
//    // Parse CAN ID (hex)
//    token = strtok_r(line_copy, " ", &saveptr);
//    if (!token) {
//        free(line_copy);
//        return 0;
//    }
//    frame->can_id = (unsigned int)strtol(token, NULL, 16);
//    
//    // Parse DLC (decimal)
//    token = strtok_r(NULL, " ", &saveptr);
//    if (!token) {
//        free(line_copy);
//        return 0;
//    }
//    frame->dlc = (unsigned char)atoi(token);
//    
//    // Validate DLC
//    if (frame->dlc > 8) {
//        free(line_copy);
//        return 0;
//    }
//    
//    // Parse data bytes (always expect 8 hex bytes from ESP32)
//    for (int i = 0; i < 8; i++) {
//        token = strtok_r(NULL, " ", &saveptr);
//        if (!token) {
//            free(line_copy);
//            return 0;
//        }
//        
//        int byte_val = parse_hex_byte(token);
//        if (byte_val == -1) {
//            free(line_copy);
//            return 0;
//        }
//        frame->data[i] = (unsigned char)byte_val;
//    }
//    
//    free(line_copy);
//    return 1;
//}

// FIXED: Parse CAN line function - handles variable DLC correctly
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
    
    // Initialize all data bytes to 0
    memset(frame->data, 0, 8);
    
    // Parse ONLY the actual number of data bytes (based on DLC)
    for (int i = 0; i < frame->dlc; i++) {
        token = strtok_r(NULL, " ", &saveptr);
        if (!token) {
            // Not enough data bytes - this is an error
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
    
    // Check if there are extra bytes beyond DLC (which is okay, just ignore them)
    // This handles cases where the ESP32 simulator sends padded bytes
    
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
                printf("Processed %d CAN frames (CSV buffer: %s)\n", 
                       frame_count,
                       csv_buffer.is_full ? "FULL - Overwriting oldest" : "FILLING");
            }
        } else {
            printf("Failed to parse line: %s\n", line);
        }
    }
    
    printf("Serial stream ended\n");
    fclose(serial_stream);
    return NULL;
}

// --- Thread 2: CSV Writer (Modified for circular buffer) ---
void *writer_thread(void *arg) {
    printf("Writing CAN data to circular buffer (max %d lines)\n", MAX_CSV_LINES);
    
    while (1) {
        CANFrame frame = dequeue(&queue);
        
        // Check for pause request
        pthread_mutex_lock(&pause_mutex);
        if (pause_requested) {
            printf("CSV writer paused for snapshot creation...\n");
            
            // Wait until copy is completed
            while (pause_requested) {
                pthread_cond_wait(&resume_condition, &pause_mutex);
            }
            printf("CSV writer resumed.\n");
        }
        pthread_mutex_unlock(&pause_mutex);
        
        // Format CSV line
        char csv_line[256];
        char timestamp_str[64];
        snprintf(timestamp_str, sizeof(timestamp_str), "%ld.%09ld", 
                frame.timestamp.tv_sec, frame.timestamp.tv_nsec);
        
        // Build CSV line
        int pos = snprintf(csv_line, sizeof(csv_line), "%s,%03X,%d,", 
                          timestamp_str, frame.can_id, frame.dlc);
        
        // Add data as hex string
        for (int i = 0; i < frame.dlc; i++) {
            pos += snprintf(csv_line + pos, sizeof(csv_line) - pos, "%02X", frame.data[i]);
            if (i < frame.dlc - 1) {
                pos += snprintf(csv_line + pos, sizeof(csv_line) - pos, " ");
            }
        }
        pos += snprintf(csv_line + pos, sizeof(csv_line) - pos, ",");
        
        // Add data as individual bytes (same as hex string for now)
        for (int i = 0; i < frame.dlc; i++) {
            pos += snprintf(csv_line + pos, sizeof(csv_line) - pos, "%02X", frame.data[i]);
            if (i < frame.dlc - 1) {
                pos += snprintf(csv_line + pos, sizeof(csv_line) - pos, " ");
            }
        }
        
        // Add line to circular buffer
        add_csv_line(csv_line);
        
        // Periodically write the main CSV file (every 50 frames)
        static int write_counter = 0;
        write_counter++;
        if (write_counter >= 50) {
            write_csv_buffer_to_file("can_data.csv");
            write_counter = 0;
        }
    }
    
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
        printf("Creating snapshot from circular buffer...\n");
        if (copy_csv_to_snapshot() == 0) {
            printf("Snapshot created successfully.\n");
        } else {
            printf("Snapshot creation failed.\n");
        }
        
        // Also update the main CSV file
        write_csv_buffer_to_file("can_data.csv");
        
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
    
    printf("CAN Data Capture Tool with Circular Buffer CSV\n");
    printf("Serial Port: %s\n", serial_port);
    printf("Max CSV Lines: %d (circular buffer)\n", MAX_CSV_LINES);
    printf("Main CSV: can_data.csv (last %d entries)\n", MAX_CSV_LINES);
    printf("Snapshot CSV: can_data_snapshot.csv (updated every %d seconds)\n\n", COPY_INTERVAL_SECONDS);
    
    init_queue(&queue);
    init_csv_buffer();
    
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
