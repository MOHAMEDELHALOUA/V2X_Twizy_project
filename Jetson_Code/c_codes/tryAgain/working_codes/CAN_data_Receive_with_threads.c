// Complete CAN_data_Receive_with_threads.c - Enhanced with GPS support
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>
#include <time.h>

#define MAX_LINE_LENGTH 512
#define QUEUE_SIZE 1000
#define COPY_INTERVAL_SECONDS 4
#define MAX_CSV_LINES 1000  // Maximum lines in CSV before overwriting

typedef struct {
    unsigned int can_id;
    unsigned char dlc;
    unsigned char data[8];
    struct timespec timestamp;
} CANFrame;

// GPS data structure
typedef struct {
    char timestamp[64];
    float latitude;
    float longitude;
    float altitude;
    float speed_kmh;
    int satellites;
    float hdop;
    char datetime[64];
    int valid;
    struct timespec system_timestamp;
} GPSFrame;

// --- Queue implementation ---
typedef struct {
    CANFrame can_items[QUEUE_SIZE];
    GPSFrame gps_items[QUEUE_SIZE];
    int can_front, can_rear, can_count;
    int gps_front, gps_rear, gps_count;
    pthread_mutex_t lock;
    pthread_cond_t not_empty;
} DataQueue;

DataQueue queue;

// Global variables for pause/copy functionality
volatile int pause_requested = 0;
pthread_mutex_t pause_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t resume_condition = PTHREAD_COND_INITIALIZER;

// Circular buffer management for each data type
typedef struct {
    char can_lines[MAX_CSV_LINES][256];
    char gps_lines[MAX_CSV_LINES][256];
    int can_current_line, can_total_lines, can_is_full;
    int gps_current_line, gps_total_lines, gps_is_full;
    pthread_mutex_t csv_mutex;
} CSVBuffer;

CSVBuffer csv_buffer = {0};

// Latest GPS data for sharing with other modules
GPSFrame latest_gps = {0};
pthread_mutex_t gps_data_mutex = PTHREAD_MUTEX_INITIALIZER;

void init_csv_buffer() {
    memset(&csv_buffer, 0, sizeof(csv_buffer));
    pthread_mutex_init(&csv_buffer.csv_mutex, NULL);
}

void init_queue(DataQueue *q) {
    memset(q, 0, sizeof(DataQueue));
    pthread_mutex_init(&q->lock, NULL);
    pthread_cond_init(&q->not_empty, NULL);
}

void enqueue_can(DataQueue *q, CANFrame item) {
    pthread_mutex_lock(&q->lock);
    if (q->can_count == QUEUE_SIZE) {
        q->can_front = (q->can_front + 1) % QUEUE_SIZE;
        q->can_count--;
    }
    q->can_items[q->can_rear] = item;
    q->can_rear = (q->can_rear + 1) % QUEUE_SIZE;
    q->can_count++;
    pthread_cond_signal(&q->not_empty);
    pthread_mutex_unlock(&q->lock);
}

void enqueue_gps(DataQueue *q, GPSFrame item) {
    pthread_mutex_lock(&q->lock);
    if (q->gps_count == QUEUE_SIZE) {
        q->gps_front = (q->gps_front + 1) % QUEUE_SIZE;
        q->gps_count--;
    }
    q->gps_items[q->gps_rear] = item;
    q->gps_rear = (q->gps_rear + 1) % QUEUE_SIZE;
    q->gps_count++;
    
    // Update latest GPS data for other modules
    pthread_mutex_lock(&gps_data_mutex);
    latest_gps = item;
    pthread_mutex_unlock(&gps_data_mutex);
    
    pthread_cond_signal(&q->not_empty);
    pthread_mutex_unlock(&q->lock);
}

// Add GPS line to circular buffer
void add_gps_csv_line(const char *line) {
    pthread_mutex_lock(&csv_buffer.csv_mutex);
    
    strncpy(csv_buffer.gps_lines[csv_buffer.gps_current_line], line, 255);
    csv_buffer.gps_lines[csv_buffer.gps_current_line][255] = '\0';
    
    csv_buffer.gps_current_line = (csv_buffer.gps_current_line + 1) % MAX_CSV_LINES;
    
    if (!csv_buffer.gps_is_full) {
        csv_buffer.gps_total_lines++;
        if (csv_buffer.gps_total_lines >= MAX_CSV_LINES) {
            csv_buffer.gps_is_full = 1;
        }
    }
    
    pthread_mutex_unlock(&csv_buffer.csv_mutex);
}

// Add CAN line to circular buffer
void add_can_csv_line(const char *line) {
    pthread_mutex_lock(&csv_buffer.csv_mutex);
    
    strncpy(csv_buffer.can_lines[csv_buffer.can_current_line], line, 255);
    csv_buffer.can_lines[csv_buffer.can_current_line][255] = '\0';
    
    csv_buffer.can_current_line = (csv_buffer.can_current_line + 1) % MAX_CSV_LINES;
    
    if (!csv_buffer.can_is_full) {
        csv_buffer.can_total_lines++;
        if (csv_buffer.can_total_lines >= MAX_CSV_LINES) {
            csv_buffer.can_is_full = 1;
        }
    }
    
    pthread_mutex_unlock(&csv_buffer.csv_mutex);
}

// Write GPS buffer to file
int write_gps_buffer_to_file(const char *filename) {
    FILE *fpt = fopen(filename, "w");
    if (!fpt) {
        perror("Failed to open GPS file for writing");
        return -1;
    }
    
    fprintf(fpt, "timestamp,latitude,longitude,altitude,speed_kmh,satellites,hdop,datetime,valid\n");
    pthread_mutex_lock(&csv_buffer.csv_mutex);
    
    if (csv_buffer.gps_is_full) {
        for (int i = csv_buffer.gps_current_line; i < MAX_CSV_LINES; i++) {
            fprintf(fpt, "%s\n", csv_buffer.gps_lines[i]);
        }
        for (int i = 0; i < csv_buffer.gps_current_line; i++) {
            fprintf(fpt, "%s\n", csv_buffer.gps_lines[i]);
        }
    } else {
        for (int i = 0; i < csv_buffer.gps_total_lines; i++) {
            fprintf(fpt, "%s\n", csv_buffer.gps_lines[i]);
        }
    }
    
    int lines_written = csv_buffer.gps_is_full ? MAX_CSV_LINES : csv_buffer.gps_total_lines;
    pthread_mutex_unlock(&csv_buffer.csv_mutex);
    
    fclose(fpt);
    printf("Written %d GPS lines to %s\n", lines_written, filename);
    return 0;
}

// Write CAN buffer to file with GPS data included
int write_can_buffer_to_file(const char *filename) {
    FILE *fpt = fopen(filename, "w");
    if (!fpt) {
        perror("Failed to open CAN file for writing");
        return -1;
    }
    
    // Enhanced header with GPS data
    fprintf(fpt, "timestamp,can_id,dlc,data_hex,data_bytes,gps_lat,gps_lon,gps_alt,gps_speed,gps_valid\n");
    
    pthread_mutex_lock(&csv_buffer.csv_mutex);
    
    // Get current GPS data
    pthread_mutex_lock(&gps_data_mutex);
    GPSFrame current_gps = latest_gps;
    pthread_mutex_unlock(&gps_data_mutex);
    
    if (csv_buffer.can_is_full) {
        for (int i = csv_buffer.can_current_line; i < MAX_CSV_LINES; i++) {
            fprintf(fpt, "%s,%.6f,%.6f,%.1f,%.1f,%d\n", 
                   csv_buffer.can_lines[i],
                   current_gps.latitude, current_gps.longitude, 
                   current_gps.altitude, current_gps.speed_kmh, current_gps.valid);
        }
        for (int i = 0; i < csv_buffer.can_current_line; i++) {
            fprintf(fpt, "%s,%.6f,%.6f,%.1f,%.1f,%d\n", 
                   csv_buffer.can_lines[i],
                   current_gps.latitude, current_gps.longitude, 
                   current_gps.altitude, current_gps.speed_kmh, current_gps.valid);
        }
    } else {
        for (int i = 0; i < csv_buffer.can_total_lines; i++) {
            fprintf(fpt, "%s,%.6f,%.6f,%.1f,%.1f,%d\n", 
                   csv_buffer.can_lines[i],
                   current_gps.latitude, current_gps.longitude, 
                   current_gps.altitude, current_gps.speed_kmh, current_gps.valid);
        }
    }
    
    int lines_written = csv_buffer.can_is_full ? MAX_CSV_LINES : csv_buffer.can_total_lines;
    
    pthread_mutex_unlock(&csv_buffer.csv_mutex);
    
    fclose(fpt);
    
    printf("Written %d CAN lines with GPS data to %s\n", lines_written, filename);
    
    return 0;
}

// Function to copy CSV file to snapshot
int copy_csv_to_snapshot() {
    int result1 = write_can_buffer_to_file("can_data_snapshot.csv");
    int result2 = write_gps_buffer_to_file("gps_data_snapshot.csv");
    return (result1 == 0 && result2 == 0) ? 0 : -1;
}

// --- Serial Setup & Data Parsing ---
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

// Parse CAN line
int parse_can_line(const char *line, CANFrame *frame) {
    // Expected format: "SIM 19F 8 FF FF 7D 0F 38 FF 40 FE"
    if (strncmp(line, "SIM ", 4) != 0) {
        return 0;
    }
    
    clock_gettime(CLOCK_REALTIME, &frame->timestamp);
    
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
    
    // Parse actual data bytes (based on DLC)
    for (int i = 0; i < frame->dlc; i++) {
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

// Parse GPS line
int parse_gps_line(const char *line, GPSFrame *frame) {
    // Expected format: "GPS <LAT> <LON> <ALT> <SPEED> <SATS> <HDOP> <DATETIME> <VALID>"
    if (strncmp(line, "GPS ", 4) != 0) {
        return 0;
    }
    
    clock_gettime(CLOCK_REALTIME, &frame->system_timestamp);
    
    char *token;
    char *line_copy = strdup(line + 4); // Skip "GPS "
    char *saveptr;
    int field = 0;
    
    token = strtok_r(line_copy, " ", &saveptr);
    while (token != NULL && field < 8) {
        switch (field) {
            case 0: // latitude
                frame->latitude = atof(token);
                break;
            case 1: // longitude
                frame->longitude = atof(token);
                break;
            case 2: // altitude
                frame->altitude = atof(token);
                break;
            case 3: // speed
                frame->speed_kmh = atof(token);
                break;
            case 4: // satellites
                frame->satellites = atoi(token);
                break;
            case 5: // hdop
                frame->hdop = atof(token);
                break;
            case 6: // datetime
                strncpy(frame->datetime, token, sizeof(frame->datetime) - 1);
                frame->datetime[sizeof(frame->datetime) - 1] = '\0';
                break;
            case 7: // valid
                frame->valid = atoi(token);
                break;
        }
        field++;
        token = strtok_r(NULL, " ", &saveptr);
    }
    
    free(line_copy);
    
    // Create timestamp string
    snprintf(frame->timestamp, sizeof(frame->timestamp), "%ld.%09ld", 
             frame->system_timestamp.tv_sec, frame->system_timestamp.tv_nsec);
    
    return (field >= 7) ? 1 : 0; // Need at least 7 fields
}

// --- Thread 1: Serial Reader ---
void *reader_thread(void *arg) {
    const char *port = (const char *)arg;
    int fd = open_serial_port(port);
    
    printf("Reading CAN + GPS data from serial port: %s\n", port);
    printf("Waiting for data...\n");
    
    FILE *serial_stream = fdopen(fd, "r");
    if (!serial_stream) {
        perror("fdopen failed");
        close(fd);
        return NULL;
    }
    
    char line[MAX_LINE_LENGTH];
    int can_frame_count = 0;
    int gps_frame_count = 0;
    
    while (fgets(line, sizeof(line), serial_stream)) {
        // Check for pause request
        pthread_mutex_lock(&pause_mutex);
        if (pause_requested) {
            printf("Data collection paused for snapshot creation...\n");
            while (pause_requested) {
                pthread_cond_wait(&resume_condition, &pause_mutex);
            }
            printf("Data collection resumed.\n");
        }
        pthread_mutex_unlock(&pause_mutex);
        
        // Remove newline
        line[strcspn(line, "\r\n")] = 0;
        
        // Skip empty lines and status messages
        if (strlen(line) == 0 || 
            strstr(line, "CAN + GPS Data Sniffer") || 
            strstr(line, "Ready") || 
            strstr(line, "===") ||
            strstr(line, "Format:") ||
            strstr(line, "Sending")) {
            continue;
        }
        
        // Try to parse as CAN data
        CANFrame can_frame;
        if (parse_can_line(line, &can_frame)) {
            enqueue_can(&queue, can_frame);
            can_frame_count++;
            
            if (can_frame_count % 100 == 0) {
                printf("Processed %d CAN frames\n", can_frame_count);
            }
            continue;
        }
        
        // Try to parse as GPS data
        GPSFrame gps_frame;
        if (parse_gps_line(line, &gps_frame)) {
            enqueue_gps(&queue, gps_frame);
            gps_frame_count++;
            
            if (gps_frame_count % 10 == 0) {
                printf("Processed %d GPS updates (Lat:%.4f, Lon:%.4f, Sats:%d)\n", 
                       gps_frame_count, gps_frame.latitude, gps_frame.longitude, gps_frame.satellites);
            }
            continue;
        }
        
        // Handle STATUS messages
        if (strncmp(line, "STATUS ", 7) == 0) {
            printf("ESP32 Status: %s\n", line + 7);
            continue;
        }
        
        // Unknown line format
        if (strlen(line) > 5) {  // Only log non-trivial unknown lines
            printf("Unknown line format: %s\n", line);
        }
    }
    
    printf("Serial stream ended\n");
    fclose(serial_stream);
    return NULL;
}

// --- Thread 2: CAN CSV Writer ---
void *can_writer_thread(void *arg) {
    printf("Writing CAN data to circular buffer\n");
    
    while (1) {
        pthread_mutex_lock(&queue.lock);
        while (queue.can_count == 0) {
            pthread_cond_wait(&queue.not_empty, &queue.lock);
        }
        
        CANFrame frame = queue.can_items[queue.can_front];
        queue.can_front = (queue.can_front + 1) % QUEUE_SIZE;
        queue.can_count--;
        pthread_mutex_unlock(&queue.lock);
        
        // Check for pause request
        pthread_mutex_lock(&pause_mutex);
        if (pause_requested) {
            while (pause_requested) {
                pthread_cond_wait(&resume_condition, &pause_mutex);
            }
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
        add_can_csv_line(csv_line);
    }
    
    return NULL;
}

// --- Thread 3: GPS CSV Writer ---
void *gps_writer_thread(void *arg) {
    printf("Writing GPS data to circular buffer\n");
    
    while (1) {
        pthread_mutex_lock(&queue.lock);
        while (queue.gps_count == 0) {
            pthread_cond_wait(&queue.not_empty, &queue.lock);
        }
        
        GPSFrame frame = queue.gps_items[queue.gps_front];
        queue.gps_front = (queue.gps_front + 1) % QUEUE_SIZE;
        queue.gps_count--;
        pthread_mutex_unlock(&queue.lock);
        
        // Check for pause request
        pthread_mutex_lock(&pause_mutex);
        if (pause_requested) {
            while (pause_requested) {
                pthread_cond_wait(&resume_condition, &pause_mutex);
            }
        }
        pthread_mutex_unlock(&pause_mutex);
        
        // Format GPS CSV line
        char csv_line[256];
        snprintf(csv_line, sizeof(csv_line), 
                "%s,%.6f,%.6f,%.1f,%.1f,%d,%.2f,%s,%d",
                frame.timestamp, frame.latitude, frame.longitude, 
                frame.altitude, frame.speed_kmh, frame.satellites, 
                frame.hdop, frame.datetime, frame.valid);
        
        // Add line to circular buffer
        add_gps_csv_line(csv_line);
    }
    
    return NULL;
}

// --- Thread 4: Automatic Copy Handler ---
void *copy_handler_thread(void *arg) {
    printf("Automatic snapshot creation every %d seconds started\n", COPY_INTERVAL_SECONDS);
    
    while (1) {
        // Wait for the specified interval
        sleep(COPY_INTERVAL_SECONDS);
        
        printf("\n--- Creating snapshot (pausing data collection) ---\n");
        
        // Request pause
        pthread_mutex_lock(&pause_mutex);
        pause_requested = 1;
        pthread_mutex_unlock(&pause_mutex);
        
        // Wait 1 second to ensure all threads are paused
        sleep(1);
        
        // Perform the copy
        printf("Creating snapshots from circular buffers...\n");
        if (copy_csv_to_snapshot() == 0) {
            printf("Snapshots created successfully.\n");
        } else {
            printf("Snapshot creation failed.\n");
        }
        
        // Also update the main CSV files
        write_can_buffer_to_file("can_data.csv");
        write_gps_buffer_to_file("gps_data.csv");
        
        // Print current GPS status
        pthread_mutex_lock(&gps_data_mutex);
        if (latest_gps.valid) {
            printf("Current GPS: Lat=%.6f, Lon=%.6f, Alt=%.1fm, Speed=%.1fkm/h, Sats=%d\n",
                   latest_gps.latitude, latest_gps.longitude, latest_gps.altitude,
                   latest_gps.speed_kmh, latest_gps.satellites);
        } else {
            printf("Current GPS: No valid GPS data\n");
        }
        pthread_mutex_unlock(&gps_data_mutex);
        
        // Resume operations
        pthread_mutex_lock(&pause_mutex);
        pause_requested = 0;
        pthread_cond_broadcast(&resume_condition);
        pthread_mutex_unlock(&pause_mutex);
        
        printf("--- Snapshot complete, resuming data collection ---\n\n");
    }
    
    return NULL;
}

// Get latest GPS data (for use by other modules)
GPSFrame get_latest_gps_data() {
    pthread_mutex_lock(&gps_data_mutex);
    GPSFrame gps_copy = latest_gps;
    pthread_mutex_unlock(&gps_data_mutex);
    return gps_copy;
}

// --- Main ---
int main(int argc, char *argv[]) {
    const char *serial_port = (argc >= 2) ? argv[1] : "/dev/ttyUSB0";
    
    printf("Enhanced CAN + GPS Data Capture Tool\n");
    printf("====================================\n");
    printf("Serial Port: %s\n", serial_port);
    printf("Max CSV Lines: %d (circular buffer)\n", MAX_CSV_LINES);
    printf("Output Files:\n");
    printf("  - can_data.csv (CAN data with GPS coordinates)\n");
    printf("  - gps_data.csv (GPS data only)\n");
    printf("  - can_data_snapshot.csv (updated every %d seconds)\n", COPY_INTERVAL_SECONDS);
    printf("  - gps_data_snapshot.csv (updated every %d seconds)\n", COPY_INTERVAL_SECONDS);
    printf("Expected Input Formats:\n");
    printf("  - CAN: SIM <ID> <DLC> <DATA_BYTES>\n");
    printf("  - GPS: GPS <LAT> <LON> <ALT> <SPEED> <SATS> <HDOP> <DATETIME> <VALID>\n");
    printf("====================================\n\n");
    
    init_queue(&queue);
    init_csv_buffer();
    
    pthread_t reader_tid, can_writer_tid, gps_writer_tid, copy_handler_tid;
    
    // Start threads
    pthread_create(&reader_tid, NULL, reader_thread, (void *)serial_port);
    pthread_create(&can_writer_tid, NULL, can_writer_thread, NULL);
    pthread_create(&gps_writer_tid, NULL, gps_writer_thread, NULL);
    pthread_create(&copy_handler_tid, NULL, copy_handler_thread, NULL);
    
    printf("System started - monitoring CAN and GPS data\n");
    printf("GPS data will be included in CAN CSV files\n");
    printf("Separate GPS CSV file also created\n");
    printf("Circular buffer behavior: overwrites after %d lines\n\n", MAX_CSV_LINES);
    
    // Wait for reader thread (main control)
    pthread_join(reader_tid, NULL);
    
    return 0;
}
