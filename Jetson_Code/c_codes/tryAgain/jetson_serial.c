//and now ?
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
#define MAX_LINE_LENGTH 128
// Struct for CAN data from ESP32(2)
typedef struct {
    unsigned int can_id;
    unsigned char dlc;
    unsigned char data[8];
} CANFrame;
// Struct for ESP-NOW data from ESP32(1)
typedef struct {
    unsigned short SOC;
    float speedKmh;
    float odometerKm;
    float displaySpeed;
    uint8_t MacAddress[6];
} Item;
// Global serial port file descriptors
int serial_port1; // /dev/ttyUSB1 for ESP32(1)
int serial_port2; // /dev/ttyUSB2 for ESP32(2)
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
// Utility functions
void print_item(const Item *item) {
    printf("[ESP-NOW] MAC: %02X:%02X:%02X:%02X:%02X:%02X | SOC: %u | Speed: %.1f km/h | Display: %.1f km/h | Odo: %.1f km\n",
        item->MacAddress[0], item->MacAddress[1], item->MacAddress[2],
        item->MacAddress[3], item->MacAddress[4], item->MacAddress[5],
        item->SOC, item->speedKmh, item->displaySpeed, item->odometerKm);
}

int parse_can_line(const char *line, CANFrame *frame) {
    // REQUIRE "SIM " prefix (like the working code)
    if (strncmp(line, "SIM ", 4) != 0)
        return 0;
    
    int id, dlc;
    unsigned int d[8] = {0};
    int parsed = sscanf(line + 4, "%x %hhu %x %x %x %x %x %x %x %x",
                        &id, &dlc, &d[0], &d[1], &d[2], &d[3],
                        &d[4], &d[5], &d[6], &d[7]);
    if (parsed < 2 + dlc)
        return 0;
    frame->can_id = id;
    frame->dlc = dlc;
    for (int i = 0; i < dlc; ++i)
        frame->data[i] = (unsigned char)d[i];
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
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= CREAD | CLOCAL;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}
// Thread functions for ESP-NOW (binary data)
void* receiver_thread(void* arg) {
    uint8_t header[2];
    uint8_t buffer[sizeof(Item)];
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
            if (enqueue(&item) != 0) {
                fprintf(stderr, "ESP-NOW queue full, dropping item.\n");
            }
        }
    }
    return NULL;
}
//void* printer_thread(void* arg) {
//    while (1) {
//        Item item;
//        dequeue(&item);
//        print_item(&item);
//    }
//    return NULL;
//}
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
//void* sender_thread(void* arg) {
//    while (1) {
//        Item item = {
//            .SOC = 85,
//            .speedKmh = 42.0,
//            .odometerKm = 1234.5,
//            .displaySpeed = 41.8,
//            .MacAddress = {0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01}
//        };
//        uint8_t header[2] = { HEADER_BYTE_1, HEADER_BYTE_2 };
//        write(serial_port1, header, 2);
//        write(serial_port1, &item, sizeof(Item));
//        usleep(5000000); // 5 seconds
//    }
//    return NULL;
//}

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

// Thread functions for CAN (text data)
void *reader_thread(void *arg) {
    int fd = setup_serial_text("/dev/ttyUSB2");
    if (fd < 0) {
        fprintf(stderr, "Failed to open CAN serial port\n");
        return NULL;
    }

    FILE *serial_stream = fdopen(fd, "r");
    if (!serial_stream) {
        perror("fdopen failed for CAN port");
        close(fd);
        return NULL;
    }

    char line[MAX_LINE_LENGTH];
    while (fgets(line, sizeof(line), serial_stream)) {
        CANFrame frame;
        if (parse_can_line(line, &frame)) {
            enqueueCAN(&queueCAN, frame);
            printf("[CAN] %03X %d", frame.can_id, frame.dlc);
            for (int i = 0; i < frame.dlc; ++i)
                printf(" %02X", frame.data[i]);
            printf("\n");
        }
    }
    fclose(serial_stream);
    return NULL;
}
void *writer_thread(void *arg) {
    FILE *fpt = fopen("CANData.csv", "w+");
    if (!fpt) {
        perror("Failed to open CSV file");
        return NULL;
    }
    fprintf(fpt, "can_id,dlc,data\n");
    fflush(fpt);

    while (1) {
        CANFrame frame = dequeueCAN(&queueCAN);
        fprintf(fpt, "%03X,%d,", frame.can_id, frame.dlc);
        for (int i = 0; i < frame.dlc; ++i) {
            fprintf(fpt, "%02X", frame.data[i]);
            if (i < frame.dlc - 1) fprintf(fpt, " ");
        }
        fprintf(fpt, "\n");
        fflush(fpt);
    }
    fclose(fpt);
    return NULL;
}
// Add error handling in main():
int main() {
    // Setup serial ports with error handling
    serial_port1 = setup_serial_binary("/dev/ttyUSB1");
    if (serial_port1 < 0) {
        fprintf(stderr, "Failed to setup ESP-NOW serial port\n");
        return 1;
    }

    // Test CAN port availability
    int test_can_port = setup_serial_text("/dev/ttyUSB2");
    int can_available = (test_can_port >= 0);
    if (test_can_port >= 0) close(test_can_port);

    if (!can_available) {
        printf("Warning: CAN port not available, running ESP-NOW only\n");
    }

    init_queueCAN(&queueCAN);

    pthread_t rx_tid, respense_tid, print_tid, reader_tid, writer_tid;

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

