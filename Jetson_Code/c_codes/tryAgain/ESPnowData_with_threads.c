///cleaned version
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

typedef struct {
    unsigned short SOC;
    float speedKmh;
    float odometerKm;
    float displaySpeed;
    uint8_t MacAddress[6];
} Item;

int serial_port;
pthread_mutex_t queue_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t queue_cond = PTHREAD_COND_INITIALIZER;
Item queue[QUEUE_SIZE];
int queue_head = 0;
int queue_tail = 0;
int queue_count = 0;

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
    serial_port = setup_serial("/dev/ttyUSB0");
    if (serial_port < 0) return 1;
    
    pthread_t rx_tid, tx_tid, print_tid;
    pthread_create(&rx_tid, NULL, receiver_thread, NULL);
    pthread_create(&tx_tid, NULL, sender_thread, NULL);
    pthread_create(&print_tid, NULL, printer_thread, NULL);
    
    pthread_join(rx_tid, NULL);
    pthread_join(tx_tid, NULL);
    pthread_join(print_tid, NULL);
    
    close(serial_port);
    return 0;
}
