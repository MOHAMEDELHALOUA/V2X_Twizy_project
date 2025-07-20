#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#define HEADER_SIZE 2
#define HEADER_BYTE_1 0xAA
#define HEADER_BYTE_2 0x55


// Struct matching ESP32's struct layout (packed)
typedef struct  {
    unsigned short SOC;        // 2 bytes
    float speedKmh;           // 4 bytes
    float odometerKm;         // 4 bytes
    float displaySpeed;       // 4 bytes
    uint8_t MacAddress[6];    // 6 bytes
} Item;


uint8_t buffer[sizeof(Item)];

void print_item(const Item* item) {
    printf("<Received from MAC: %02X:%02X:%02X:%02X:%02X:%02X | SOC: %u | Speed: %.1f km/h | Display: %.1f km/h | Odometer: %.1f km>\n",
        item->MacAddress[0], item->MacAddress[1], item->MacAddress[2],
        item->MacAddress[3], item->MacAddress[4], item->MacAddress[5],
        item->SOC, item->speedKmh, item->displaySpeed, item->odometerKm
    );
}

time_t last_received = time(NULL);

int main() {
    const char *serial_port_path = "/dev/ttyUSB0";  // Use actual port (e.g., ttyUSB0 or ttyUSB0)
    int serial_port = open(serial_port_path, O_RDWR | O_NOCTTY);

    if (serial_port < 0) {
        perror("Error opening serial port");
        return 1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(serial_port, &tty) != 0) {
        perror("Error from tcgetattr");
        return 1;
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
    tty.c_cc[VMIN] = 1;  // Block until entire struct is received
    tty.c_cc[VTIME] = 1;

    tcsetattr(serial_port, TCSANOW, &tty);
    while (1) {

        if (difftime(time(NULL), last_received) > 10) {
            printf("No unit nearby...\n");
            last_received = time(NULL); // Avoid repeating too often
        }
        uint8_t b;
        // Wait for header
        while (read(serial_port, &b, 1) == 1) {
            if (b == HEADER_BYTE_1) {
                if (read(serial_port, &b, 1) == 1 && b == HEADER_BYTE_2) {
                    // Found header, now read full Item
                    int bytes_read = 0;
                    while (bytes_read < sizeof(Item)) {
                        int result = read(serial_port, buffer + bytes_read, sizeof(Item) - bytes_read);
                        if (result > 0) {
                            bytes_read += result;
                        } else {
                            perror("Read error during struct");
                            break;
                        }
                    }

                    if (bytes_read == sizeof(Item)) {
                        last_received = time(NULL);
                        Item *item = (Item *)buffer;
                        print_item(item);
                    } else {
                        fprintf(stderr, "Incomplete Item received\n");
                    }
                }
            }
        }
    }
    close(serial_port);
    return 0;
}
