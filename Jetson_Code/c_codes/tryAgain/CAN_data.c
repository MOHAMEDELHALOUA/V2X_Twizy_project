#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#define SERIAL_PORT "/dev/ttyUSB1"  // change to your device, e.g. /dev/ttyAMA0

int main() {
    int serial_port = open(SERIAL_PORT, O_RDONLY | O_NOCTTY);
    if (serial_port < 0) {
        perror("Error opening serial port");
        return 1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_port, &tty) != 0) {
        perror("Error from tcgetattr");
        close(serial_port);
        return 1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // One stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS;// No flow control
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON; // Disable canonical mode
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow ctrl
    tty.c_iflag &= ~(ICRNL | INLCR);
    tty.c_oflag &= ~OPOST; // Raw output

    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        perror("Error from tcsetattr");
        close(serial_port);
        return 1;
    }

    // Read line-by-line and print
    char buffer[256];
    int buffer_index = 0;
    char byte;

    while (1) {
        int n = read(serial_port, &byte, 1);
        if (n > 0) {
            if (byte == '\n' || buffer_index >= sizeof(buffer) - 1) {
                buffer[buffer_index] = '\0';
                printf("%s\n", buffer);
                buffer_index = 0;
            } else if (byte != '\r') {
                buffer[buffer_index++] = byte;
            }
        }
    }

    close(serial_port);
    return 0;
}


