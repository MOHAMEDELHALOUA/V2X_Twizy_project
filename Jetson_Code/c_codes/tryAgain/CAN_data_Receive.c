#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#define MAX_LINE_LENGTH 128

typedef struct {
    unsigned int can_id;
    unsigned char dlc;
    unsigned char data[8];
} CANFrame;

int parse_can_line(const char *line, CANFrame *frame) {
    if (strncmp(line, "SIM ", 4) != 0)
        return 0;

    int id, dlc;
    unsigned int d[8] = {0};
    int parsed = sscanf(line + 4, "%x %hhu %x %x %x %x %x %x %x %x",
                        &id, &dlc,
                        &d[0], &d[1], &d[2], &d[3],
                        &d[4], &d[5], &d[6], &d[7]);

    if (parsed < 2 + dlc)
        return 0;

    frame->can_id = id;
    frame->dlc = dlc;
    for (int i = 0; i < dlc; ++i)
        frame->data[i] = (unsigned char)d[i];

    return 1;
}

int open_serial_port(const char *device) {
    int fd = open(device, O_RDONLY | O_NOCTTY);
    if (fd == -1) {
        perror("Unable to open serial port");
        exit(EXIT_FAILURE);
    }

    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag &= ~PARENB; // no parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;     // 8 bits
    options.c_cflag |= CREAD | CLOCAL;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input
    options.c_oflag &= ~OPOST;  // raw output

    tcsetattr(fd, TCSANOW, &options);

    return fd;
}
int main(int argc, char *argv[]) {
    const char *serial_port2 = (argc >= 2) ? argv[1] : "/dev/ttyUSB1";
    int fd = open_serial_port(serial_port2);
    char line[MAX_LINE_LENGTH];
    FILE *serial_stream = fdopen(fd, "r");
    
    // Check if serial_stream creation failed
    if (!serial_stream) {
        perror("fdopen failed");
        close(fd);
        return EXIT_FAILURE;
    }
    
    // Check if CSV file opening failed
    FILE *fpt = fopen("MyFile.csv", "w+");
    if (!fpt) {
        perror("Failed to open CSV file");
        fclose(serial_stream);
        return EXIT_FAILURE;
    }
    
    fprintf(fpt, "can_id,dlc,data\n");  
    
    while (fgets(line, sizeof(line), serial_stream)) {
        CANFrame frame;
        if (parse_can_line(line, &frame)) {
            printf("%03X %d", frame.can_id, frame.dlc);
            for (int i = 0; i < frame.dlc; ++i)
                printf(" %02X", frame.data[i]);
            printf("\n");
            
            // CSV output - only write if parsing succeeded
            fprintf(fpt, "%03X,%d,", frame.can_id, frame.dlc);
            for (int i = 0; i < frame.dlc; ++i) {
                fprintf(fpt, "%02X", frame.data[i]);
                if (i < frame.dlc - 1) fprintf(fpt, " ");  // Space between bytes
            }
            fprintf(fpt, "\n");
            fflush(fpt);  // Force write to disk immediately
        }
    }
    
    fclose(fpt);
    fclose(serial_stream);
    return 0;
}
