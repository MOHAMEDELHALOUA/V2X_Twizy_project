#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <pthread.h>

#define MAX_LINE_LENGTH 128
#define QUEUE_SIZE 100

typedef struct {
    unsigned int can_id;
    unsigned char dlc;
    unsigned char data[8];
} CANFrame;

// --- Queue implementation ---
typedef struct {
    CANFrame items[QUEUE_SIZE];
    int front, rear, count;
    pthread_mutex_t lock;
    pthread_cond_t not_empty;
} CANQueue;

CANQueue queue;

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

// --- Serial Setup & CAN Parsing ---
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

int parse_can_line(const char *line, CANFrame *frame) {
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

// --- Thread 1: Serial Reader ---
void *reader_thread(void *arg) {
    const char *port = (const char *)arg;
    int fd = open_serial_port(port);
    FILE *serial_stream = fdopen(fd, "r");
    if (!serial_stream) {
        perror("fdopen failed");
        close(fd);
        return NULL;
    }

    char line[MAX_LINE_LENGTH];
    while (fgets(line, sizeof(line), serial_stream)) {
        CANFrame frame;
        if (parse_can_line(line, &frame)) {
            enqueue(&queue, frame);
        }
    }

    fclose(serial_stream);
    return NULL;
}

// --- Thread 2: CSV Writer ---
void *writer_thread(void *arg) {
    FILE *fpt = fopen("MyFile.csv", "w+");
    if (!fpt) {
        perror("Failed to open CSV file");
        return NULL;
    }

    fprintf(fpt, "can_id,dlc,data\n");
    fflush(fpt);

    while (1) {
        CANFrame frame = dequeue(&queue);
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

// --- Main ---
int main(int argc, char *argv[]) {
    const char *serial_port = (argc >= 2) ? argv[1] : "/dev/ttyUSB1";
    init_queue(&queue);

    pthread_t reader_tid, writer_tid;
    pthread_create(&reader_tid, NULL, reader_thread, (void *)serial_port);
    pthread_create(&writer_tid, NULL, writer_thread, NULL);

    pthread_join(reader_tid, NULL);
    pthread_join(writer_tid, NULL);

    return 0;
}

