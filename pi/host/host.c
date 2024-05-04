#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/stat.h>

#include "comms.h"

#define SERIAL_PORT "/dev/serial0"

#define SEN_DAT_FIFO "/tmp/sensor_data_fifo"
// #define SERIAL_PORT "/dev/ttyAMA1"

struct
{
    char *name;
    char *path;
    bool isMaster;
} progs[] = {
    {"Time Tick", "../progs/timeTick/time.py", false},
    {"MQTT Ctrl Test", "../progs/mqttCtrl/main.py", true}};

bool writeCmd(int fd, uint8_t cmd, uint8_t *payload, uint16_t payloadLen);
void replyOk(int fd);
int getBytes(int fd, uint8_t *buf, size_t count);
int pktsBet = 0;


typedef struct {
    long long timeMs;
    senData_t senData;
} senProgData_t;

int main(int argc, char **argv)
{
    printf("Pi host v0.1\n\n");

    time_t startTime = time(NULL);

    uint8_t totProgs = sizeof(progs) / sizeof(progs[0]);

    for (uint8_t i = 0; i < totProgs; i++)
    {
        printf("programs[%d]:\n", i + 1);
        printf("\tname: %s\n", progs[i].name);
        printf("\tpath: %s\n", progs[i].path);
        printf("\tisMaster: %s\n", progs[i].isMaster ? "true" : "false");
        printf("\n");
    }

    int serialFd;
    if ((serialFd = open(SERIAL_PORT, O_RDWR | O_NOCTTY)) == -1)
    {
        perror("Failed to open serial port");
        return 1;
    }

    struct termios options;
    tcgetattr(serialFd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag &= ~CSIZE;           // Mask character size bits
    options.c_cflag &= ~CSTOPB;          // Clear stop field
    options.c_cflag |= CS8;              // 8 data bits
    // options.c_cflag |= CSTOPB;  // Set two stop bits
    options.c_cflag &= ~PARENB;          // No parity
    options.c_cflag &= ~CRTSCTS;         // Disable hardware flow control
    options.c_lflag &= ~ECHO;            // Disable echo
    options.c_lflag &= ~ICANON;          // Disable canonical input mode (line oriented)
    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver
    options.c_cc[VTIME] = 10;
    options.c_cc[VMIN] = 0;
    tcsetattr(serialFd, TCSANOW, &options);

    // Flush input buffer
    if (tcflush(serialFd, TCIFLUSH) == -1)
    {
        perror("Error flushing input buffer");
        close(serialFd);
        return EXIT_FAILURE;
    }

    // Flush output buffer
    if (tcflush(serialFd, TCOFLUSH) == -1)
    {
        perror("Error flushing output buffer");
        close(serialFd);
        return EXIT_FAILURE;
    }

    printf("Opened serial port\n");

    if (mkfifo(SEN_DAT_FIFO, 0666) == -1) {
        if (errno !=EEXIST) {
            perror("Failed to create FIFO");
            return EXIT_FAILURE;
        }
    }
    int senDataFd;
    
    if ((senDataFd = open(SEN_DAT_FIFO, O_RDWR | O_NONBLOCK)) == -1) {
        perror("Failed to open sensor data fifo");
        return EXIT_FAILURE;
    }


    if (!writeCmd(serialFd, SBC_TX_EVT_RUNNING, NULL, 0))
    {
        fprintf(stderr, "Error writing EVT_RUNNING\n");
        return EXIT_FAILURE;
    }

    sleep(3);

    if (!writeCmd(serialFd, SBC_TX_EVT_PROG_STARTED, NULL, 0))
    {
        fprintf(stderr, "Error writing EVT_PROG_STARTED\n");
        return 1;
    }

    uint8_t b, seqMatch = 0, buf[1024], eseq[START_SEQ_LEN];
    uint16_t payloadLen, missed = 0;

    while (1)
    {
        seqMatch = 0;
        printf("Reading...\n");
        while (seqMatch != START_SEQ_LEN)
        {
            if (getBytes(serialFd, &b, 1) == -1)
            {
                perror("Error reading");
                break;
            }
            if (b == startSeq[seqMatch])
                seqMatch++;
            else if (b == startSeq[0])
                seqMatch = 1;
            else if (b == startSeq[1])
                seqMatch = 2;
            else if (b == startSeq[2])
                seqMatch = 3;
            else
            {
                seqMatch = 0;
                // printf("*Seq mis %c\n", b);
                missed++;
            }
        }

        if (missed)
            printf("*Missed: %d\n", missed);
        missed = 0;

        if (getBytes(serialFd, buf, 1) == -1)
        {
            perror("Error getting command");
            continue;
        }

        if (getBytes(serialFd, buf + 1, 2) == -1)
        {
            perror("Error getting payloadLen");
            continue;
        }

        payloadLen = *(buf + 1);

        if (getBytes(serialFd, buf + 3, payloadLen) == -1)
        {
            perror("Error reading payload");
            continue;
        }

        printf("payloadLen: %d\n", payloadLen);

        uint8_t crc;
        if (getBytes(serialFd, &crc, 1) == -1)
        {
            perror("Error getting crc");
            continue;
        }

        if (calcCrc8((uint8_t *)buf, payloadLen + 3) != crc)
        {
            printf("CRC mimsatch\n");
            continue;
        }

        printf("Got Cmd: %c, payloadLen: %d\n", *buf, payloadLen);

        switch (*buf)
        {
        case TX_SBC_CMD_PING:
            printf("Got ping\n");
            if (writeCmd(serialFd, SBC_TX_EVT_PONG, NULL, 0))
            {
                printf("Wrote pong\n");
            }
            else
            {
                printf("Could not write pong\n");
            }

            time_t currTime = time(NULL);
            double secs = difftime(currTime, startTime);
            printf("**** Packets between %.0f secs: ==== %d ====\n", secs, pktsBet);
            pktsBet = 0;

            break;

        case TX_SBC_CMD_SHUTDOWN:
            printf("Got shutdown\n");
            system("sudo poweroff");
            break;

        case TX_SBC_EVT_SEN_DATA:
            printf("Got sen data\n");
            senData_t *sd = (senData_t *)(buf + 3);
            printf("\tthrottle: %d\n", sd->throttle);
            printf("\tsteering: %d\n", sd->steering);
            printf("\tdists: %d %d %d\n", sd->dists[0], sd->dists[1], sd->dists[2]);
            printf("\taccX: %.2f, accY: %.2f, accZ: %.2f\n", sd->accX, sd->accY, sd->accZ);
            printf("\tgyroX: %.2f, gyroY: %.2f, gryoZ: %.2f\n", sd->gyroX, sd->gyroY, sd->gyroZ);
            struct timeval tv;
            gettimeofday(&tv, NULL);
            senProgData_t spd = {
                senData: *sd,
                timeMs: (long long)(tv.tv_sec) * 1000 + (tv.tv_usec) / 1000
            };
            size_t res;
            if ((res=write(senDataFd, &spd, sizeof(senProgData_t))) == -1) {
                if (res == EAGAIN || res == EWOULDBLOCK) {
                    perror("Again/would-block");
                } else if (res == EPIPE) {
                    perror("No readers");
                } else {
                    perror("Write failed");
                }
            }
            pktsBet++;
            break;
        }
    }

    close(serialFd);
}

bool writeCmd(int fd, uint8_t cmd, uint8_t *payload, uint16_t payloadLen)
{
    uint8_t buf[1024];
    memcpy(buf, startSeq, START_SEQ_LEN);
    *(buf + START_SEQ_LEN) = cmd;
    memcpy(buf + START_SEQ_LEN + 1, &payloadLen, 2);
    memcpy(buf + START_SEQ_LEN + 3, payload, payloadLen);
    *(buf + START_SEQ_LEN + 3 + payloadLen) = calcCrc8((uint8_t *)(buf + START_SEQ_LEN), 3 + payloadLen);
    if (write(fd, buf, START_SEQ_LEN + 4 + payloadLen) == -1)
    {
        perror("Could not write");
        return false;
    }

    printf("Wrote %c payloadLen: %d\n", cmd, payloadLen);

    return true;
}

int getBytes(int fd, uint8_t *buf, size_t count)
{
    size_t readLen = 0, currReadLen;

    while (count)
    {
        if ((currReadLen = read(fd, buf + readLen, count)) == -1)
            return -1;
        if (currReadLen == 0)
        {
            usleep(1000);
            continue;
        }
        count -= currReadLen;
        readLen += currReadLen;
    }

    return readLen;
}
