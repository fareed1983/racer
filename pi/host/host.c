#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <stdint.h>

#include "../../rx/src/comms.h"

#define SERIAL_PORT "/dev/serial0"


struct {
    char *name;
    char *path;
    bool isMaster;
} progs[] = { 
    { "Time Tick",          "../progs/timeTick/time.py",    false },
    { "MQTT Ctrl Test",     "../progs/mqttCtrl/main.py",    true }
};

void makeStringFromBuf(char *bufStart, int payloadLen, char *outString);
bool writeCmd(int fd, uint8_t cmd, uint8_t *payload, uint16_t payloadLen);
void replyOk(int fd);
int getBytes(int fd, uint8_t *buf, size_t count);

int main(int argc, char **argv)
{
    printf("Pi host v0.1\n\n");

    uint8_t totProgs = sizeof(progs) / sizeof(progs[0]);

    for (uint8_t i = 0; i < totProgs; i++) {
        printf("programs[%d]:\n", i + 1);
        printf("\tname: %s\n", progs[i].name);
        printf("\tpath: %s\n", progs[i].path);
        printf("\tisMaster: %s\n", progs[i].isMaster ? "true" : "false");
        printf("\n");
    }

    int serialFd;
        if ((serialFd = open(SERIAL_PORT, O_RDWR | O_NOCTTY)) == -1) {
        perror("Failed to open serial port");
        return 1;
    }

    struct termios options;
    tcgetattr(serialFd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver
    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE; // Mask character size bits
    options.c_cflag |= CS8; // 8 data bits
    tcsetattr(serialFd, TCSANOW, &options);

    // Flush input buffer
    if (tcflush(serialFd, TCIFLUSH) == -1) {
        perror("Error flushing input buffer");
        close(serialFd);
        return 1;
    }

    // Flush output buffer
    if (tcflush(serialFd, TCOFLUSH) == -1) {
        perror("Error flushing output buffer");
        close(serialFd);
        return 1;
    }

    printf("Opened serial port\n");
 
    if (!writeCmd(serialFd, SBC_TX_EVT_RUNNING , NULL, 0)) {
        fprintf(stderr, "Error writing EVT_RUNNING\n");
        return 1;
    }
   
    uint8_t b, seqMatch = 0, payload[1024];
    char cmd;
    uint16_t payloadLen;
    int readLen;

    while (1) {
        seqMatch = 0;
        printf("Reading...\n");
        while(seqMatch != START_SEQ_LEN) {
            if (getBytes(serialFd, &b, 1) == -1) {
                perror("Error reading");
                break;
            }
            if (b == startSeq[seqMatch]) seqMatch++; else seqMatch = 0;
        }

        if (getBytes(serialFd, &cmd, 1) == -1) {
            perror("Error getting command");
            continue;
        }

        if (getBytes(serialFd, (uint8_t *)&payloadLen, 2) == -1) {
            perror("Error getting payloadLen");
            continue;
        }

        if (getBytes(serialFd, payload, payloadLen) == -1) {
            perror("Error reading payload");
            continue;
        }

        switch(cmd) {
            case TX_SBC_CMD_PING:
                printf("Got ping\n");
                if (writeCmd(serialFd, SBC_TX_EVT_PONG, NULL, 0)) {
                    printf("Wrote pong\n");
                } else {
                    printf("Could not write pong\n");
                }
                break;
        }
        
    }

    close(serialFd);
}

void makeStringFromBuf(char *bufStart, int payloadLen, char *outString) 
{
    memcpy(outString, bufStart, payloadLen);
}

bool writeCmd(int fd, uint8_t cmd, uint8_t *payload, uint16_t payloadLen) {
    printf("Writing cmd %c\n", cmd);
    uint8_t buf[1024];
    memcpy(buf, startSeq, START_SEQ_LEN);
    *(buf + 3) =  cmd;
    memcpy(buf + 4, &payloadLen, 2);
    memcpy(buf + 6, payload, payloadLen);
    if (write(fd, buf, START_SEQ_LEN + 6 + payloadLen) == -1) {
        perror("Could not write");
        return false;
    }

    return true;
}

int getBytes(int fd, uint8_t *buf, size_t count) 
{
    size_t readLen = 0, currReadLen;

    while (count) {
        if ((currReadLen = read(fd, buf + readLen, count)) == -1) return -1;
        if (currReadLen == 0) {
            usleep(10000);
            continue;
        }
        count -= currReadLen;
        readLen += currReadLen;
    }

    return readLen;
}

