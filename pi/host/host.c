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
int writeCmd(int fd, uint8_t cmd, uint8_t *payload, uint16_t payloadLen);
void replyOk(int fd);

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
    uint8_t buf[1024], payload[1024];

    if ((serialFd = open(SERIAL_PORT, O_RDWR | O_NOCTTY)) == -1) {
        perror("Failed to open serial port");
        return 1;
    }

    uint8_t cmd[1024];
    if (writeCmd(serialFd, SBC_TX_EVT_RUNNING , NULL, 0) == -1) {
        perror("Error writing EVT_RUNNING");
        return 1;
    }

    int bytesRead;
    while (1) {
        if ((bytesRead = read(serialFd, buf, sizeof(buf))) == -1) {
            perror("Error reading from searial port");
        } else if (bytesRead == 0) {
            usleep(10000); // 10  ms
            continue;
        }
        
        uint16_t idx = 0, payloadLen;
        char cmd;
        
        do {
            int seqFound = 0;
            while (seqFound < START_SEQ_LEN && idx < bytesRead) {
                if (buf[idx] != startSeq[seqFound]) {
                    seqFound = 0;
                }
                idx++;
            }
            cmd = buf[idx++];
            payloadLen = buf[idx++];
            printf("Got cmd %c: plen: %d - ", cmd, payloadLen);
            
            if (bytesRead < 4 + payloadLen) {
                printf("Payload length mismatch!");
                continue;
            }

            switch (cmd) {
                case TX_SBC_CMD_GET_PROGS:
                    printf("TX_SBC_CMD_GET_PROGS\n");
                    replyOk(serialFd);
                    break;
                case TX_SBC_CMD_RUN_PROG:
                    makeStringFromBuf(buf + idx, payloadLen, payload);
                    printf("TX_SBC_CMD_RUN_PROG: %s", payload);
                    replyOk(serialFd);
                    break;
            }
        } while (idx < bytesRead);
        
    }

    close(serialFd);
}

void makeStringFromBuf(char *bufStart, int payloadLen, char *outString) {
    memcpy(outString, bufStart, payloadLen);
}

int writeCmd(int fd, uint8_t cmd, uint8_t *payload, uint16_t payloadLen) {
    char out[1024];
    memcpy(out, startSeq, START_SEQ_LEN);
    *(out + 3) =  cmd;
    memcpy(out + 4, payload, payloadLen);
    return write(fd, out, START_SEQ_LEN + 1 + payloadLen);
}

void replyOk(int fd) {
    if (write(fd, "ok", 2) == -1) {
        perror("Error replying ok");
        exit(1);
    }
}
