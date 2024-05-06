#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include "uart.h"
#include "comms.h"

int getBytes(int fd, uint8_t *buf, size_t count);

int uartInit(char *path) {
    int serialFd, ret;

    if ((serialFd = open(path, O_RDWR | O_NOCTTY)) == -1)
    {
        perror("Failed to open serial port");
        return serialFd;
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
    if ((ret = tcflush(serialFd, TCIFLUSH)) == -1) {
        close(serialFd);
        return ret;
    }

    // Flush output buffer
    if ((ret = tcflush(serialFd, TCOFLUSH)) == -1){
        close(serialFd);
        return ret;
    }

    return serialFd;

}

//TODO consider bufSize
int uartReadCmd(int serialFd, uint8_t *buf, size_t bufSize) {
    uint8_t b, seqMatch = 0, eseq[START_SEQ_LEN];
    uint16_t payloadLen;
    
    while (seqMatch != START_SEQ_LEN) {
        
        if (getBytes(serialFd, &b, 1) == -1) {
            perror("Error reading");
            return -1;
        }
        
        if (b == startSeq[seqMatch]) seqMatch++;
        else if (b == startSeq[0]) seqMatch = 1;
        else if (b == startSeq[1]) seqMatch = 2;
        else if (b == startSeq[2]) seqMatch = 3;
        else seqMatch = 0;
    }

    if (getBytes(serialFd, buf, 1) == -1) {
        perror("Error getting command");
        return -1;
    }

    if (getBytes(serialFd, buf + 1, 2) == -1) {
        perror("Error getting payloadLen");
            return -1;
    }

    payloadLen = *(buf + 1);
    printf("payloadLen=%d\n", payloadLen);

    if (getBytes(serialFd, buf + 3, payloadLen) == -1) {
        perror("Error reading payload");
        return -1;
    }

    uint8_t crc;
    if (getBytes(serialFd, &crc, 1) == -1) {
        perror("Error getting crc");
        return -1;
    }

    if (calcCrc8((uint8_t *)buf, payloadLen + 3) != crc) {
        printf("CRC mimsatch\n");
        return -1;
    }

    return payloadLen;
}

int uartWriteCmd(int serialFd, uint8_t cmd, uint8_t *payload, uint16_t payloadLen) {
    uint8_t buf[1024];
    memcpy(buf, startSeq, START_SEQ_LEN);
    *(buf + START_SEQ_LEN) = cmd;
    memcpy(buf + START_SEQ_LEN + 1, &payloadLen, 2);
    memcpy(buf + START_SEQ_LEN + 3, payload, payloadLen);
    *(buf + START_SEQ_LEN + 3 + payloadLen) = calcCrc8((uint8_t *)(buf + START_SEQ_LEN), 3 + payloadLen);
    int ret;
    if ((ret = write(serialFd, buf, START_SEQ_LEN + 4 + payloadLen)) == -1)
    {
        perror("Could not write");
        return ret;
    }

    printf("Wrote %c payloadLen: %d\n", cmd, payloadLen);

    return ret;
}

int getBytes(int fd, uint8_t *buf, size_t count) {
    size_t readLen = 0, currReadLen;

    while (count)
    {
        if ((currReadLen = read(fd, buf + readLen, count)) == -1)
            return -1;

        if (currReadLen == 0)
        {
            usleep(10);
            continue;
        }
        count -= currReadLen;
        readLen += currReadLen;
    }

    return readLen;
}
