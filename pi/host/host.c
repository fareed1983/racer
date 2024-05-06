#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <pthread.h>

#include "comms.h"
#include "uart.h"
#include "i2cSlave.h"

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

typedef struct {
    long long timeMs;
    senData_t senData;
} senProgData_t;

void* uartReader(void* arg);
void* i2cReader(void* arg);


int main(int argc, char **argv)
{
    printf("Pi host v0.1\n\n");

    int serialFd;

    if ((serialFd = uartInit(SERIAL_PORT)) == -1) {
        perror("Error opening uart");
        exit(EXIT_FAILURE);
    }

    
    printf("Opened serial port\n");

    pthread_t uartThread, i2cThread;
    int ret;

    ret = pthread_create(&uartThread, NULL, uartReader, &serialFd);
    if (ret) {
        fprintf(stderr, "Error - pthread_create() return code: %d\n", ret);
        exit(EXIT_FAILURE);
    }

    ret = pthread_create(&i2cThread, NULL, i2cReader, &serialFd);
    if (ret) {
        fprintf(stderr, "Error - pthread_create() return code: %d\n", ret);
        exit(EXIT_FAILURE);
    }

    uint8_t totProgs = sizeof(progs) / sizeof(progs[0]);

    for (uint8_t i = 0; i < totProgs; i++)
    {
        printf("programs[%d]:\n", i + 1);
        printf("\tname: %s\n", progs[i].name);
        printf("\tpath: %s\n", progs[i].path);
        printf("\tisMaster: %s\n", progs[i].isMaster ? "true" : "false");
        printf("\n");
    }


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

    if (!uartWriteCmd(serialFd, SBC_TX_EVT_RUNNING, NULL, 0))
    {
        fprintf(stderr, "Error writing EVT_RUNNING\n");
        return EXIT_FAILURE;
    }

    sleep(3);

    if (!uartWriteCmd(serialFd, SBC_TX_EVT_PROG_STARTED, NULL, 0))
    {
        fprintf(stderr, "Error writing EVT_PROG_STARTED\n");
        return EXIT_FAILURE;
    }


    pthread_join(uartThread, NULL);

}

void* uartReader(void* arg) {

    uint8_t buf[1024];
    int payloadLen;
    int serialFd = *(int*)arg;
    time_t startTime = time(NULL);


    while (1)
    {
        if ((payloadLen = uartReadCmd(serialFd, buf, sizeof(buf))) == -1)  {
            printf("Error reading\n");
            continue;
        }

        printf("Got Cmd: %c, payloadLen: %d\n", *buf, payloadLen);

        switch (*buf)
        {
        case TX_SBC_CMD_PING:
            printf("Got ping\n");
            if (uartWriteCmd(serialFd, SBC_TX_EVT_PONG, NULL, 0))
            {
                printf("Wrote pong\n");
            }
            else
            {
                printf("Could not write pong\n");
            }

            time_t currTime = time(NULL);
            double secs = difftime(currTime, startTime);

            break;

        case TX_SBC_CMD_SHUTDOWN:
            printf("Got shutdown\n");
            system("sudo poweroff");
            break;

        case TX_SBC_EVT_SEN_DATA:
            printf("Got sen data on UART!!!\n");
            senData_t *sd = (senData_t *)(buf + 3);
            
            break;
        }
    }

    close(serialFd);

}


void* i2cReader(void* arg) {

    if (!slaveInit()) {
        printf("Error init i2c slave\n");
        exit(EXIT_FAILURE);
    }

    printf("I2C Slavve initialized");
    uint8_t buf[1024];
    int readSize;
    while(1) {
        if ((readSize = slaveRead(buf, sizeof(buf))) == -1) {
            usleep(10);
            continue;
        }
        if (readSize == 0) {
            usleep(10);
            continue;
        }
        switch (*buf) {
            case TX_SBC_EVT_SEN_DATA:
                senData_t *sd = (senData_t *)(buf + 1);
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
                // size_t res;
                // if ((res=write(senDataFd, &spd, sizeof(senProgData_t))) == -1) {
                //     if (res == EAGAIN || res == EWOULDBLOCK) {
                //         perror("Again/would-block");
                //     } else if (res == EPIPE) {
                //         perror("No readers");
                //     } else {
                //         perror("Write failed");
                //     }
                // }
                break;
        }
        printf("I2C read %d bytes\n", readSize);    
    }
}