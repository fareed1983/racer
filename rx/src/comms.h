#ifndef __COMMS_H
#define __COMMS_H

// State transition events
#define TX_SBC_CMD_PING         'G'
#define SBC_TX_EVT_PONG         'g'
#define SBC_TX_EVT_RUNNING      'r'
#define TX_SBC_CMD_GET_PROGS    'g'
#define TX_SBC_CMD_RUN_PROG     'P'
#define SBC_TX_EVT_PROG_STARTED 'p'
#define TX_SBC_TERM_PROG        'K'
#define SBC_TX_EVT_PROG_EXIT    'x'
#define SBC_TX_CMD_MASTER       'm'
#define SBC_TX_CMD_YIELD        'y'
#define TX_SBC_CMD_SHUTDOWN     'U'

// General communication
#define TX_SBC_CMD_GET_NET_STAT 'g'
#define SBC_TX_EVT_NET_STAT     'n'
#define TX_SBC_CMD_SET_SSID     'W'
#define TX_SBC_EVT_SEN_DATA     'd'  

typedef enum { 
  OFF, 
  BOOTING, 
  READY,
  PROG_STARTING, 
  PROG_RUNNING_PASSIVE,
  PROG_RUNNING_MASTER,
  PROG_TERMINATING
} sbcStates_t;

#define START_SEQ_LEN 3
#define START_SEQ "<!~"

const char *startSeq = START_SEQ;


#define UL_FR_IDX 0
#define UL_LT_IDX 1
#define UL_RT_IDX 2
#define UL_BK_IDX 3

typedef struct {
  int8_t throttle;
  int8_t steering;
  uint16_t dists[3];
  uint8_t padding1[2];
  float accX, accY, accZ;
  uint8_t padding2[4]; 
  float gyroX, gyroY, gyroZ;
} senData_t;


#endif
