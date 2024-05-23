#ifndef __COMMS_H
#define __COMMS_H

// Radio stuff
#define RF69_FREQ 433.0
#define RFM69_CS    8
#define RFM69_INT   3
#define RFM69_RST   4

#define RX_ADDR   1
#define TX_ADDR   2

// Radio TX->RX
#define TX_RX_CMD_RST_BEARING     'B'
#define TX_RX_CMD_SIMPLE_CTRL     'C'
#define TX_RX_CMD_DIRECTION_CTRL  'c'
#define TX_RX_CMD_RESET_DIRECTION 'R'
#define TX_RX_CMD_START_SBC       'S'
#define TX_RX_CMD_SHUTDOWN_SBC    's'

// Radio RX->TX
#define TX_RX_EVT_STATE_CHANGE    'T'

// State transition events
#define RX_SBC_CMD_PING           'G'
#define SBC_RX_EVT_PONG           'g'
#define SBC_RX_EVT_RUNNING        'r'
#define RX_SBC_CMD_GET_PROGS      'g'
#define RX_SBC_CMD_RUN_PROG       'P'
#define SBC_RX_EVT_PROG_STARTED   'p'
#define RX_SBC_CMD_TERM_PROG      'K'
#define SBC_RX_EVT_PROG_EXIT      'x'
#define SBC_RX_CMD_MASTER         'm'
#define SBC_RX_CMD_YIELD          'y'
#define RX_SBC_CMD_SHUTDOWN       'U'

// General communication
#define RX_SBC_CMD_GET_NET_STAT   'g'
#define SBC_RX_EVT_NET_STAT       'n'
#define RX_SBC_CMD_SET_SSID       'W'
#define RX_SBC_EVT_SEN_DATA       'd'  

typedef enum { 
  OFF, 
  BOOTING, 
  READY,
  PROG_STARTING, 
  PROG_RUNNING_PASSIVE,
  PROG_RUNNING_MASTER,
  PROG_TERMINATING
} sbcStates_t;

typedef struct {
  int8_t throttle;
  int8_t steering;
} txRxSimpleCtrl_t;

typedef struct {
  int16_t throttle;
  int16_t angle;
  int8_t magnitude;
} txRxDirectionCtrl_t;

#define START_SEQ_LEN 4

#define START_SEQ "<!~|"

extern const char *startSeq;


#define UL_FR_IDX 0
#define UL_LT_IDX 1
#define UL_RT_IDX 2
#define UL_BK_IDX 3

typedef struct {
  int8_t throttle;
  int8_t steering;
  uint16_t dists[3];
  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
} senData_t;

#define CRC8_POLY 0x07
uint8_t calcCrc8(uint8_t *data, size_t len);

#endif
