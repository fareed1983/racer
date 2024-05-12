// #include <Arduino.h>
// // State transition events
// #define TX_SBC_CMD_PING         'G'
// #define SBC_TX_EVT_PONG         'g'
// #define SBC_TX_EVT_RUNNING      'r'
// #define TX_SBC_CMD_GET_PROGS    'g'
// #define TX_SBC_CMD_RUN_PROG     'P'
// #define SBC_TX_EVT_PROG_STARTED 'p'
// #define TX_SBC_TERM_PROG        'K'
// #define SBC_TX_EVT_PROG_EXIT    'x'
// #define SBC_TX_CMD_MASTER       'm'
// #define SBC_TX_CMD_YIELD        'y'
// #define TX_SBC_CMD_SHUTDOWN     'U'

// // General communication
// #define TX_SBC_CMD_GET_NET_STAT 'g'
// #define SBC_TX_EVT_NET_STAT     'n'
// #define TX_SBC_CMD_SET_SSID     'W'
// #define TX_SBC_EVT_SEN_DATA     'd'  

// typedef enum { 
//   OFF, 
//   BOOTING, 
//   READY,
//   PROG_STARTING, 
//   PROG_RUNNING_PASSIVE,
//   PROG_RUNNING_MASTER,
//   PROG_TERMINATING
// } sbcStates_t;

// #define START_SEQ_LEN 4

// #define START_SEQ "<!~|"

// extern const char *startSeq;


// #define UL_FR_IDX 0
// #define UL_LT_IDX 1
// #define UL_RT_IDX 2
// #define UL_BK_IDX 3

// typedef struct {
//   int8_t throttle;
//   int8_t steering;
//   uint16_t dists[3];
//   float accX, accY, accZ;
//   float gyroX, gyroY, gyroZ;
// } senData_t;

// #define CRC8_POLY 0x07

// char buf[128] = {0};
// unsigned long nextPingSec = 0, nextSdSec = 0;

// const char *startSeq = START_SEQ;
// senData_t sd;

// uint8_t calcCrc8(uint8_t *data, size_t len) {
//   uint8_t crc = 0;

//   for (size_t i = 0; i < len; i++) {
//     crc ^= data[i];
//     for (int j = 0; j < 8; j++) {
//         if (crc & 0x80) {
//             crc = (crc << 1) ^ CRC8_POLY;
//         } else {
//             crc <<= 1;
            
//         }
//     }
//   }

//   return crc;
// }

// bool writeCmd(uint8_t cmd, uint8_t *payload, uint16_t payloadLen) {
//     memcpy(buf, startSeq, START_SEQ_LEN);
//     memcpy(buf + START_SEQ_LEN, &cmd, 1);
//     // *(buf + START_SEQ_LEN) =  cmd;
//     memcpy(buf + START_SEQ_LEN + 1, &payloadLen, 2);
//     memcpy(buf + START_SEQ_LEN + 3, payload, payloadLen);
//     uint8_t crc = calcCrc8((uint8_t *)(buf + START_SEQ_LEN), 3 + payloadLen);
//     memcpy(buf + START_SEQ_LEN + 3 + payloadLen, &crc, 1); 
//     // *(buf + START_SEQ_LEN + 3 + payloadLen) = calcCrc8((uint8_t *)(buf + START_SEQ_LEN), 3 + payloadLen);
//     size_t bytes = START_SEQ_LEN + 4 + payloadLen;
//     // for (int i = 0; i < bytes; i++) {
//     //   Serial1.write(buf[i]);
//     //   //delayMicroseconds(1);
//     // }
//     if (Serial1.write(buf, bytes) != bytes) return false; 
//     Serial.printf("Wrote %c payloadLen:%d, bytes: %d\n", cmd, payloadLen, bytes);

//     return true;
// }



// void setup() {
//   Serial.begin(115200);
//   Serial1.begin(115200);



// }

// void loop() {
//   unsigned long currTime = millis(); 

//   uint8_t seqMatch = 0;
//   uint16_t payloadLen = 0;
//   char b;
//   while (Serial1.available()) {
//     while (seqMatch != START_SEQ_LEN && Serial1.available()) {
//       if (Serial1.readBytes(&b, 1) == 0) {
//         Serial.println("Error getting start seq");
//         continue;
//       }
//       if (b == startSeq[seqMatch]) seqMatch++;
//       else if (b == startSeq[0]) seqMatch = 1; 
//       else if (b == startSeq[1]) seqMatch = 2;
//       else if (b == startSeq[2]) seqMatch = 3;
//       else {
//         seqMatch = 0;
//         Serial.printf("Seq mis %c\n", b);
//       }
//     }

//     if (seqMatch != START_SEQ_LEN) {
//       Serial.println("Seq not matched");
//       break;
//     }

//     if (Serial1.readBytes(buf, 1) != 1) {
//       Serial.println("Error getting command");
//       break;
//     }

//     if (Serial1.readBytes(buf + 1, 2) != 2) {
//       Serial.println("Error getting payloadLen");
//       break;
//     }

//     payloadLen = *(buf + 1);


//     Serial.printf("cmd: %c payloadLen: %d\n", *buf, payloadLen);

//     if (Serial1.readBytes(buf + 3, payloadLen) != payloadLen) {
//       Serial.println("Error getting payload");
//       break;
//     }

//     uint8_t crc;
//     if (Serial1.readBytes((char *)&crc, 1) != 1) {
//       Serial.println("Error getting CRC");
//       break;
//     }

//     if (calcCrc8((uint8_t *)buf, payloadLen + 3) != crc) {
//       Serial.println("CRC mismatch");
//       break;
//     }

//     Serial.printf("Got Cmd: %c, payloadLen: %d\n", *buf, payloadLen);

//     switch (*buf) {
//       case SBC_TX_EVT_RUNNING:
//         Serial.println("Got EVT_RUNNING");
//         break;

//       case SBC_TX_EVT_PONG:
//         //Serial.println("Got PONG!");
//         break;
      
//       case SBC_TX_EVT_PROG_STARTED:
//         Serial.println("Got PROG started");
//         break;
//     }
//   }

//   if (nextPingSec < currTime) {
//     //Serial.println("Writing ping");

//     nextPingSec = currTime + 1000;
//     if (!writeCmd(TX_SBC_CMD_PING, NULL, 0)) {
//       Serial.println("Could not write");
//     }
//   }

//   if (nextSdSec < currTime) {
//     //Serial.println("Writing ping");

//     nextSdSec = currTime + 100;
//     sd = {
//       throttle: 100,
//       steering: 50,
//       dists: { 1, 120, 300 },
//       accX: 0,
//       accY: 0,
//       accZ: 0,
//       gyroX: 0,
//       gyroY: 0,      
//       gyroZ: 0
//     };

//     // if (!writeCmd(TX_SBC_EVT_SEN_DATA, (uint8_t *)&sd, sizeof(senData_t))) {
//     //   Serial.println("Error sending sensor data");
//     // }

//     if (!writeCmd(TX_SBC_CMD_PING, NULL, 0)) {
//       Serial.println("Could not write");
//     }
//   }

//   delay(5);
// }

#include <Arduino.h>


void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
}

uint8_t buf[128];

void loop() {
  unsigned long currTime = millis();

  int a;
  for (a=0; a<8; a++)
  {
    buf[a]=a;
  }

  Serial1.write(buf, a);

  Serial.printf("Wrote %d bytes\n", a);
  
  delay(1000);
}