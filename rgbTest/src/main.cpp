// #include <Arduino.h>

// #include <FastLED.h>

// // How many leds in your strip?
// #define NUM_LEDS 15


// #define DATA_PIN 3

// char buf[128] = {0};
// unsigned long nextLed = 0;


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
// uint8_t calcCrc8(uint8_t *data, size_t len);



// // Define the array of leds
// CRGB leds[NUM_LEDS];



// const char *startSeq = START_SEQ;

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
//     *(buf + START_SEQ_LEN) =  cmd;
//     memcpy(buf + START_SEQ_LEN + 1, &payloadLen, 2);
//     memcpy(buf + START_SEQ_LEN + 3, payload, payloadLen);
//     *(buf + START_SEQ_LEN + 3 + payloadLen) = calcCrc8((uint8_t *)(buf + START_SEQ_LEN), 3 + payloadLen);
//     size_t bytes = START_SEQ_LEN + 4 + payloadLen;
//     if (Serial.write(buf, bytes) != bytes) return false; 

//     return true;
// }


// void setup() { 
//   Serial.begin(115200);
//   Serial.setTimeout(100);

  
//   FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
//   FastLED.setBrightness(2);

//   delay(100);
//   writeCmd(SBC_TX_EVT_RUNNING, NULL, 0);
//   delay(2000);
//   writeCmd(SBC_TX_EVT_PROG_STARTED, NULL, 0);
// }

// int led = NUM_LEDS - 1;

// void loop() { 

//   uint8_t seqMatch = 0;
//   uint16_t payloadLen = 0;
//   char b;

//   while (Serial.available()) {
//     led --;
//     if (led < 1) led = NUM_LEDS - 1;
//     leds[led] = CRGB::Black;
//     while (seqMatch != START_SEQ_LEN && Serial.available()) {
//       if (Serial.readBytes(&b, 1) == 0) {
//         continue;
//       }
//       if (b == startSeq[seqMatch]) seqMatch++;
//       else if (b == startSeq[0]) seqMatch = 1; 
//       else if (b == startSeq[1]) seqMatch = 2;
//       else if (b == startSeq[2]) seqMatch = 3;
//       else {
//         seqMatch = 0;
//       }
//     }

//     if (seqMatch != START_SEQ_LEN) {
//       leds[led] = CRGB::White;
//       break;
//     }

//     if (Serial.readBytes(buf, 1) != 1) {
//       leds[led] = CRGB::Orange;
//       break;
//     }

//     if (Serial.readBytes(buf + 1, 2) != 2) {
//       leds[led] = CRGB::Orange;
//       break;
//     }

//     payloadLen = *(buf + 1);

//     if (Serial.readBytes(buf + 3, payloadLen) != payloadLen) {
//       leds[led] = CRGB::Orange;
//       break;
//     }

//     uint8_t crc;
//     if (Serial.readBytes((char *)&crc, 1) != 1) {
//       leds[led] = CRGB::Orange;
//       break;
//     }

//     if (calcCrc8((uint8_t *)buf, payloadLen + 3) != crc) {
//       leds[led] = CRGB::Red;
//       break;
//     }

//     switch (*buf) {
//       case TX_SBC_CMD_PING:
//         leds[led] = CRGB::Blue;
//         writeCmd(SBC_TX_EVT_PONG, NULL, 0);
//         break;
//       case TX_SBC_EVT_SEN_DATA:
//         leds[led] = CRGB::Green;
//         break;
//     }


//   }



//   unsigned long currTime = millis();

//   if (nextLed < currTime) {
//     if (leds[0] == CRGB::Yellow) leds[0] = CRGB::Blue; else leds[0] = CRGB::Yellow;
//     nextLed = currTime + 500;
//   }

//   FastLED.show();

//   delay(2);
// }



#include <Arduino.h>

#include <FastLED.h>

#define NUM_LEDS 15
#define DATA_PIN 3

char buf[128] = {0};
CRGB leds[NUM_LEDS];
int led = NUM_LEDS - 1;
unsigned long nextLed = 0;

void setup() { 
  Serial.begin(115200);
  //Serial.setTimeout(100);

  
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);  // GRB ordering is assumed
  FastLED.setBrightness(2);

}

#define TOT_BYTES 8
bool errLast = false;
void loop() { 

  memset(buf, 0, sizeof(buf));
  while (Serial.available()) {
    if (errLast) {
      buf[0]=0;
      while (buf[0] != TOT_BYTES - 1 && Serial.available()) Serial.readBytes(buf, 1);
      errLast = false;
    }

    led --;
    if (led < 1) led = NUM_LEDS - 1;
    leds[led] = CRGB::Green;
    
    Serial.readBytes(buf, TOT_BYTES);
    for (uint8_t a = 0; a < TOT_BYTES; a++) {
      if (buf[a] != a) {
        leds[led] = CRGB::Red;
        errLast = true;
        break;
      }
    }
  }

  unsigned long currTime = millis();

  if (nextLed < currTime) {
    if (leds[0] == CRGB::Yellow) leds[0] = CRGB::Blue; else leds[0] = CRGB::Yellow;
    nextLed = currTime + 500;
  }
    
  FastLED.show();

}