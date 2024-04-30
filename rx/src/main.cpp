#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <FastLED.h>
#include <IRremote.hpp>
#include <Ultrasonic.h>
#include <MPU6050_light.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <math.h>

#include "comms.h"

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define ADDR_OLED 0x3C

// Radio stuff
#define RF69_FREQ 433.0
#define RFM69_CS    8
#define RFM69_INT   3
#define RFM69_RST   4

#define RX_ADDR   1
#define TX_ADDR   2

#define PIN_LED_DATA A4

#define PIN_MPU_INT A3

#define PIN_BATT_SENSE A5

// #define PIN_IR_BK 22
// #define PIN_IR_FR 6

#define PIN_UL_TRIG_0 A0
#define PIN_UL_ECHO_0 12

#define PIN_UL_TRIG_1 A1
#define PIN_UL_ECHO_1 11

#define PIN_UL_TRIG_2 A2
#define PIN_UL_ECHO_2 10

#define PIN_UL_TRIG_3 A3
#define PIN_UL_ECHO_3 9

#define PIN_PWR_CTRL 5

#define SER_MIN 1150
#define SER_MID 1500
#define SER_MAX 1850

#define ESC_MIN 1100
#define ESC_MID 1500
#define ESC_MAX 1900

#define ULTRAS_TOT 3


/*
  0x1e  Magnometer HW-127
  0x27  16x2 LCD
  0x3c  OLED   
  0x40  PWM
  0x68  MPU
  0x70  Temperature sensor (?)
*/

#define SER_IDX 15
#define ESC_IDX 14

#define SERUS(pulseWidth)  pwm.writeMicroseconds(SER_IDX, pulseWidth);
#define ESCUS(pulseWidth)  pwm.writeMicroseconds(ESC_IDX, pulseWidth);

#define NUM_LEDS 8

#define DANGER_DIST 50

#define SBC_ST_OFF              'o'
#define SBC_ST_BOOTING          'b'
#define SBC_ST_READY            'r'
#define SBC_ST_POWERING_OFF     'w'
#define SBC_ST_PROG_STARTING    's'
#define SBC_ST_PROG_PASSIVE     'p'
#define SBC_ST_PROG_MASTER      'm'
#define SEC_ST_PROG_TERM        'k'

RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rf69_manager(rf69, RX_ADDR);
CRGB leds[NUM_LEDS];
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
MPU6050 mpu(Wire);
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

int iri = 0;
uint16_t dists[4] = {0, 0, 0, 0}, prevSs = SER_MID, prevEs = ESC_MID, distFront, ultraIdx = 0;
float prevBattV = 0, lastYaw = 0;
unsigned long nextSec = 0, poweroffTransition = 0;
bool connected = false;
char sbcState = SBC_ST_OFF;
char str1[128] = {0}, buf[128];
senData_t sd;

struct {
  int th;
  int st;
  bool raspToggle;
} cmd;

// struct {
//   IRsend *sender;
//   char pin;
//   char cmd;
// } irs[2];

Ultrasonic ultras[4]={
  Ultrasonic(PIN_UL_TRIG_0, PIN_UL_ECHO_0),
  Ultrasonic(PIN_UL_TRIG_1, PIN_UL_ECHO_1),
  Ultrasonic(PIN_UL_TRIG_2, PIN_UL_ECHO_2),
  Ultrasonic(PIN_UL_TRIG_3, PIN_UL_ECHO_3)
};	

bool writeCmd(uint8_t cmd, uint8_t *payload, uint16_t payloadLen);

void setup() {
  // irs[0] = { sender: new IRsend(), pin: PIN_IR_BK, cmd: 'b' };
  // irs[1] = { sender: new IRsend(), pin: PIN_IR_FR, cmd: 'f' };

  
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial1.setTimeout(10);


  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  
  pinMode(RFM69_RST, OUTPUT);
  pinMode(PIN_PWR_CTRL, OUTPUT);

  pinMode(PIN_BATT_SENSE, INPUT_PULLDOWN);
  digitalWrite(RFM69_RST, LOW);

  lcd.init(); 
  lcd.backlight();
  
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Racer RX v0.1");

  if (!rf69_manager.init()) {
    lcd.print("RFM69 radio init failed");
    while (1) delay(1000);
  }

  lcd.setCursor(0, 1);
  
  if (!rf69.setFrequency(RF69_FREQ)) {
    lcd.print("setFrequency failed");
    while (1) delay(1000);
  } else {
    lcd.print("RFM69 radio init OK");
  }

  rf69.setTxPower(20, true);
  uint8_t key[] = "FareedR11051983";
  rf69.setEncryptionKey(key);
  
  FastLED.addLeds<NEOPIXEL, PIN_LED_DATA>(leds, NUM_LEDS);  // GRB ordering is assumed
  FastLED.setBrightness(2);

  ESCUS(ESC_MID);
  SERUS(SER_MID);

  //for (int i = 0; i < 2; i++) irs[i].sender->begin(irs[i].pin);

  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  Wire.begin();
  Wire.setClock(1000000);

  nDevices = 0;
  for(address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  mpu.begin();
  mpu.calcOffsets(true,true); // gyro and accelero

  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("No cmd!         ");

  // if(!display.begin(SSD1306_SWITCHCAPVCC, ADDR_OLED)) { // Address 0x3D for 128x64
  //   Serial.println(F("SSD1306 allocation failed"));
  //   for(;;);
  // } 

  // display.clearDisplay();
  // display.setTextColor(SSD1306_WHITE);
  // display.setCursor(0, 0);
  // display.cp437(true);
  // display.setTextSize(2);
  // display.println("Racer RX");
  // display.setTextSize(1);
  // display.println("v0.1");
  // display.println(str1);
  // display.display();

  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
}

void loop() {

  uint8_t len = sizeof(cmd);
  uint8_t from;
  bool upd0 = false, upd1 = false;
  unsigned long currTime = millis(); 
  char prevSbcState = sbcState;

  // TODO replace this with photoresistor input
  if (sbcState == SBC_ST_POWERING_OFF && poweroffTransition < currTime) {
    sbcState = SBC_ST_OFF;
    digitalWrite(PIN_PWR_CTRL, false);
    poweroffTransition = 0;
  }

  if (rf69_manager.recvfromAckTimeout((uint8_t *)&cmd, &len, 150, &from)) {
    int ss = map(cmd.st, -100, 100, SER_MIN, SER_MAX);
    if (cmd.raspToggle) {
      switch (sbcState) {
        case SBC_ST_OFF:
          digitalWrite(PIN_PWR_CTRL, false);
          delay(1000);
          digitalWrite(PIN_PWR_CTRL, true);
          sbcState = SBC_ST_BOOTING;
          break;
        
        case SBC_ST_BOOTING:        
        case SBC_ST_POWERING_OFF:
          Serial.println("Not doing anything as transitory state");
          break;

        default:
          writeCmd(TX_SBC_CMD_SHUTDOWN, NULL, 0);
          sbcState = SBC_ST_POWERING_OFF;
          poweroffTransition = currTime + 20000;
          break;
      }
    }

    if (cmd.th < 1) cmd.th /= 2;
    int es = map(cmd.th, -100, 100, ESC_MIN, ESC_MAX);

    if (distFront < DANGER_DIST && cmd.th > 0) {
      es = ESC_MID;
    }

    if (prevSs != ss) {
      SERUS(ss);
      prevSs = ss;
      upd1 = true;
    }

    if (prevEs != es) {
      ESCUS(es);
      prevEs = es;
      upd1 = true;
    }
    
    connected = true;
  } else if (connected) {   // no command received

    if (prevEs != ESC_MID || prevSs != SER_MID) {
      prevEs = ESC_MID;
      prevSs = SER_MID;
      SERUS(SER_MID); 
      ESCUS(ESC_MID);
    } 

    upd1 = true;
    connected = false;
  }

  int d;
  
  for (d = 0; d < ULTRAS_TOT; d++) {
    dists[d] = ultras[d].read();
    delay(10);
  }

  if (distFront != dists[UL_FR_IDX]) {
    distFront = dists[UL_FR_IDX];
    if (distFront < DANGER_DIST && cmd.th > 0 && prevEs != ESC_MID) {
        ESCUS(ESC_MID); 
    }
    //upd = true;
  }

  // irs[iri].sender->sendNEC(0x0102, irs[iri].cmd, 0);
  // iri++;
  // if (iri == 2) iri = 0;

  for (d = 0; d < NUM_LEDS; d++) {
    leds[d] = CRGB(random(0, 255), random(0, 255), random(0, 255));
  }

  float v;
  d = analogRead(PIN_BATT_SENSE);
  v = d * 7.52 / 410.0;
  v = int(v * 10) / 10.0;
  if (v != prevBattV) {
    prevBattV = v;
    upd0 = true;
  }

  mpu.update();
  v = mpu.getAngleZ();
  v = int(v * 10) / 10.0;

  if (v != lastYaw) {
    lastYaw = v;
    upd0 = true;
  }

  if (nextSec < currTime) {
    upd0 = true;
  }

  uint8_t seqMatch = 0;
  uint16_t payloadLen = 0;
  char b;
  while (Serial1.available()) {
    while (seqMatch != START_SEQ_LEN && Serial1.available()) {
      if (Serial1.readBytes(&b, 1) == 0) {
        Serial.println("Error getting start seq");
        continue;
      }
      if (b == startSeq[seqMatch]) seqMatch++;
      else if (b == startSeq[0]) seqMatch = 1; 
      else if (b == startSeq[1]) seqMatch = 2;
      else if (b == startSeq[2]) seqMatch = 3;
      else {
        seqMatch = 0;
        Serial.printf("Seq mis %c\n", b);
      }
    }

    if (seqMatch != START_SEQ_LEN) {
      Serial.println("Seq not matched");
      break;
    }

    if (Serial1.readBytes(buf, 1) != 1) {
      Serial.println("Error getting command");
      break;
    }

    if (Serial1.readBytes(buf + 1, 2) != 2) {
      Serial.println("Error getting payloadLen");
      break;
    }

    payloadLen = *(buf + 1);


    Serial.printf("cmd: %c payloadLen: %d\n", *buf, payloadLen);

    if (Serial1.readBytes(buf + 3, payloadLen) != payloadLen) {
      Serial.println("Error getting payload");
      break;
    }

    uint8_t crc;
    if (Serial1.readBytes((char *)&crc, 1) != 1) {
      Serial.println("Error getting CRC");
      break;
    }

    if (calcCrc8((uint8_t *)buf, payloadLen + 3) != crc) {
      Serial.println("CRC mismatch");
      break;
    }

    Serial.printf("Got Cmd: %c, payloadLen: %d\n", *buf, payloadLen);

    switch (*buf) {
      case SBC_TX_EVT_RUNNING:
        Serial.println("Got EVT_RUNNING");
        sbcState = SBC_ST_READY;
        break;

      case SBC_TX_EVT_PONG:
        Serial.println("Got PONG!");
        break;
      
      case SBC_TX_EVT_PROG_STARTED:
        Serial.println("Got PROG started");
        sbcState = SBC_ST_PROG_PASSIVE;
        break;
    }
  }

  if (prevSbcState != sbcState) upd1 = true;

  if (upd0) {
    lcd.setCursor(0, 0);
    sprintf(str1, "%d:%03d V%.01f Y%.01f  ", ultraIdx, dists[ultraIdx], prevBattV, lastYaw);
    lcd.print(str1);
  }

  if (upd1) {
    lcd.setCursor(0, 1);
    if (connected) {
      sprintf(str1, "%c T:%03d S:%03d", sbcState, cmd.th, cmd.st);
    } else {
        sprintf(str1, "%c DISCO         ", sbcState);
    }
    lcd.print(str1);
  }

  if (nextSec < currTime) {
    ultraIdx ++;
    if (ultraIdx == ULTRAS_TOT) ultraIdx = 0;
    nextSec = currTime + 1000;
    if (!writeCmd(TX_SBC_CMD_PING, NULL, 0)) {
      Serial.println("Could not write");
    }

    Serial.printf("freeMem: %d\n", freeMemory());
  }


    // sensors_event_t event; 
    // mag.getEvent(&event);
  
    // /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
    // Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
    // Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
    // Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

    // // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // // Calculate heading when the magnetometer is level, then correct for signs of axis.
    // float heading = atan2(event.magnetic.y, event.magnetic.x);
    
    // // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // // Find yours here: http://www.magnetic-declination.com/
    // // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    // float declinationAngle = 0.22;
    // heading += declinationAngle;
    
    // // Correct for when signs are reversed.
    // if(heading < 0)
    //   heading += 2*PI;
      
    // // Check for wrap due to addition of declination.
    // if(heading > 2*PI)
    //   heading -= 2*PI;
    
    // // Convert radians to degrees for readability.
    // float headingDegrees = heading * 180/M_PI; 
    
    // Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  if (sbcState == SBC_ST_PROG_PASSIVE || sbcState == SBC_ST_PROG_MASTER) {
    sd = {
      throttle: connected ? cmd.th : 0,
      steering: connected ? cmd.st : 0,
      dists: { dists[0], dists[1], dists[2] },
      accX: mpu.getAccX(),
      accY: mpu.getAccY(),
      accZ: mpu.getAccZ(),
      gyroX: mpu.getGyroX(),
      gyroY: mpu.getGyroY(),      
      gyroZ: mpu.getGyroZ()
    };

    // Serial.printf("\tthrottle: %d\n", sd.throttle);
    // Serial.printf("\tsteering: %d\n", sd.steering);
    // Serial.printf("\tdists: %d %d %d\n", sd.dists[0], sd.dists[1], sd.dists[2]);
    // Serial.printf("\taccX: %.2f, accY: %.2f, accZ: %.2f\n", sd.accX, sd.accY, sd.accZ);
    // Serial.printf("\tgyroX: %.2f, gyroY: %.2f, gryoZ: %.2f\n", sd.gyroX, sd.gyroY, sd.gyroZ);
    if (!writeCmd(TX_SBC_EVT_SEN_DATA, (uint8_t *)&sd, sizeof(sd))) {
      Serial.println("Error sending sensor data");
    }
  }



  // static unsigned long lastPrint = 0;
  // if(millis() - lastPrint > 1000){ // print data every second
  //   Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
  //   Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
  //   Serial.print("\tY: ");Serial.print(mpu.getAccY());
  //   Serial.print("\tZ: ");Serial.println(mpu.getAccZ());
  
  //   Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
  //   Serial.print("\tY: ");Serial.print(mpu.getGyroY());
  //   Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
  
  //   Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
  //   Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());
    
  //   Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
  //   Serial.print("\tY: ");Serial.print(mpu.getAngleY());
  //   Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
  //   Serial.println(F("=====================================================\n"));
  //   lastPrint = millis();
  // }


  FastLED.show();

}

bool writeCmd(uint8_t cmd, uint8_t *payload, uint16_t payloadLen) {
    memcpy(buf, startSeq, START_SEQ_LEN);
    *(buf + START_SEQ_LEN) =  cmd;
    memcpy(buf + START_SEQ_LEN + 1, &payloadLen, 2);
    memcpy(buf + START_SEQ_LEN + 3, payload, payloadLen);
    *(buf + START_SEQ_LEN + 3 + payloadLen) = calcCrc8((uint8_t *)(buf + START_SEQ_LEN), 3 + payloadLen);
    size_t bytes = START_SEQ_LEN + 4 + payloadLen;

    if (Serial1.write(buf, bytes) != bytes) return false; 

    Serial.printf("Wrote %c payloadLen:%d, bytes: %d\n", cmd, payloadLen, bytes);

    return true;
}


