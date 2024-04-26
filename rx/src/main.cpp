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
#include <math.h>

#include "comms.h"

MPU6050 mpu(Wire);

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

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

#define UL_FR_IDX 0
#define UL_LT_IDX 1
#define UL_RT_IDX 2
#define UL_BK_IDX 3

#define ULTRAS_TOT 3

// #define TICK_MIN 150
// #define TICK_MAX 600

//#define SERUS(pulseWidth) pwm.setPWM(SER_IDX, 0, map(pulseWidth, SER_MIN, SER_MAX, TICK_MIN, TICK_MAX));
//#define ESCUS(pulseWidth) pwm.setPWM(ESC_IDX, 0, map(pulseWidth, ESC_MIN, ESC_MAX, TICK_MIN, TICK_MAX));

#define SERUS(pulseWidth)  pwm.writeMicroseconds(SER_IDX, pulseWidth);
#define ESCUS(pulseWidth)  pwm.writeMicroseconds(ESC_IDX, pulseWidth);

#define SER_IDX 15
#define ESC_IDX 14

#define NUM_LEDS 8

#define DANGER_DIST 50

RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rf69_manager(rf69, RX_ADDR);
CRGB leds[NUM_LEDS];
LiquidCrystal_I2C lcd(0x27, 16, 2);
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

bool raspOn = false;

/*
  0x1e  Magnometer HW-127
  0x27  16x2 LCD
  0x3c  OLED   
  0x40  PWM
  0x68  MPU
  0x70  Temperature sensor (?)
*/
struct {
  int th;
  int st;
  bool raspToggle;
} cmd;

char type;

char str1[128] = {0}, str2[32];

char buf[1024];

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
  Serial.begin(115200);

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
  

  //attachInterrupt(digitalPinToInterrupt(PIN_MPU_INT), mpuInterrupt, RISING); 

}

int iri = 0;

int dists[4] = {0, 0, 0, 0}, prevSs = SER_MID, prevEs = ESC_MID, distFront, ultraIdx = 0;
float prevBattV = 0, lastYaw = 0;
unsigned long nextSec = 0;
bool connected = false;

void loop() {

  uint8_t len = sizeof(cmd);
  uint8_t from;
  bool upd = false;
  unsigned long currTime = millis(); 

  if (rf69_manager.recvfromAckTimeout((uint8_t *)&cmd, &len, 150, &from)) {
    int ss = map(cmd.st, -100, 100, SER_MIN, SER_MAX);
    if (cmd.raspToggle) {
      if (raspOn) {
        digitalWrite(PIN_PWR_CTRL, false);
        delay(1000);
        digitalWrite(PIN_PWR_CTRL, true);
      } else {
        Wire.beginTransmission(0x03);
        Wire.write("shutdown");
        Wire.endTransmission(0x03);
      }
      raspOn = !raspOn;
      delay(1000);
    }

    if (cmd.th < 1) cmd.th /= 2;
    int es = map(cmd.th, -100, 100, ESC_MIN, ESC_MAX);

    if (distFront < DANGER_DIST && cmd.th > 0) {
      es = ESC_MID;
    }

    if (prevSs != ss) {
      SERUS(ss);
      prevSs = ss;
      upd = true;
    }

    if (prevEs != es) {
      ESCUS(es);
      prevEs = es;
      upd = true;
    }
    
    if (upd) {
      lcd.setCursor(0, 1);
      sprintf(str1, "T:%03d  S:%03d", cmd.th, cmd.st);
      lcd.print(str1);
    }

    connected = true;
  } else if (connected) {

    if (prevEs != ESC_MID || prevSs != SER_MID) {
      prevEs = ESC_MID;
      prevSs = SER_MID;
      SERUS(SER_MID); 
      ESCUS(ESC_MID);
    } 

    lcd.setCursor(0, 1);
    lcd.print("No cmd!         ");
    connected = false;
    
  }

  int d;
  
  for (d = 0; d < ULTRAS_TOT; d++) {
    dists[d] = ultras[d].read();
    delay(10);
  }

  upd = false;

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
    upd = true;
  }

  mpu.update();
  v = mpu.getAngleZ();
  v = int(v * 10) / 10.0;

  if (v != lastYaw) {
    lastYaw = v;
    upd = true;
  }

  if (nextSec < currTime) {
    upd = true;
  }

  if (upd) {
    lcd.setCursor(0, 0);
    sprintf(str1, "%d:%03d V%.01f Y%.01f    ", ultraIdx, dists[ultraIdx], prevBattV, lastYaw);
    lcd.print(str1);
  }

  if (nextSec < currTime) {
    ultraIdx ++;
    if (ultraIdx == ULTRAS_TOT) ultraIdx = 0;
    nextSec = currTime + 1000;
    if (!writeCmd(TX_SBC_CMD_PING, NULL, 0)) {
      Serial.println("Could not write");
    }
  }

  char b;
  uint8_t seqMatch = 0;
  uint16_t payloadLen = 0;
  while (Serial1.available()) {
    Serial.println("Reading...");
    while (seqMatch != START_SEQ_LEN && Serial1.available()) {
      if (Serial1.readBytes(&b, 1) == 0) {
        Serial.println("Error getting start seq");
        continue;
      }
      printf("Got %c\n", b);
      if (startSeq[seqMatch] == b) seqMatch ++; else seqMatch = 0;
    }

    if (!Serial1.available()) break;

    if (!Serial1.readBytes(&b, 1)) {
      Serial.println("Error getting command");
      break;
    }

    if (!Serial1.readBytes((char *)&payloadLen, 2)) {
      Serial.println("Error getting payloadLen");
      break;
    }

    if (Serial1.readBytes(buf, payloadLen) != payloadLen) {
      Serial.println("Error getting payload");
      break;
    }

    Serial.printf("Got cmd %d payloadLen:%d\n", b, payloadLen);

    switch (b) {
      case SBC_TX_EVT_RUNNING:
        Serial.println("Got EVT_RUNNING");
        break;
      case SBC_TX_EVT_PONG:
        Serial.println("Got PONG!");
        break;
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
    *(buf + 3) =  cmd;
    memcpy(buf + 4, &payloadLen, 2);
    memcpy(buf + 6, payload, payloadLen);
    size_t bytes = START_SEQ_LEN + 6 + payloadLen;
    if (Serial1.write(buf, bytes) != bytes) return false; 

    return true;
}