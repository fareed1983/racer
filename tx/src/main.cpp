#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#include <IRremote.hpp>

#include "comms.h"

#define PIN_CLK A1
#define PIN_LATCH A2
#define PIN_DATA A3

#define PIN_IR_RCV 10

#define PIN_VBAT A7

#define PIN_VRX A5
#define PIN_VRY A4
#define PIN_VSW 12

#define PIN_SWA 11
#define PIN_SWB 10

#define PIN_FRC A0

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define ADDR_OLED 0x3C

#define SPKR_ITER 3

RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rf69_manager(rf69, TX_ADDR);


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
char buf[1024], str1[128] = {0}, str2[32];

#define WINDOW_SIZE 5

unsigned long prevsp = 999999;
int sensorValue = 0, a; 
float reading;

enum { NOS, INIT, FAIL, CONN, TRX } connState, prevConnState;

typedef struct {
  float readings[WINDOW_SIZE];
  int readIndex = 0;
  float total = 0;
  float average = 0;
} reading_t;

#define REV_TRANS_CNT 8 // How many iterations with switch pressed to change reverse state
reading_t rTh, rVx, rVy;

int prevSteer = 0, prevThrottle = 0, cenVx = 0, cenVy = 0, prevRev = false, rev = false, revTrans = REV_TRANS_CNT, lastCmd = 0, txFailCount = 0;
uint8_t spkr = 0;

txRxSimpleCtrl_t sc;

// Initialize the readings array to 0
void initReadings(reading_t *r) {
    for (int i = 0; i < WINDOW_SIZE; i++) {
        r->readings[i] = 0.0;
    }
}

// Function to add a new reading and compute the moving average
float addReading(reading_t *r, float newReading) {
    // Subtract the oldest reading from total and replace it with the new reading
    r->total = r->total - r->readings[r->readIndex];
    r->readings[r->readIndex] = newReading;
    r->total = r->total + newReading;
    
    // Advance to the next position in the array
    r->readIndex = (r->readIndex + 1) % WINDOW_SIZE;
    
    // Calculate the average
    r->average = r->total / WINDOW_SIZE;
    
    return r->average;
}

void setup() {

  pinMode(PIN_LATCH, OUTPUT);
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);

  digitalWrite(PIN_LATCH, LOW);
  shiftOut(PIN_DATA, PIN_CLK, MSBFIRST, 0b00000000);
  shiftOut(PIN_DATA, PIN_CLK, MSBFIRST, 0b00111110);
  digitalWrite(PIN_LATCH, HIGH);

  Serial.begin(115200);
  Wire.begin();

  connState = INIT;
  prevConnState = NOS;

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  
  initReadings(&rTh);
  initReadings(&rVx);
  initReadings(&rVy);

  pinMode(PIN_VSW, INPUT_PULLUP);
  pinMode(PIN_SWA, INPUT_PULLUP);
  pinMode(PIN_SWB, INPUT_PULLUP);

  strcpy(str1, "Devices:\n");

  int nDevices = 0;
  byte error, address;

  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      if (nDevices >= 1) strcat (str1, ", ");
      strcat(str1, "S:0x");
      if (address<16) strcat(str1, "0");
      sprintf(str2, "%x", address);
      strcat(str1, str2);
      nDevices++;
    } else if (error==4)
    {
      if (nDevices > 1) strcat (str1, ", ");
      strcat(str1, "E:0x");
      if (address<16) strcat(str1, "0");
      sprintf(str2, "%x", address);
      strcat(str1, str2);
    }
  }
  if (nDevices == 0) strcat(str1, "No I2C devices found");

  if(!display.begin(SSD1306_SWITCHCAPVCC, ADDR_OLED)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  } 

  if (!rf69_manager.init()) {
    strcat(str1, "\nRFM69 fail");
  } else if (!rf69.setFrequency(RF69_FREQ)) {
     strcat(str1, "\nFreq fail");
  } else {
    strcat(str1, "\nRadio OK");
  }
  
  rf69.setTxPower(20, true);
  uint8_t key[] = "FareedR11051983";
  rf69.setEncryptionKey(key);

  rf69_manager.setTimeout(10);
  rf69_manager.setRetries(3);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.cp437(true);
  display.setTextSize(2);
  display.println("Racer TX");
  display.setTextSize(1);
  display.println("v0.1");
  display.println(str1);
  display.display();
  
  IrReceiver.begin(PIN_IR_RCV, ENABLE_LED_FEEDBACK);

  delay(500);
}


void loop() {
  unsigned long ms = millis();

  sensorValue = analogRead(PIN_FRC);
  reading = (pow(10, sensorValue / 1024.0) - 1.0) * 20.0;
  reading = (pow(10, reading / 200.0) - 1.0) * 20.0;
  reading = (pow(10, reading / 200.0) - 1.0) * 20.0;
  reading = (pow(10, reading / 200.0) - 1.0) * 12.0;

  unsigned long sp = 0;

  if (reading > 15) reading = 15;
  if (revTrans < REV_TRANS_CNT) reading = 0; // allow cooldown when transitioning reverse state

  reading = addReading(&rTh, reading);
  
  if (reading > 1) {
    sp = pow(2, int(reading)) - 1;
  } else {
    sp = reading;
  }
  
  /*
    0b00rgbBY0
  */

  if (sp >= 1024) sp = 1023;

  if (prevsp != sp || prevConnState != connState || prevRev != rev || spkr == 1 || spkr == SPKR_ITER) {
    unsigned long  rgbby;
    switch(connState) { // it's a common anode RGB LED
      case NOS:
      case INIT:
        rgbby = 0b00111110;
        break;
      case FAIL:
        rgbby = 0b00100000;
        break;
      case CONN:
        rgbby = 0b00001000;
        break;
      case TRX:
        rgbby = 0b00010000;
        break;
      default:
        rgbby = 0b00100000;
    }

    if (rev) rgbby |= 0b00000100;
    
    digitalWrite(PIN_LATCH, LOW);
    shiftOut(PIN_DATA, PIN_CLK, MSBFIRST, (0b11111110 & (sp >> 2)) | ((spkr > 1) ? 1 : 0));
    shiftOut(PIN_DATA, PIN_CLK, MSBFIRST, (0b00111110 | (0b11000000 & sp << 6) | (0xb00000001 & sp >> 2)) ^ rgbby);
    digitalWrite(PIN_LATCH, HIGH);

    prevsp = sp;
    prevConnState = connState;

    // Serial.print(sensorValue);
    // Serial.print(":");
    // Serial.println(sp);
  }

  if (spkr) spkr--;

  static char irCmd = '-';
  static unsigned long lastIrCmd;

  if (lastIrCmd + 250 < ms) irCmd = '-';

  if (IrReceiver.decode()) {

    if (IrReceiver.decodedIRData.protocol == NEC && IrReceiver.decodedIRData.address == 0x0102) {
      Serial.print(F("Proto:"));
      Serial.print(getProtocolString(IrReceiver.decodedIRData.protocol));
      Serial.print(F(", data:"));
      PrintULL::print(&Serial, IrReceiver.decodedIRData.decodedRawData, HEX);
      Serial.print(F(", addr: "));
      Serial.print(IrReceiver.decodedIRData.address, HEX);
      Serial.print(F(", cmd: "));
      Serial.println(IrReceiver.decodedIRData.command, HEX);
      irCmd =  IrReceiver.decodedIRData.command;
      lastIrCmd = ms;
    }

    IrReceiver.resume();
  }

  sensorValue = analogRead(PIN_VRX);
  
  int vrx, vry, vsw, swa, swb;

  vrx = addReading(&rVx, analogRead(PIN_VRX));
  vry = addReading(&rVy, analogRead(PIN_VRY));

  vsw = !digitalRead(PIN_VSW);
  swa = !digitalRead(PIN_SWA);
  swb = !digitalRead(PIN_SWB);

  prevRev = rev;

  // The first time readIndex becomes zero
  if (!cenVx && !rVx.readIndex) {
    
    cenVx = rVx.average;
    cenVy = rVy.average;

    display.setCursor(0, 10);

    Serial.print("cenVx: ");
    Serial.println(cenVx);

  } else if (cenVx) {
    display.clearDisplay();

    if (vsw) {
      revTrans --;
      if (!revTrans) {
        rev = !rev;
        spkr = SPKR_ITER;
      }
    } else {
      revTrans = REV_TRANS_CNT;
    }

    int cth, cst;

    cth = (rTh.average * 10.0);
    if (cth > 100) cth = 100;
    if (rev) cth *= -1;

    cst = (cenVx - vrx) / -5.12;
    
    sprintf(str1, "S:%d", cst);
    display.setCursor(0, 25);
    display.print(str1);

    sprintf(str1, "y:%d", vry);
    display.setCursor(45, 25);
    display.print(str1);

    sprintf(str1, "s:%d", vsw);
    display.setCursor(90, 25);
    display.print(str1);

    sprintf(str1, "T:%d", cth);
    display.setCursor(0, 35);
    display.print(str1);




    if (sc.throttle != cth || sc.steering != cst || lastCmd + 100 < ms || swb ) {
      uint8_t len; 

      if (swb) { // TODO put this behind a menu. In menu mode, stop sending ctrl cmds
        buf[0] = TX_RX_CMD_START_SBC;
        len = 1;
      } else {
        sc.throttle  = cth;
        sc.steering  = cst;
      //cmd.raspToggle = swb;
        buf[0] = TX_RX_CMD_SIMPLE_CTRL;
        memcpy(buf + 1, &sc, sizeof(sc));
        len = sizeof(txRxDirectionCtrl_t) + 1;
      }

      if (!rf69_manager.sendtoWait((uint8_t *)buf, len, RX_ADDR)) {
        if (txFailCount == 5) {
          spkr = SPKR_ITER;
          connState = FAIL;
        }
        txFailCount ++;
      } else {
        if (connState == FAIL) spkr = SPKR_ITER;
        connState = TRX;
        txFailCount = 0;
      }

      lastCmd = ms;
    } 
    
    if (connState == TRX && lastCmd + 100 < ms) connState = CONN;

    display.setCursor(90, 35);
    sprintf(str1, "A:%d", connState);
    display.print(str1);

    display.setCursor(0, 45);
    sprintf(str1, "D:%c", irCmd);
    display.print(str1);
  
    float measuredvbat = analogRead(PIN_VBAT);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    display.setCursor(45, 35);
    sprintf(str1, "V:%.02f", measuredvbat);
    display.print(str1);

    sprintf(str1, "A:%d", swa);
    display.setCursor(45, 45);
    display.print(str1);

    sprintf(str1, "B:%d", swb);
    display.setCursor(90, 45);
    display.print(str1);

    display.setCursor(0, 0);
    display.print("TX DBG");

    display.display();
  }
   
  delay(10);
}

