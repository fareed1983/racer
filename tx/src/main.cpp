#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>

#define PIN_CLK 10
#define PIN_LATCH 11
#define PIN_DATA 12

#define PIN_VRX A1
#define PIN_VRY A2
#define PIN_VSW A3

#define PIN_FRC A0

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define ADDR_OLED 0x3C

/* Radio stuff */
#define RF69_FREQ 433.0

#define RFM69_CS    8
#define RFM69_INT   3
#define RFM69_RST   4
#define LED        13

#define RX_ADDR   1
#define TX_ADDR   2


RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rf69_manager(rf69, TX_ADDR);


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
char str1[128] = {0}, str2[32];

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

struct {
  int th;
  int st;  
} cmd;

#define REV_TRANS_CNT 10 // How many iterations with switch pressed to change reverse state
reading_t rTh, rVx, rVy;

int prevSteer = 0, prevThrottle = 0, cenVx = 0, cenVy = 0, rev = false, revTrans = REV_TRANS_CNT, lastCmd = 0;

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
  Serial.begin(115200);
  Wire.begin();

  connState = INIT;
  prevConnState = NOS;


  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);
  
  initReadings(&rTh);
  initReadings(&rVx);
  initReadings(&rVy);

  pinMode(PIN_LATCH, OUTPUT);
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);

  pinMode(PIN_VSW, INPUT_PULLUP);

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
  display.println("Racer RX");
  display.setTextSize(1);
  display.println("v0.1");
  display.println(str1);
  display.display();
  
  delay(2000);
}


void loop() {
  int ms = millis();

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

  if (sp >= 1024) sp = 1023;

  if (prevsp != sp || prevConnState != connState) {
    unsigned long  cst;
    switch(connState) { // it's a common anode RGB LED
      case NOS:
      case INIT:
        cst = 0x00;
        break;
      case FAIL:
        cst = 0x18;
        break;
      case CONN:
        cst = 0x24;
        break;
      case TRX:
        cst = 0x30;
        break;
      default:
        cst = 0x38;
    }

    digitalWrite(PIN_LATCH, LOW);
    shiftOut(PIN_DATA, PIN_CLK, MSBFIRST, 0xff & ((sp >> 8) | cst));
    shiftOut(PIN_DATA, PIN_CLK, MSBFIRST, 0xff & sp);
    digitalWrite(PIN_LATCH, HIGH);
    
    prevsp = sp;
    prevConnState = connState;

    // Serial.print(sensorValue);
    // Serial.print(":");
    // Serial.println(sp);
  }

  sensorValue = analogRead(PIN_VRX);
  
  int vrx, vry, vsw;

  vrx = addReading(&rVx, analogRead(PIN_VRX));
  vry = addReading(&rVy, analogRead(PIN_VRY));

  vsw = digitalRead(PIN_VSW);


  // The first time readIndex becomes zero
  if (!cenVx && !rVx.readIndex) {
    
    cenVx = rVx.average;
    cenVy = rVy.average;

    display.setCursor(0, 10);

    Serial.print("cenVx: ");
    Serial.println(cenVx);

  } else if (cenVx) {
    display.clearDisplay();

    if (!vsw) {
      revTrans --;
      if (!revTrans) {
        rev = !rev;
      }
    } else {
      revTrans = REV_TRANS_CNT;
    }

    int cth, cst;

    cth = (rTh.average * 10.0);
    if (cth > 100) cth = 100;
    if (rev) cth *= -1;

    cst = (cenVx - vrx) / 5.12;
    
    sprintf(str1, "S:%d", cst);
    display.setCursor(0, 10);
    display.print(str1);

    sprintf(str1, "y:%d", vry);
    display.setCursor(45, 10);
    display.print(str1);

    sprintf(str1, "s:%d", vsw);
    display.setCursor(90, 10);
    display.print(str1);

    sprintf(str1, "T:%d", cth);
    display.setCursor(0, 20);
    display.print(str1);

    sprintf(str1, "R:%d", rev);
    display.setCursor(45, 20);
    display.print(str1);

    if (cmd.th != cth || cmd.st != cst || lastCmd + 250 < ms) {
      cmd.th = cth;
      cmd.st = cst;
      if (!rf69_manager.sendtoWait((uint8_t *)&cmd, sizeof(cmd), RX_ADDR)) {
        connState = FAIL;
      } else {
        connState = TRX;
      }

      lastCmd = ms;
    } 
    
    if (connState == TRX && lastCmd + 100 < ms) connState = CONN;

    display.setCursor(90, 20);

    sprintf(str1, "A:%d", connState);
    display.print(str1);

    display.display();
  }

  delay(5);
}

