#include <Arduino.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <RH_RF69.h>
#include <RHReliableDatagram.h>
#include <Servo.h>
#include <FastLED.h>
#include <IRremote.hpp>
#include <Ultrasonic.h>



// Radio stuff
#define RF69_FREQ 433.0
#define RFM69_CS    8
#define RFM69_INT   3
#define RFM69_RST   4
#define LED        13

#define RX_ADDR   1
#define TX_ADDR   2

#define PIN_LED_DATA A0
#define PIN_SERVO A1
#define PIN_ESC A2

#define PIN_MOTION A3

#define PIN_IR_RT 10
#define PIN_IR_LT 11
#define PIN_IR_FR 12
#define PIN_IR_BK 13


#define PIN_UL_TRIG_0 5
#define PIN_UL_ECHO_0 6

#define POS_MIN 1250
#define POS_MAX 1750
#define POS_CEN 1500

#define ESC_MID 1500
#define ESC_MIN 1000
#define ESC_MAX 2000

#define NUM_LEDS 8

#define DANGER_DIST 50

RH_RF69 rf69(RFM69_CS, RFM69_INT);
RHReliableDatagram rf69_manager(rf69, RX_ADDR);
CRGB leds[NUM_LEDS];
LiquidCrystal_I2C lcd(0x27, 16, 2);

Servo savox, esc;


struct {
  int th;
  int st;  
} cmd;

char str1[128] = {0}, str2[32];

struct {
  IRsend *sender;
  char pin;
  char cmd;
} irs[4];

Ultrasonic ultraFront(PIN_UL_TRIG_0, PIN_UL_ECHO_0);	

void setup() {
  irs[0] = { sender: new IRsend(), pin: PIN_IR_RT, cmd: 'r' };
  irs[1] = { sender: new IRsend(), pin: PIN_IR_LT, cmd: 'l' };
  irs[2] = { sender: new IRsend(), pin: PIN_IR_FR, cmd: 'f' };
  irs[3] = { sender: new IRsend(), pin: PIN_IR_BK, cmd: 'b' };
  
  Serial.begin(115200);
  
  pinMode(LED, OUTPUT);
  pinMode(RFM69_RST, OUTPUT);
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

  savox.attach(PIN_SERVO, POS_MIN, POS_MAX);
  savox.writeMicroseconds(POS_CEN); 

  esc.attach(PIN_ESC, POS_MIN, POS_MAX);
  esc.writeMicroseconds(ESC_MID); 

  for (int i = 0; i < 4; i++) irs[i].sender->begin(irs[i].pin);

  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("No cmd!         ");
}

int iri = 0;

int distFront = 100, prevSs = POS_CEN, prevEs = ESC_MID;

void loop() {

  uint8_t len = sizeof(cmd);
  uint8_t from;
  bool upd = false;

  if (rf69_manager.recvfromAckTimeout((uint8_t *)&cmd, &len, 150, &from)) {

    int ss = map(cmd.st, -100, 100, POS_MIN, POS_MAX);
    if (cmd.th < 1) cmd.th /= 2;
    int es = map(cmd.th, -100, 100, ESC_MIN, ESC_MAX);

    if (distFront < DANGER_DIST && cmd.th > 0) {
      es = ESC_MID;
    }

    if (prevSs != ss) {
      savox.writeMicroseconds(ss);
      prevSs = ss;
      upd = true;
    }

    if (prevEs != es) {
      esc.writeMicroseconds(es);
      prevEs = es;
      upd = true;
    }
    
    if (upd) {
      lcd.setCursor(0, 1);
      sprintf(str1, "T:%03d   S:%03d", cmd.th, cmd.st);
      lcd.print(str1);
    }
  } else {
    if (prevEs != ESC_MID || prevSs != POS_CEN) {
      prevEs = ESC_MID;
      prevSs = POS_CEN;
      savox.writeMicroseconds(POS_CEN); 
      esc.writeMicroseconds(ESC_MID);
    } 
    lcd.setCursor(0, 1);
    lcd.print("No cmd!         ");
  }

  int d;
  d = ultraFront.read();

  if (distFront != d) {
    distFront = d;
    if (distFront < DANGER_DIST && cmd.th > 0 && prevEs != ESC_MID) {
        esc.writeMicroseconds(ESC_MID); 
    }

    lcd.setCursor(0, 0);
    sprintf(str1, "F:%03dcm   ", distFront);
    lcd.print(str1);
  }

  irs[iri].sender->sendNEC(0x0102, irs[iri].cmd, 0);
  iri++;
  if (iri == 4) iri = 0;

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(random(0, 255), random(0, 255), random(0, 255));
  }

  FastLED.show();

}

