#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
char str1[128] = {0}, str2[32];

#define WINDOW_SIZE 10
float readings[WINDOW_SIZE]; // Array to store the last 'n' readings
int readIndex = 0; // Index of the current reading
float total = 0; // Running total of the readings
float average = 0; // The average of the readings
unsigned long prevsp = 1;
int sensorValue = 0, a; 
float reading;

// Initialize the readings array to 0
void initReadings() {
    for (int i = 0; i < WINDOW_SIZE; i++) {
        readings[i] = 0.0;
    }
}

// Function to add a new reading and compute the moving average
float addReading(float newReading) {
    // Subtract the oldest reading from total and replace it with the new reading
    total = total - readings[readIndex];
    readings[readIndex] = newReading;
    total = total + newReading;
    
    // Advance to the next position in the array
    readIndex = (readIndex + 1) % WINDOW_SIZE;
    
    // Calculate the average
    average = total / WINDOW_SIZE;
    
    return average;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  initReadings();

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

  Serial.println("SSD1306 init success");

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.cp437(true);
  display.setTextSize(2);
  display.println("Racer RX");
  display.setTextSize(1);
  display.println("");
  display.println(str1);
  display.println("");
  display.println("v0.1");
  display.display();

  delay(1000);
}


void loop() {

  sensorValue = analogRead(PIN_FRC);
  reading = (pow(10, sensorValue / 1024.0) - 1.0) * 20.0;
  reading = (pow(10, reading / 200.0) - 1.0) * 20.0;
  reading = (pow(10, reading / 200.0) - 1.0) * 20.0;
  reading = (pow(10, reading / 200.0) - 1.0) * 12.0;

  unsigned long sp = 0;

  //reading /= 2.0;
  if (reading > 15) reading = 15;
  reading = addReading(reading);

  if (reading > 1) {
    sp = pow(2, int(reading)) - 1;
  } else {
    sp = reading;
  }

  if (sp >= 1024) sp = 1023;

  if (prevsp != sp) {
    digitalWrite(PIN_LATCH, LOW);
    shiftOut(PIN_DATA, PIN_CLK, MSBFIRST, 0xff & (sp >> 8));
    shiftOut(PIN_DATA, PIN_CLK, MSBFIRST, 0xff & sp);
    digitalWrite(PIN_LATCH, HIGH);
    prevsp = sp;

    // Serial.print(sensorValue);
    // Serial.print(":");
    // Serial.println(sp);
  }

  sensorValue = analogRead(PIN_VRX);
  
  display.clearDisplay();

  int vrx, vry, vsw;

  vrx = analogRead(PIN_VRX);
  vry = analogRead(PIN_VRY);
  vsw = digitalRead(PIN_VSW);

  sprintf(str1, "x:%d", vrx);
  display.setCursor(0, 10);
  display.print(str1);

  sprintf(str1, "y:%d", vry);
  display.setCursor(45, 10);
  display.print(str1);

  sprintf(str1, "s:%d", vsw);
  display.setCursor(90, 10);
  display.print(str1);

  display.display();

  delay(5);

}

