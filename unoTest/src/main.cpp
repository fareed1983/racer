/*
 * Ultrasonic Simple
 * Prints the distance read by an ultrasonic sensor in
 * centimeters. They are supported to four pins ultrasound
 * sensors (liek HC-SC04) and three pins (like PING)))
 * and Seeed Studio sensors).
 *
 * The circuit:
 * * Module HR-SC04 (four pins) or PING))) (and other with
 *   three pins), attached to digital pins as follows:
 * ---------------------    --------------------
 * | HC-SC04 | Arduino |    | 3 pins | Arduino |
 * ---------------------    --------------------
 * |   Vcc   |   5V    |    |   Vcc  |   5V    |
 * |   Trig  |   12    | OR |   SIG  |   13    |
 * |   Echo  |   13    |    |   Gnd  |   GND   |
 * |   Gnd   |   GND   |    --------------------
 * ---------------------
 * Note: You do not obligatorily need to use the pins defined above
 * 
 * By default, the distance returned by the read()
 * method is in centimeters. To get the distance in inches,
 * pass INC as a parameter.
 * Example: ultrasonic.read(INC)
 *
 * created 3 Apr 2014
 * by Erick Simões (github: @ErickSimoes | twitter: @AloErickSimoes)
 * modified 23 Jan 2017
 * by Erick Simões (github: @ErickSimoes | twitter: @AloErickSimoes)
 * modified 03 Mar 2017
 * by Erick Simões (github: @ErickSimoes | twitter: @AloErickSimoes)
 * modified 11 Jun 2018
 * by Erick Simões (github: @ErickSimoes | twitter: @AloErickSimoes)
 *
 * This example code is released into the MIT License.
 */
#include <Arduino.h>
#include <Ultrasonic.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define PIN_DATA 2
#define PIN_LATCH 4
#define PIN_CLK 3

#define PIN_VRX A1
#define PIN_VRY A2
#define PIN_VSW A3

#define POS_MIN 1150
#define POS_MAX 1750
#define POS_CEN (POS_MAX-POS_MIN)/2+POS_MIN

#define ADDR_MPU 0x68
#define ADDR_OLED 0x3C

/*
 * Pass as a parameter the trigger and echo pin, respectively,
 * or only the signal pin (for sensors 3 pins), like:
 * Ultrasonic ultrasonic(13);
 */
Ultrasonic ultrasonic(12, 13);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

char line[50];

int pos = POS_MIN, inc = 50;
int distance = 0, vrx, vry, sw = 0;
int sensorValue = 0; 
unsigned long prevsp = 1;
float elapsedTime, currentTime, previousTime;

Servo savox;

// Define the size of the moving average window
#define WINDOW_SIZE 10
float readings[WINDOW_SIZE]; // Array to store the last 'n' readings
int readIndex = 0; // Index of the current reading
float total = 0; // Running total of the readings
float average = 0; // The average of the readings

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
  Wire.begin();

  pinMode(PIN_LATCH, OUTPUT);
  pinMode(PIN_DATA, OUTPUT);
  pinMode(PIN_CLK, OUTPUT);

  initReadings();

  byte error, address;
  int nDevices;
  Serial.begin(115200);

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
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


  savox.attach(9, POS_MIN, POS_MAX);
  savox.writeMicroseconds(POS_CEN); 

  // from https://projecthub.arduino.cc/Nicholas_N/how-to-use-the-accelerometer-gyroscope-gy-521-647e65
  Wire.beginTransmission(ADDR_MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  if(!display.begin(SSD1306_SWITCHCAPVCC, ADDR_OLED)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  display.clearDisplay();

  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  
  display.display();

}


void loop() {

  distance = ultrasonic.read();

  display.clearDisplay();
  display.setTextSize(1);

  sprintf(line, "D:%dcm", distance);
  display.setCursor(0, 0);
  display.print(line);

  // if (distance < 20) {
  //   display.setCursor(80, 0);
  //   display.print("ALERT!");
  // }

  // if (distance < 20) {
  //   display.setTextSize(1);
  //   display.setCursor(20, 20);
  //   display.println("ALERT!");
  //   pos = POS_CEN;
  // } else {
  //   pos += inc;
  //   if (pos <= POS_MIN || pos >= POS_MAX) inc *= -1;
  // }


  sensorValue = analogRead(A0);
  float reading = (pow(10, sensorValue / 860.0) - 1.0) * 20.0;
  reading = (pow(10, reading / 200.0) - 1.0) * 20.0;
  reading = (pow(10, reading / 200.0) - 1.0) * 20.0;
  reading = (pow(10, reading / 200.0) - 1.0) * 20.0;
  reading = (pow(10, reading / 200.0) - 1.0) * 10.0;

  //reading /= 2.0;
  if (reading > 15) reading = 15;
  reading = addReading(reading);

  //sprintf(line, "Sen: %d, %d, %d", linearValue3, (int)linearValue2, (int)linearValue);

  unsigned long sp = 0;
  
  if (reading > 2) {
      sp = pow(2, int(reading));
  } else if ((int)reading == 2) {
    sp = 3;
  } else {
    sp = reading;
  }

  if (sp >= 1024) sp = 1023;

  // if (sp > 0) {
  //   Serial.print(reading);
  //   Serial.print(":");
  //   Serial.println(sp);
  // }

  if (prevsp != sp) {
    digitalWrite(PIN_LATCH, LOW);
    shiftOut(PIN_DATA, PIN_CLK, MSBFIRST, 0xff & (sp >> 8));
    shiftOut(PIN_DATA, PIN_CLK, MSBFIRST, 0xff & sp);
    digitalWrite(PIN_LATCH, HIGH);
    prevsp = sp;
  }


    //sprintf(line, "Deg: %d, Inc: %d", pos, inc);
  sprintf(line, "S:%d", (int)reading);
  display.setCursor(64, 0);
  display.print(line);

  vrx = analogRead(PIN_VRX);
  vry = analogRead(PIN_VRY);

  sprintf(line, "x:%d", vrx);
  display.setCursor(0, 10);
  display.print(line);

  sprintf(line, "y:%d", vry);
  display.setCursor(45, 10);
  display.print(line);

  sprintf(line, "s:%d", analogRead(PIN_VSW));
  display.setCursor(90, 10);
  display.print(line);


  Wire.beginTransmission(ADDR_MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(ADDR_MPU, 14, true); // Read 6 registers total, each axis value is stored in 2 registers

  float AccX, AccY, AccZ;
  float GyroX, GyroY, GyroZ;
  float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
  float roll, pitch, yaw;
  float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;

  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)

  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds

  display.display();

  savox.writeMicroseconds(pos); 

  delay(5);
}