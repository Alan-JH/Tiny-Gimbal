/*
 * Tiny-Gimbal control code, for board v2.2
 * Alan Hsu
 * 29 September 2022
*/

#include "BNO055Constants.h"
#include <Servo.h> // Use MegaTinyCore implementation of Servo
#include <Wire.h>

Servo servoX;
Servo servoY;

// Servo PWM pin definitions
#define XPIN 0
#define YPIN 1

// Tunable values for center and remove position servo values. 
// Center centers each axis, while remove places the gimbal in a position where the camera is easy to remove
#define XCENTER 1500
#define YCENTER 1450
#define XREMOVE 1900
#define YREMOVE 1450

int x = XCENTER;
int y = YCENTER;

// Minimum and maximum allowable servo pulse width
#define MINPULSE 800
#define MAXPULSE 2200

// Pin definitions for center and remove buttons
#define CENTERPIN 2
#define REMOVEPIN 3

// Pin definitions for center and remove indicator LEDs
#define CENTERLED 10
#define REMOVELED 9

// Center and Remove Button Variables
bool center;
bool centerprevious = 1;
bool removecamera;
bool removeprevious = 1;

// PID Loop Parameters
float XKp = 1.6;
float XKd = 50000;
float XKi = 0;
float YKp = 2;
float YKd = 70000;
float YKi = 0;
long long lastreading; // in microseconds
float lastXError;
float lastYError;
float dEx; // Derivative
float dEy;
float intEx; // Integral
float intEy;

void setup() {
  // Set debug UART pins to default, initialize to 57600 baud
  Serial.pins(5, 4);
  Serial.begin(57600, (SERIAL_8N1));

  // Set button and LED indicator pin modes
  pinMode(CENTERPIN, INPUT_PULLUP);
  pinMode(REMOVEPIN, INPUT_PULLUP);
  pinMode(CENTERLED, OUTPUT);
  pinMode(REMOVELED, OUTPUT);

  // Initialize servos, write center position
  servoX.attach(XPIN);
  servoY.attach(YPIN);
  servoX.writeMicroseconds(x);
  servoY.writeMicroseconds(y);

  // Wait for servos to center, then initialize IMU
  delay(1000);
  if (!IMUBegin(OPERATION_MODE_NDOF)){
    while (1){ // If IMU fails to intialize, error message and do not proceed to loop
      Serial.println("Failed to initialize IMU");
      delay(1000);
    }
  } 

  // Set last reading variable for PID
  lastreading = micros();
}


void loop() {
  // Get Euler sensor values
  float euler[3];
  getEuler(euler);
  euler[1] = -1*euler[1]; // Y inversion
  //euler[2] = -1*euler[2]; // X inversion

  // If center button is pressed, toggle center bool and make sure remove is false so both arent true at once
  if (!digitalRead(CENTERPIN) && centerprevious){
    center = !center;
    removecamera = false;
  }

  // If remove button is pressed, toggle remove bool and make sure center is false so both arent true at once
  if (!digitalRead(REMOVEPIN) && removeprevious){
    removecamera = !removecamera;
    center = false;
  }
  centerprevious = digitalRead(CENTERPIN);
  removeprevious = digitalRead(REMOVEPIN);

  // Set LED indicators appropriately
  digitalWrite(REMOVELED, removecamera);
  digitalWrite(CENTERLED, center);

  if (center){ // If center is true, set x and y vals to centered PWM position
    x = XCENTER;
    y = YCENTER;
    intEx = 0; // Make sure integral doesn't go haywire while centered or removed
    intEy = 0;
    lastreading = micros();
  } else if (removecamera){ // If remove is true, set x and y vals to remove PWM position
    x = XREMOVE;
    y = YREMOVE;
    intEx = 0;
    intEy = 0;
    lastreading = micros();
  } else { // If neither center or remove, calculate PID
    float dt = micros() - lastreading;
    dEx = (euler[2] - lastXError);
    intEx += euler[2] * dt;
    x -= XKp*euler[2] + XKd * dEx/dt + XKi * intEx;

    dEy = (euler[1] - lastYError);
    intEy += euler[1] * dt;
    y -= YKp*euler[1] + YKd * dEy/dt + YKi * intEy;
    
    lastreading = micros();
    lastXError = euler[2];
    lastYError = euler[1];

    // Clamp servo PWM values to min and max
    x = min(max(x, MINPULSE), MAXPULSE);
    y = min(max(y, MINPULSE), MAXPULSE);
  }

  // Write to servos
  servoX.writeMicroseconds(x);
  servoY.writeMicroseconds(y);
  
  //Debug print
  Serial.print("SENSOR X: " + String(euler[2]));
  Serial.print(" Y: " + String(euler[1]));
  Serial.println();
}

// BNO055 Driver implementation:

bool IMUBegin(byte mode){
    Wire.begin();

    uint8_t id = read8(BNO055_CHIP_ID_ADDR);
    if (id != BNO055_ID){
        delay(1000);
        id = read8(BNO055_CHIP_ID_ADDR);
        if (id != BNO055_ID) {
            return false;
        }
    }

    setMode(OPERATION_MODE_CONFIG);

    /* Reset */
    write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
    delay(30);
    while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID) {
        delay(10);
    }
    delay(50);

    /* Set to normal power mode */
    write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    delay(10);

    write8(BNO055_PAGE_ID_ADDR, 0);

    write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
    delay(10);
    /* Set the requested operating mode (see section 3.3) */
    setMode(mode);
    delay(20);

    return true;
}

void setMode(byte mode){
    write8(BNO055_OPR_MODE_ADDR, mode);
    delay(30);
}

bool getEuler(float *buf){
    uint8_t wireBuf[6];
    memset(wireBuf, 0, 6);

    readLen(BNO055_EULER_H_LSB_ADDR, wireBuf, 6);

    buf[0] = (float)(((int16_t)wireBuf[0]) | (((int16_t)wireBuf[1]) << 8)) / 16.0;
    buf[1] = (float)(((int16_t)wireBuf[2]) | (((int16_t)wireBuf[3]) << 8)) / 16.0;
    buf[2] = (float)(((int16_t)wireBuf[4]) | (((int16_t)wireBuf[5]) << 8)) / 16.0;

    return true;
}

bool getQuat(float *buf){
    uint8_t wireBuf[8];
    memset(wireBuf, 0, 8);

    readLen(BNO055_EULER_H_LSB_ADDR, wireBuf, 8);

    const double scale = (1.0 / (1 << 14));
    double _w = (float)(((int16_t)wireBuf[0]) | (((int16_t)wireBuf[1]) << 8)) * scale;
    double _x = (float)(((int16_t)wireBuf[2]) | (((int16_t)wireBuf[3]) << 8)) * scale;
    double _y = (float)(((int16_t)wireBuf[4]) | (((int16_t)wireBuf[5]) << 8)) * scale;
    double _z = (float)(((int16_t)wireBuf[6]) | (((int16_t)wireBuf[7]) << 8)) * scale;

    //convert to euler
    double sqw = _w * _w;
    double sqx = _x * _x;
    double sqy = _y * _y;
    double sqz = _z * _z;

    buf[0] = atan2(2.0 * (_x * _y + _z * _w), (sqx - sqy - sqz + sqw))*180/PI;
    buf[1] = asin(-2.0 * (_x * _z - _y * _w) / (sqx + sqy + sqz + sqw))*180/PI;
    buf[2] = atan2(2.0 * (_y * _z + _x * _w), (-sqx - sqy + sqz + sqw))*180/PI;

    return true;
}

bool setRemap(byte remap, byte sign){
    write8(BNO055_AXIS_MAP_CONFIG_ADDR, remap);
    write8(BNO055_AXIS_MAP_SIGN_ADDR, sign);
    return true;
}

byte write8(byte reg, byte data){
    Wire.beginTransmission(BNO055_ADDRESS_A);
    Wire.write(reg);
    Wire.write(data);
    return Wire.endTransmission();
}

byte read8(byte reg){
    Wire.beginTransmission(BNO055_ADDRESS_A);
    Wire.write(reg);
    Wire.endTransmission();
    delay(5);
    Wire.requestFrom(BNO055_ADDRESS_A, 1);
    delay(5);
    if (Wire.available()){
        return Wire.read();
    }
    return 0;
}

bool readLen(byte reg, byte *buf, uint8_t len){
    Wire.beginTransmission(BNO055_ADDRESS_A);
    Wire.write(reg);
    Wire.endTransmission();
    delay(5);
    Wire.requestFrom(BNO055_ADDRESS_A, len);
    delay(5);
    for (int i = 0; i < len; i++){
        buf[i] = Wire.read();
    }
    return true;
}
