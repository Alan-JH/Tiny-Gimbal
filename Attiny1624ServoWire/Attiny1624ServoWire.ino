#include "BNO055Constants.h"
#include <Servo.h> // Use MegaTinyCore implementation of Servo
#include <Wire.h>

Servo servoX;
Servo servoY;

#define XPIN 0
#define YPIN 1

// Tunable values for center and remove position servo values
#define XCENTER 1500
#define YCENTER 1450
#define XREMOVE 1900
#define YREMOVE 1450

int x = XCENTER;
int y = YCENTER;

#define MINPULSE 800
#define MAXPULSE 2200

#define CENTERPIN 2
#define REMOVEPIN 3

// Center and Remove Button Variables
bool center;
bool centerprevious = 1;
bool removecamera;
bool removeprevious = 1;

// PD Loop Parameters
// Not implement integral because it would be a pain to implement with center and remove functionality
int Kp = 1;
int Kd = 100;
long long lastreading; // in microseconds
float lastXError;
float lastYError;

void setup() {
  Serial.pins(5, 4);
  Serial.begin(57600, (SERIAL_8N1)); // Debug UART
  servoX.attach(XPIN);
  servoY.attach(YPIN);
  servoX.writeMicroseconds(x);
  servoY.writeMicroseconds(y);
  pinMode(CENTERPIN, INPUT_PULLUP);
  pinMode(REMOVEPIN, INPUT_PULLUP);
  delay(1000);
  if (!IMUBegin(OPERATION_MODE_NDOF)){
    while (1){
      Serial.println("Failed to initialize IMU");
      delay(1000);
    }
  }
  lastreading = micros();
}


void loop() {
  float euler[3];
  getEuler(euler);
  euler[1] = -1*euler[1]; // Y
  //euler[2] = -1*euler[2]; // X inversion
  int cut = 0;
  int jerk = 0;
  int celerity = 120;

  if (!digitalRead(CENTERPIN) && centerprevious){
    center = !center;
  }
  centerprevious = digitalRead(CENTERPIN);

  if (!digitalRead(REMOVEPIN) && removeprevious){
    removecamera = !removecamera;
  }
  centerprevious = digitalRead(CENTERPIN);
  removeprevious = digitalRead(REMOVEPIN);

  if (center){
    x = XCENTER;
    y = YCENTER;
  } else if (removecamera){
    x = XREMOVE;
    y = YREMOVE;
  } else {
    dEx = (euler[2] - lastXError) / (micros() - lastreading);
    x += Kp*euler[2] + Kd * dEx;

    dEy = (euler[1] - lastYError) / (micros() - lastreading);
    y += Kp*euler[1] + Kd * dEy;
    
    lastreading = micros();
    lastXError = euler[2];
    lastYError = euler[1];
    
    Serial.print("SERVO X: " + String(x));
    Serial.print(" Y: " + String(y));
    Serial.println();
    x = min(max(x, MINPULSE), MAXPULSE);
    y = min(max(y, MINPULSE), MAXPULSE);
  }
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
