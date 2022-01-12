#include "BNO055Constants.h"
#include <Servo.h> // Use MegaTinyCore implementation of Servo
#include <Wire.h>

Servo servoP;
Servo servoR;
Servo servoY;

#define PITCHPIN 0
#define ROLLPIN 1
#define YAWPIN 10

#define ERRORPIN 4 // Pin gets pulled high in case of instantiation error

#define NOTIFPIN 2 // Pin gets pulled high when yaw is reset from 360 to 0, to indicate that camera is not ready.

void setup() {
  Serial.pins(5, 3);
  Serial.begin(57600, (SERIAL_8N1)); // Debug UART
  pinMode(ERRORPIN, OUTPUT); // Error pin, goes high if error state
  pinMode(NOTIFPIN, OUTPUT);
  digitalWrite(ERRORPIN, LOW);
  digitalWrite(NOTIFPIN, LOW);
  delay(1000);
  if (!IMUBegin(OPERATION_MODE_NDOF)){
    while (1){
      digitalWrite(ERRORPIN, HIGH);
      delay(100);
      digitalWrite(ERRORPIN, LOW);
      delay(100);
    }
  }
  servoP.attach(PITCHPIN);
  servoR.attach(ROLLPIN);
  servoY.attach(YAWPIN);
}

void loop() {
  float euler[3];
  getEuler(euler);
  //x = euler[0], y = euler[1], z = euler[2]
  
  //Comment out the following four lines in final implementation:
  Serial.print("X: " + String(euler[0]));
  Serial.print(" Y: " + String(euler[1]));
  Serial.print(" Z: " + String(euler[2]));
  Serial.println();
}

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
    buf[0] = (float)(((int16_t)wireBuf[0]) | (((int16_t)wireBuf[1]) << 8)) * scale;
    buf[1] = (float)(((int16_t)wireBuf[2]) | (((int16_t)wireBuf[3]) << 8)) * scale;
    buf[2] = (float)(((int16_t)wireBuf[4]) | (((int16_t)wireBuf[5]) << 8)) * scale;
    buf[3] = (float)(((int16_t)wireBuf[6]) | (((int16_t)wireBuf[7]) << 8)) * scale;

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
