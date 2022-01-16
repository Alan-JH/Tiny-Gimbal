#include "BNO055Constants.h"
#include <Servo.h> // Use MegaTinyCore implementation of Servo
#include <Wire.h>

Servo servoX;
Servo servoY;
Servo servoZ;
int x = 1500;
int y = 1500;
int z = 1500;
int xbusy; //millisecond timestamp of when the x servo was last told to spin around one rotation
float yawTotal;

#define XPIN 0
#define YPIN 1
#define ZPIN 10

#define MINPULSE 600
#define MAXPULSE 2400

#define SPINDELAY 1500

#define ERRORPIN 4 // Pin gets pulled high in case of instantiation error

#define NOTIFPIN 2 // Pin gets pulled high when yaw is reset from 360 to 0, to indicate that camera is not ready.

void setup() {
  Serial.pins(5, 3);
  Serial.begin(57600, (SERIAL_8N1)); // Debug UART
  pinMode(ERRORPIN, OUTPUT); // Error pin, goes high if error state
  pinMode(NOTIFPIN, OUTPUT); // Notifies pi when camera is spinning around one rotation
  digitalWrite(ERRORPIN, LOW);
  digitalWrite(NOTIFPIN, LOW);
  servoX.attach(XPIN);
  servoY.attach(YPIN);
  servoZ.attach(ZPIN);
  servoX.writeMicroseconds(x);
  servoY.writeMicroseconds(y);
  servoZ.writeMicroseconds(z);
  delay(1000);
  if (!IMUBegin(OPERATION_MODE_NDOF)){
    while (1){
      digitalWrite(ERRORPIN, HIGH);
      delay(100);
      digitalWrite(ERRORPIN, LOW);
      delay(100);
    }
  }
  delay(2000);
}

void loop() {
  float euler[3];
  getEuler(euler);
  //x = euler[0], y = euler[1], z = euler[2]
  yawTotal = euler[0];
  if (euler[0] > 180){
    yawTotal -= 360;
  }
  int cut = 2;
  int jerk = 0;
  int celerity = 100;
  if(millis() - xbusy > SPINDELAY && yawTotal>=cut)
  {
    x+=map(yawTotal,cut,180,jerk,celerity/2);
  }
  else if(millis() - xbusy > SPINDELAY && yawTotal<=-cut)
  {
    x-=map(yawTotal,-180,-cut,celerity/2,jerk);
  }
  
  if(euler[1]>=cut)
  {
    y-=map(euler[1],cut,90,jerk,celerity);
  }
  else if(euler[1]<=-cut)
  {
    y+=map(euler[1],-90,-cut,celerity,jerk);
  }
  if(euler[2]>=cut)
  {
    z-=map(euler[2],cut,90,jerk,celerity);
  }
  else if(euler[2]<=-cut)
  {
    z+=map(euler[2],-90,-cut,celerity,jerk);
  }
  Serial.print("SERVO X: " + String(x));
  Serial.print(" Y: " + String(y));
  Serial.print(" Z: " + String(z));
  Serial.println();
  if(x<MINPULSE){x=MINPULSE + 1333; servoX.writeMicroseconds(x); digitalWrite(NOTIFPIN, HIGH); xbusy = millis(); }
  if(x>MAXPULSE){x=MAXPULSE - 1333; servoX.writeMicroseconds(x); digitalWrite(NOTIFPIN, HIGH); xbusy = millis(); }
  if(y<MINPULSE){y=MINPULSE;}
  if(y>MAXPULSE){y=MAXPULSE;}
  if(z<MINPULSE){z=MINPULSE;}
  if(z>MAXPULSE){z=MAXPULSE;}
  if (millis() - xbusy > SPINDELAY ){
    digitalWrite(NOTIFPIN, LOW);
    servoX.writeMicroseconds(x);
  }
  servoY.writeMicroseconds(y);
  servoZ.writeMicroseconds(z);
  //Debug print
  Serial.print("SENSOR X: " + String(euler[0]));
  Serial.print(" Y: " + String(euler[1]));
  Serial.print(" Z: " + String(euler[2]));
  Serial.print(" YAWTOTAL: " + String(yawTotal));
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
