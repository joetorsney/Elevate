/**
 * Sets up the MPU6050
 * Sends roll, pitch, yaw angles on the serial bus.
 */ 

#include <Wire.h>

const int IMU_addr = 0x68;
int16_t AccX,AccY,AccZ,Temp,GyroX,GyroY,GyroZ;

void setup() {
    Wire.begin();
    Wire.beginTransmission(IMU_addr);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
    Serial.begin(9600);
}

void loop() {
    Wire.beginTransmission(IMU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    Wire.requestFrom(IMU_addr, 14, true);
    AccX=Wire.read()<<8|Wire.read();
    AccY=Wire.read()<<8|Wire.read();
    AccZ=Wire.read()<<8|Wire.read();
    Temp=Wire.read()<<8|Wire.read();
    GyroX=Wire.read()<<8|Wire.read();
    GyroY=Wire.read()<<8|Wire.read();
    GyroZ=Wire.read()<<8|Wire.read();
    Serial.print("AccX = "); Serial.print(AccX);
    Serial.print(" || AccY = "); Serial.print(AccY);
    Serial.print(" || AccZ = "); Serial.print(AccZ);
    Serial.print(" || Temp = "); Serial.print(Temp/340.00+36.53);
    Serial.print(" || GyroX = "); Serial.print(GyroX);
    Serial.print(" || GyroY = "); Serial.print(GyroY);
    Serial.print(" || GyroZ = "); Serial.println(GyroZ);
    delay(250);
}