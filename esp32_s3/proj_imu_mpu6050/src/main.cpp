#include <Arduino.h>
#include "Wire.h"
#include <MPU6050_light.h>

// MPU6050供电3.3V

#define I2C_SDA 18
#define I2C_SCL 19
// 18-19是ESP32-S3上的硬件IIC

MPU6050 mpu(Wire);
unsigned long timer = 0;


void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA,I2C_SCL);
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}

void loop() {
  mpu.update();
  
  if((millis()-timer)>10){ // print data every 10ms
	Serial.print("X : ");
	Serial.print(mpu.getAngleX());
	Serial.print("\tY : ");
	Serial.print(mpu.getAngleY());
	Serial.print("\tZ : ");
	Serial.println(mpu.getAngleZ());
	timer = millis();  
  }
}
