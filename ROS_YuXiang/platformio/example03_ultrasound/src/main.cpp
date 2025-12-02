#include <Arduino.h>
#define TRIG 27
#define ECHO 21

void setup() {
  Serial.begin(115200);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
}

void loop() {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  double delta_time = pulseIn(ECHO, HIGH);
  float detect_distance = delta_time * 0.0343 / 2; //声速=340m/s
  Serial.printf("距离：%f cm\n", detect_distance);
  delay(1000);
}
