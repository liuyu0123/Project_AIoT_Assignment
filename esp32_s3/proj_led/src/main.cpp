#include <Arduino.h>

int LED_OUT = 2;
// 将LED正极介入GPIO2口，右边从上数第4个引脚。

void setup() {
  pinMode(LED_OUT, OUTPUT);
}

void loop() {
  digitalWrite(LED_OUT, LOW);
  delay(1000);
  digitalWrite(LED_OUT, HIGH);
  delay(1000);
}
