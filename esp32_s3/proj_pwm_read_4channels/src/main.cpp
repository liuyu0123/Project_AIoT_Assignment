#include <Arduino.h>

#define PWM_PIN1  4
#define PWM_PIN2  5
#define PWM_PIN3  6
#define PWM_PIN4  7

constexpr uint32_t PERIOD_US = 20000;   // 50 Hz 周期

volatile uint32_t t1, t2, t3, t4;
volatile uint32_t pwm1, pwm2, pwm3, pwm4;

/* ---- 每个口一个 ISR ---- */
void IRAM_ATTR isr1() { uint32_t now = micros(); if (digitalRead(PWM_PIN1)) t1 = now; else pwm1 = now - t1; }
void IRAM_ATTR isr2() { uint32_t now = micros(); if (digitalRead(PWM_PIN2)) t2 = now; else pwm2 = now - t2; }
void IRAM_ATTR isr3() { uint32_t now = micros(); if (digitalRead(PWM_PIN3)) t3 = now; else pwm3 = now - t3; }
void IRAM_ATTR isr4() { uint32_t now = micros(); if (digitalRead(PWM_PIN4)) t4 = now; else pwm4 = now - t4; }

void setup() {
  Serial.begin(115200);
  pinMode(PWM_PIN1, INPUT_PULLDOWN);
  pinMode(PWM_PIN2, INPUT_PULLDOWN);
  pinMode(PWM_PIN3, INPUT_PULLDOWN);
  pinMode(PWM_PIN4, INPUT_PULLDOWN);

  attachInterrupt(digitalPinToInterrupt(PWM_PIN1), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PWM_PIN2), isr2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PWM_PIN3), isr3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PWM_PIN4), isr4, CHANGE);
}

void loop() {
  static uint32_t t0;
  if (millis() - t0 > 200) {
    t0 = millis();

    // [capture](parameters) -> return_type { body }
    auto toDuty = [](float pwm) -> float {return float (pwm - 1000) * 100.0f / 1000.0f;};
    // float duty1 = (float)pwm1 / PERIOD_US * 100.0f;
    // float duty2 = (float)pwm2 / PERIOD_US * 100.0f;
    // float duty3 = (float)pwm3 / PERIOD_US * 100.0f;
    // float duty4 = (float)pwm4 / PERIOD_US * 100.0f;

    float duty1 = toDuty(pwm1);
    float duty2 = toDuty(pwm2);
    float duty3 = toDuty(pwm3);
    float duty4 = toDuty(pwm4);

    Serial.printf("CH1:%4lu µs %.1f%% | CH2:%4lu µs %.1f%% | CH3:%4lu µs %.1f%% | CH4:%4lu µs %.1f%%\n",
                  pwm1, duty1, pwm2, duty2, pwm3, duty3, pwm4, duty4);
  }
}