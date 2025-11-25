#include <Arduino.h>

#define PWM_PIN 15   // AUX 信号输入（可改）
#define PRINT_MS 500 // 打印间隔

volatile uint32_t pwmHigh = 0;
volatile uint32_t lastRise = 0;

void IRAM_ATTR onEdge()
{
    uint32_t now = micros();
    if (digitalRead(PWM_PIN))
    { // 上升沿
        lastRise = now;
    }
    else
    {                             // 下降沿
        pwmHigh = now - lastRise; // 高电平持续时间
    }
}

void setup()
{
    Serial.begin(115200);
    pinMode(PWM_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PWM_PIN), onEdge, CHANGE);
    Serial.println("ESP32-S3 PWM Reader (AUX test)");
}

void loop()
{
    static uint32_t tmr = 0;
    if (millis() - tmr >= PRINT_MS)
    {
        tmr = millis();
        // 把 1000-2000 µs 映射成 0-100 %
        float pct = (pwmHigh - 1000) * 100.0f / 1000.0f;
        Serial.printf("High: %4d µs  |  %.1f %%\n", pwmHigh, pct);
    }
}