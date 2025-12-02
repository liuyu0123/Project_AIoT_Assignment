// 文件：main.cpp
// 功能：ESP32-S4 双有刷电机差速控制（PWM接收机 + L9110S驱动）
// L9110S特性：每路电机需要两个PWM引脚（IA/IB），3.3V兼容

#include <Arduino.h>

// ==================== 硬件配置（L9110S专用） ====================
// PWM输入引脚（连接接收机，需5V→3.3V分压）
#define PWM_THROTTLE_PIN 6 // 油门通道
#define PWM_STEER_PIN 7    // 转向通道

// L9110S驱动引脚（每路电机两个PWM输入）
#define LEFT_MOTOR_IA 1  // 左电机正转PWM
#define LEFT_MOTOR_IB 2  // 左电机反转PWM
#define RIGHT_MOTOR_IA 3 // 右电机正转PWM
#define RIGHT_MOTOR_IB 4 // 右电机反转PWM

// 参数配置
#define PWM_FREQ 10000   // L9110S建议10kHz（降低发热）
#define PWM_RESOLUTION 8 // 8位分辨率(0-255)
#define MAX_PWM 255      // 最大输出
#define DEADZONE 5       // 摇杆死区

// 遥控信号参数
#define PWM_MIN_US 1000
#define PWM_MAX_US 2000
#define PWM_CENTER_US 1500
#define SIGNAL_TIMEOUT_MS 500

// ==================== 全局变量 ====================
volatile uint16_t throttlePulseWidth = PWM_CENTER_US;
volatile uint16_t steerPulseWidth = PWM_CENTER_US;
volatile uint32_t throttleLastEdge = 0;
volatile uint32_t steerLastEdge = 0;

uint32_t lastSignalTime = 0;
bool signalLost = false;

// ==================== 中断服务函数 ====================
void ICACHE_RAM_ATTR onThrottlePWM()
{
    static uint32_t riseTime = 0;
    if (digitalRead(PWM_THROTTLE_PIN))
    {
        riseTime = micros();
    }
    else
    {
        throttlePulseWidth = micros() - riseTime;
    }
}

void ICACHE_RAM_ATTR onSteerPWM()
{
    static uint32_t riseTime = 0;
    if (digitalRead(PWM_STEER_PIN))
    {
        riseTime = micros();
    }
    else
    {
        steerPulseWidth = micros() - riseTime;
    }
}

// ==================== L9110S电机控制函数 ====================
void setupL9110S()
{
    // 配置4个PWM通道
    ledcSetup(0, PWM_FREQ, PWM_RESOLUTION); // 左电机IA
    ledcSetup(1, PWM_FREQ, PWM_RESOLUTION); // 左电机IB
    ledcSetup(2, PWM_FREQ, PWM_RESOLUTION); // 右电机IA
    ledcSetup(3, PWM_FREQ, PWM_RESOLUTION); // 右电机IB

    // 绑定到GPIO
    ledcAttachPin(LEFT_MOTOR_IA, 0);
    ledcAttachPin(LEFT_MOTOR_IB, 1);
    ledcAttachPin(RIGHT_MOTOR_IA, 2);
    ledcAttachPin(RIGHT_MOTOR_IB, 3);

    // 初始停止（全部写0）
    ledcWrite(0, 0);
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 0);

    Serial.println("L9110S驱动初始化完成");
}

// L9110S专用：设置单个电机速度
// speed: -255 ~ 255，正转/反转
void setL9110SPower(uint8_t channelIA, uint8_t channelIB, int16_t speed)
{
    speed = constrain(speed, -MAX_PWM, MAX_PWM);

    if (speed > 0)
    {
        // 正转：IA=速度, IB=0
        ledcWrite(channelIA, speed);
        ledcWrite(channelIB, 0);
    }
    else if (speed < 0)
    {
        // 反转：IA=0, IB=速度
        ledcWrite(channelIA, 0);
        ledcWrite(channelIB, -speed);
    }
    else
    {
        // 停止：全部写0
        ledcWrite(channelIA, 0);
        ledcWrite(channelIB, 0);
    }
}

// 差速控制
void updateMotorControl(int16_t throttle, int16_t steer)
{
    // 死区处理
    if (abs(throttle) < DEADZONE)
        throttle = 0;
    if (abs(steer) < DEADZONE)
        steer = 0;

    // 映射到PWM范围
    int16_t throttle_pwm = map(throttle, -100, 100, -MAX_PWM, MAX_PWM);
    int16_t steer_pwm = map(steer, -100, 100, -MAX_PWM, MAX_PWM);

    // 差速计算
    int16_t left_speed = throttle_pwm + steer_pwm / 2;
    int16_t right_speed = throttle_pwm - steer_pwm / 2;

    // 限幅
    left_speed = constrain(left_speed, -MAX_PWM, MAX_PWM);
    right_speed = constrain(right_speed, -MAX_PWM, MAX_PWM);

    // 输出到L9110S
    setL9110SPower(0, 1, left_speed);  // 左电机
    setL9110SPower(2, 3, right_speed); // 右电机

    Serial.printf("L:%d R:%d\n", left_speed, right_speed);
}

// ==================== PWM信号处理 ====================
int16_t mapPWMtoControl(uint16_t pulseWidth)
{
    if (pulseWidth < 800 || pulseWidth > 2200)
        return 0;
    return map(pulseWidth, PWM_MIN_US, PWM_MAX_US, -100, 100);
}

bool isSignalValid()
{
    bool valid = (throttlePulseWidth >= 800 && throttlePulseWidth <= 2200 &&
                  steerPulseWidth >= 800 && steerPulseWidth <= 2200);

    if (valid)
    {
        lastSignalTime = millis();
        return true;
    }

    if (millis() - lastSignalTime > SIGNAL_TIMEOUT_MS)
    {
        if (!signalLost)
        {
            signalLost = true;
            Serial.println("信号丢失！");
            // 紧急停止
            setL9110SPower(0, 1, 0);
            setL9110SPower(2, 3, 0);
        }
        return false;
    }
    return true;
}

// ==================== 初始化 ====================
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("=== ESP32-S3 + L9110S 差速控制器 ===");

    setupL9110S();

    pinMode(PWM_THROTTLE_PIN, INPUT);
    pinMode(PWM_STEER_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(PWM_THROTTLE_PIN), onThrottlePWM, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PWM_STEER_PIN), onSteerPWM, CHANGE);

    Serial.println("系统就绪");
}

// ==================== 主循环 ====================
void loop()
{
    if (!isSignalValid())
    {
        delay(10);
        return;
    }

    signalLost = false;

    uint16_t throttlePWM, steerPWM;
    noInterrupts();
    throttlePWM = throttlePulseWidth;
    steerPWM = steerPulseWidth;
    interrupts();

    int16_t throttle = mapPWMtoControl(throttlePWM);
    int16_t steer = mapPWMtoControl(steerPWM);

    updateMotorControl(throttle, steer);

    delay(20);
}