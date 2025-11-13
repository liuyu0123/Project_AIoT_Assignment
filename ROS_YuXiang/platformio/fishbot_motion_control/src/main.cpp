#include <Arduino.h>
#include <Esp32McpwmMotor.h>
#include <Esp32PcntEncoder.h>
#include <PidController.h>
#include <Kinematics.h>

Esp32PcntEncoder encoders[2];
Esp32McpwmMotor motor; // 创建一个名为motor的对象，用于控制电机
PidController pid_controller[2];
Kinematics kinematics;

int64_t last_ticks[2];    // 记录上次读取的计数器数值
int32_t delta_ticks[2];   // 记录两次读取计数器之间的差值
int64_t last_update_time; // 记录上次更新速度的时间
float current_speeds[2];  // 记录电机的当前速度

float target_linear_speed = 50.0f;
float target_angle_speed = 0.1f;
float out_left_speed;
float out_right_speed;

void motorSpeedControl()
{
    uint64_t dt = millis() - last_update_time;

    delta_ticks[0] = encoders[0].getTicks() - last_ticks[0];
    delta_ticks[1] = encoders[1].getTicks() - last_ticks[1];

    current_speeds[0] = float(delta_ticks[0] * 0.1051566) / dt;
    current_speeds[1] = float(delta_ticks[1] * 0.1051566) / dt;

    last_update_time = millis();
    last_ticks[0] = encoders[0].getTicks();
    last_ticks[1] = encoders[1].getTicks();

    motor.updateMotorSpeed(0, pid_controller[0].update(current_speeds[0]));
    motor.updateMotorSpeed(1, pid_controller[1].update(current_speeds[1]));

    Serial.printf("speed=%fm/s, speed=%fm/s\n", current_speeds[0], current_speeds[1]);
}

void setup()
{
    Serial.begin(115200);         // 初始化串口通信，波特率为115200
    encoders[0].init(0, 32, 33);  // 初始化编码器0
    encoders[1].init(1, 26, 25);  // 初始化编码器1
    motor.attachMotor(0, 22, 23); // 将电机0连接到引脚22和引脚23
    motor.attachMotor(1, 12, 13); // 将电机1连接到引脚12和引脚13
    // motor.updateMotorSpeed(0, 70);
    // motor.updateMotorSpeed(1, 70);
    pid_controller[0].update_pid(0.625, 0.125, 0.0);
    pid_controller[1].update_pid(0.625, 0.125, 0.0);

    pid_controller[0].out_limit(-100, 100);
    pid_controller[1].out_limit(-100, 100);

    kinematics.set_wheel_distance(175);
    kinematics.set_motor_param(0, 0.1051566);
    kinematics.set_motor_param(1, 0.1051566);

    kinematics.kinematics_inverse(target_linear_speed, target_angle_speed, out_left_speed, out_right_speed);

    pid_controller[0].update_target(out_left_speed);
    pid_controller[1].update_target(out_right_speed);
}

void loop()
{
    delay(10);
    kinematics.update_motor_speed(millis(), encoders[0].getTicks(), encoders[1].getTicks());
    // motorSpeedControl();
    motor.updateMotorSpeed(0, pid_controller[0].update(kinematics.get_motor_speed(0)));
    motor.updateMotorSpeed(1, pid_controller[1].update(kinematics.get_motor_speed(1)));
}