#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include <Arduino.h>

typedef struct
{
    float per_pulse_distance;
    int16_t motor_speed;
    int64_t last_encoder_tick;
} motor_param_t;

typedef struct
{
    float x;
    float y;
    float angle;
    float linear_speed;
    float angle_speed;
} odom_t;


class Kinematics
{
public:
    Kinematics() = default;

    void set_motor_param(uint8_t id, float per_pulse_distance);

    void set_wheel_distance(float wheel_distance);

    void kinematics_inverse(float linear_speed, float angle_speed,
                            float &out_linear_speed, float &out_angle_speed);

    void kinematics_forward(float left_speed, float right_speed,
                            float &out_linear_speed, float &out_angle_speed);

    void update_motor_speed(uint64_t current_time, int32_t left_tick, int32_t right_tick);

    int16_t get_motor_speed(uint8_t id);

    void update_odom(uint16_t dt);

    odom_t &get_odom();

    static void TransAngleInPI(float angle, float &out_angle);

private:
    motor_param_t motor_param_[2];
    uint64_t last_update_time_;
    float wheel_distance_;
    odom_t odom_;
};

#endif