#include "Kinematics.h"

void Kinematics::set_motor_param(uint8_t id, float per_pulse_distance)
{ 
    motor_param_[id].per_pulse_distance = per_pulse_distance;
}

void Kinematics::set_wheel_distance(float wheel_distance)
{ 
    this->wheel_distance_ = wheel_distance;
}

int16_t Kinematics::get_motor_speed(uint8_t id)
{ 
    return motor_param_[id].motor_speed;
}


void Kinematics::update_motor_speed(uint64_t current_time, int32_t left_tick, int32_t right_tick)
{ 
    uint32_t dt = current_time - last_update_time_;
    last_update_time_ = current_time;

    int32_t dtick1 = left_tick - motor_param_[0].last_encoder_tick;
    int32_t dtick2 = right_tick - motor_param_[1].last_encoder_tick;
    motor_param_[0].last_encoder_tick = left_tick;
    motor_param_[1].last_encoder_tick = right_tick;

    motor_param_[0].motor_speed = float(dtick1 * motor_param_[0].per_pulse_distance) / dt * 1000;
    motor_param_[1].motor_speed = float(dtick1 * motor_param_[1].per_pulse_distance) / dt * 1000;
}


void Kinematics::kinematics_forward(float left_speed, float right_speed,
                                    float &out_linear_speed, float &out_angle_speed)
{ 
    out_linear_speed = (right_speed + left_speed) * 0.5;
    out_angle_speed = (right_speed + left_speed) / wheel_distance_;
}


void Kinematics::kinematics_inverse(float linear_speed, float angle_speed,
                                    float &out_left_speed, float &out_right_speed)
{ 
    out_left_speed = linear_speed - angle_speed * wheel_distance_ * 0.5;
    out_right_speed = linear_speed + angle_speed * wheel_distance_ * 0.5;
}