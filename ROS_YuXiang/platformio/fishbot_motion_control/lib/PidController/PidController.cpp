#include "PidController.h"
#include "Arduino.h"

PidController::PidController(float kp, float ki, float kd)
{
    reset();
    update_pid(kp, ki, kd);
}

float PidController::update(float current)
{
    float error = target_ - current;
    derror_ = error_last_ - error;
    error_last_ = error;

    error_sum_ += error;
    if (error_sum_ > intergral_up_)
        error_sum_ = intergral_up_;
    if (error_sum_ < -1 * intergral_up_)
        error_sum_ = -1 * intergral_up_;

    float output = kp_ * error + ki_ * error_sum_ + kd_ * derror_;

    if (output > out_max_)
        output = out_max_;
    if (output < out_min_)
        output = out_min_;

    return output;
}

void PidController::update_target(float target)
{ 
    target_ = target;
}

void PidController::update_pid(float kp, float ki, float kd)
{
    reset();
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PidController::reset()
{
    target_ = 0;
    out_max_ = 0;
    out_min_ = 0;
    kp_ = 0;
    ki_ = 0;
    kd_ = 0;
    error_sum_ = 0;
    error_last_ = 0;
    derror_ = 0;
}

void PidController::out_limit(float out_min, float out_max)
{
    out_min_ = out_min;
    out_max_ = out_max;
}