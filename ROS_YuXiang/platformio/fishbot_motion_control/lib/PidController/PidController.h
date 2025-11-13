#ifndef __PIDCONTROLLER_H__
#define __PIDCONTROLLER_H__

class PidController
{
public:
    PidController() = default;
    PidController(float kp, float ki, float kd);

private:
    float target_;
    float out_min_;
    float out_max_;
    float kp_, ki_, kd_;
    //pid
    float error_sum_;
    float error_last_;
    float derror_;
    float error_pre_;
    float intergral_up_ = 2500;

public:
    float update(float current);
    void update_target(float target);
    void update_pid(float kp, float ki, float kd);
    void reset();
    void out_limit(float out_min, float out_max);
};

#endif