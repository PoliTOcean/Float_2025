// nav_pid.h
#pragma once
#include <Arduino.h>

class NavPIDController {
public:
    float Kp, Ki, Kd;
    float output_min, output_max;
    float integral_min, integral_max;

    // Aggiornato per corrispondere al file .cpp
    NavPIDController(float kp, float ki, float kd, 
                     float out_min, float out_max, 
                     float int_min, float int_max);

    void reset();
    float calculate(float setpoint, float current_value, float dt);

private:
    float _integral_term;
    float _prev_error;
    bool  _first_calculation;
};