#include "nav_pid.h"
#include <algorithm>
#include <cmath>

NavPIDController::NavPIDController(float kp, float ki, float kd, 
                                   float out_min, float out_max, 
                                   float int_min, float int_max)
    : Kp(kp), Ki(ki), Kd(kd), 
      output_min(out_min), output_max(out_max),
      integral_min(int_min), integral_max(int_max),
      _integral_term(0.0f), _prev_error(0.0f), _first_calculation(true) {}

void NavPIDController::reset() {
    _integral_term = 0.0f;
    _prev_error = 0.0f;
    _first_calculation = true;
}

float NavPIDController::calculate(float setpoint, float current_value, float dt) {
    // Controllo di sicurezza sul dt
    if (dt <= 0.0f || std::isinf(dt) || std::isnan(dt)) {
        return 0.0f;
    }

    float error = setpoint - current_value;

    // Componente Proporzionale
    float p_term = Kp * error;

    // Componente Integrale (con Anti-Windup)
    _integral_term += Ki * error * dt;
    _integral_term = constrain(_integral_term, integral_min, integral_max);

    // Componente Derivata
    float d_term = 0.0f;
    if (!_first_calculation) {
        d_term = Kd * (error - _prev_error) / dt;
    } else {
        _first_calculation = false;
    }

    // Salvataggio errore per il loop successivo
    _prev_error = error;

    // Somma e saturazione dell'output (Clamp)
    float output = p_term + _integral_term + d_term;
    return constrain(output, output_min, output_max);
}