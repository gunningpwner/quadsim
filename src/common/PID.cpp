#include "PID.h"

PID::PID(float kp, float ki, float kd, float integral_saturation_limit)
    : m_kp(kp),
      m_ki(ki),
      m_kd(kd),
      m_integral_sum(0.0f),
      m_previous_error(0.0f),
      m_first_run(true),
      m_integral_saturation_limit(integral_saturation_limit) {}

float PID::calculate(float error, float dt) {
    if (m_first_run) {
        m_previous_error = error;
        m_first_run = false;
    }

    // Proportional term
    float p_term = m_kp * error;

    // Integral term
    
    m_integral_sum += error * dt;
    
    // Clamp Integral to stop wind up
    if (m_integral_saturation_limit != 0.0f){
        m_integral_sum = std::clamp(m_integral_sum, -m_integral_saturation_limit, m_integral_saturation_limit);
    }
    
    float i_term = m_ki * m_integral_sum;

    // Derivative term
    float d_term = m_kd * ((error - m_previous_error) / dt);

    // Update previous error
    m_previous_error = error;

    return p_term + i_term + d_term;
}

void PID::reset() {
    m_integral_sum = 0.0f;
    m_previous_error = 0.0f;
    m_first_run = true;
}