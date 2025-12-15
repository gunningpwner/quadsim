#pragma once

#include <cstdint>
#include <algorithm> // For std::clamp

class PID {
public:
    PID(float kp, float ki, float kd, float integral_saturation_limit = 0.0f);

    float calculate(float error, float dt);
    void reset();

private:
    float m_kp;
    float m_ki;
    float m_kd;
    float m_integral_sum;
    float m_previous_error;
    bool m_first_run;
    float m_integral_saturation_limit;
};