#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
private:
    float Kp = 4.5f, Ki = 0.08f, Kd = 1.2f;
    float integral = 0.0f;
    float lastInput = 0.0f;
    unsigned long lastTime = 0;

public:
    void setTunings(float p, float i, float d) { Kp = p; Ki = i; Kd = d; }

    float compute(float setpoint, float input, float feedforward) {
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        if (dt <= 0.0f) dt = 0.1f;
        lastTime = now;

        float error = setpoint - input;
        integral += error * dt;
        float iTerm = Ki * integral;

        // Anti-windup
        if (iTerm < 0.0f) {
            iTerm = 0.0f;
            integral = 0.0f;
        } else if (iTerm > 255.0f) {
            iTerm = 255.0f;
            integral = 255.0f / (Ki > 0.001f ? Ki : 1.0f);
        }

        float dInput = (input - lastInput) / dt;
        lastInput = input;

        float output = (Kp * error) + iTerm - (Kd * dInput) + feedforward;
        return constrain(output, 0.0f, 255.0f);
    }

    void resetIntegral() { integral = 0.0f; }
};

#endif
