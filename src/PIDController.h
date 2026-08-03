#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

// ПИД-регулятор с поддержкой Feedforward и защитой от накопления ошибки (Anti-Windup)
class PIDController {
private:
    float Kp = 4.5f, Ki = 0.08f, Kd = 1.2f; // Коэффициенты регулятора
    float integral = 0.0f;                  // Накопленная ошибка
    float lastInput = 0.0f;                 // Предыдущая температура
    unsigned long lastTime = 0;

public:
    void setTunings(float p, float i, float d) { Kp = p; Ki = i; Kd = d; }

    // Расчет управляющего сигнала ШИМ (0..255)
    float compute(float setpoint, float input, float feedforward) {
        unsigned long now = millis();
        float dt = (now - lastTime) / 1000.0f;
        if (dt <= 0.0f) dt = 0.1f;
        lastTime = now;

        float error = setpoint - input;

        // Интегральная часть
        integral += error * dt;
        float iTerm = Ki * integral;

        // Защита от насыщения интегратора (Anti-Windup)
        if (iTerm < 0.0f) {
            iTerm = 0.0f;
            integral = 0.0f;
        } else if (iTerm > 255.0f) {
            iTerm = 255.0f;
            integral = 255.0f / (Ki > 0.001f ? Ki : 1.0f);
        }

        // Дифференциальная часть по изменению входа (без «ударов» при смене целевой T)
        float dInput = (input - lastInput) / dt;
        lastInput = input;

        // Итоговый ШИМ с учетом упреждающего сигнала (Feedforward)
        float output = (Kp * error) + iTerm - (Kd * dInput) + feedforward;
        return constrain(output, 0.0f, 255.0f);
    }

    void resetIntegral() { integral = 0.0f; }
};

#endif
