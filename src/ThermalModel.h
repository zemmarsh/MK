#ifndef THERMAL_MODEL_H
#define THERMAL_MODEL_H

#include <Arduino.h>

// Адаптивная модель теплопотерь на основе рекурсивного МНК (RLS)
class ThermalModel {
private:
    float K_loss = 2.50f;  // Оцениваемый коэффициент теплопотерь
    float lambda = 0.995f; // Коэффициент «забывания» старых данных
    float P = 1000.0f;     // Ковариация (уверенность модели)

public:
    // Обучение модели: уточнение коэффициента K_loss прямо во время работы
    void updateRLS(float deltaT, float actualPWM) {
        if (deltaT <= 1.0f) return;

        float K = (P * deltaT) / (lambda + deltaT * P * deltaT);
        float estimatedPWM = K_loss * deltaT;
        float error = actualPWM - estimatedPWM; // Ошибка предсказания

        K_loss = K_loss + K * error;
        P = (P - K * deltaT * P) / lambda;

        K_loss = constrain(K_loss, 0.5f, 5.0f); // Физические ограничения
    }

    // Вычисление упреждающей мощности (Feedforward) по разнице температур
    float getFeedforward(float setpoint, float outdoorTemp) {
        float deltaT = setpoint - outdoorTemp;
        return (deltaT > 0.0f) ? (K_loss * deltaT) : 0.0f;
    }
};

#endif
