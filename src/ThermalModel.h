#ifndef THERMAL_MODEL_H
#define THERMAL_MODEL_H

#include <Arduino.h>

class ThermalModel {
private:
    float K_loss = 2.50f;
    float lambda = 0.995f;
    float P = 1000.0f;

public:
    void updateRLS(float deltaT, float actualPWM) {
        if (deltaT <= 1.0f) return;

        float K = (P * deltaT) / (lambda + deltaT * P * deltaT);
        float estimatedPWM = K_loss * deltaT;
        float error = actualPWM - estimatedPWM;

        K_loss = K_loss + K * error;
        P = (P - K * deltaT * P) / lambda;

        K_loss = constrain(K_loss, 0.5f, 5.0f);
    }

    float getFeedforward(float setpoint, float outdoorTemp) {
        float deltaT = setpoint - outdoorTemp;
        return (deltaT > 0.0f) ? (K_loss * deltaT) : 0.0f;
    }
};

#endif
