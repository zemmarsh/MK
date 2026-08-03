#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

class Sensor {
private:
    float filteredVal = 20.0f;
    float alpha = 0.25f; // EMA-фильтрация
    unsigned long lastPoll = 0;
    unsigned long intervalMs;

public:
    Sensor(unsigned long interval) : intervalMs(interval) {}

    bool update(float rawMeasurement) {
        unsigned long now = millis();
        if (now - lastPoll >= intervalMs) {
            lastPoll = now;
            filteredVal = (alpha * rawMeasurement) + ((1.0f - alpha) * filteredVal);
            return true;
        }
        return false;
    }

    float getValue() const { return filteredVal; }
};

#endif
