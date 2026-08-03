#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>

// Фильтрация показаний датчика методом экспоненциального сглаживания (EMA)
class Sensor {
private:
    float filteredVal = 20.0f;  // Отфильтрованное значение
    float alpha = 0.25f;        // Коэффициент сглаживания (чем меньше, тем глаже график)
    unsigned long lastPoll = 0; // Время последнего опроса
    unsigned long intervalMs;   // Период опроса (мс)

public:
    Sensor(unsigned long interval) : intervalMs(interval) {}

    // Сглаживание сырых данных с датчика по таймеру
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
