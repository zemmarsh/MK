#ifndef THERMOSTAT_H
#define THERMOSTAT_H

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "Sensor.h"
#include "PIDController.h"
#include "ThermalModel.h"

#define ONE_WIRE_BUS_PLATE 2   // Датчик пластины (DS18B20 #1)
#define ONE_WIRE_BUS_OUTDOOR 3 // Датчик улицы (DS18B20 #2)
#define PWM_PIN 6              // MOSFET / Нагреватель (D6 PWM)

enum class State { BOOT, AUTOTUNE, HEATING, STABILIZED };

class Thermostat {
private:
    State currentState = State::BOOT;

    OneWire oneWirePlate{ONE_WIRE_BUS_PLATE};
    OneWire oneWireOutdoor{ONE_WIRE_BUS_OUTDOOR};
    DallasTemperature sensorPlate{&oneWirePlate};
    DallasTemperature sensorOutdoor{&oneWireOutdoor};

    Sensor plateSensor{750};   // 12 бит
    Sensor outdoorSensor{100}; // 9 бит

    PIDController pid;
    ThermalModel model;

    const float setpoint = 40.0f;
    unsigned long stateTimer = 0;
    unsigned long lastPrintTimer = 0;
    float currentPWM = 0.0f;

public:
    void setup() {
        pinMode(PWM_PIN, OUTPUT);
        sensorPlate.begin();
        sensorOutdoor.begin();
        currentState = State::BOOT;
    }

    void update() {
        sensorPlate.requestTemperatures();
        sensorOutdoor.requestTemperatures();

        float rawPlate = sensorPlate.getTempCByIndex(0);
        float rawOutdoor = sensorOutdoor.getTempCByIndex(0);

        if (rawPlate == DEVICE_DISCONNECTED_C || rawPlate < -50.0f) rawPlate = 20.0f;
        if (rawOutdoor == DEVICE_DISCONNECTED_C || rawOutdoor < -50.0f) rawOutdoor = 20.0f;

        plateSensor.update(rawPlate);
        outdoorSensor.update(rawOutdoor);

        float plateT = plateSensor.getValue();
        float outdoorT = outdoorSensor.getValue();

        switch (currentState) {
            case State::BOOT:
                stateTimer = millis();
                currentState = State::AUTOTUNE;
                break;

            case State::AUTOTUNE:
                analogWrite(PWM_PIN, 200);
                currentPWM = 200.0f;

                if (millis() - stateTimer > 15000) {
                    pid.setTunings(4.5f, 0.08f, 1.2f);
                    pid.resetIntegral();
                    currentState = State::HEATING;
                }
                break;

            case State::HEATING: {
                float ff = model.getFeedforward(setpoint, outdoorT);
                currentPWM = pid.compute(setpoint, plateT, ff);
                analogWrite(PWM_PIN, (int)currentPWM);

                if (abs(setpoint - plateT) < 0.5f) {
                    currentState = State::STABILIZED;
                }
                break;
            }

            case State::STABILIZED: {
                float ff = model.getFeedforward(setpoint, outdoorT);
                currentPWM = pid.compute(setpoint, plateT, ff);
                analogWrite(PWM_PIN, (int)currentPWM);

                float deltaT = setpoint - outdoorT;
                model.updateRLS(deltaT, currentPWM);

                if (abs(setpoint - plateT) > 1.5f) {
                    currentState = State::HEATING;
                }
                break;
            }
        }

        // Вывод данных раз в 500 миллисекунд
        if (millis() - lastPrintTimer >= 500) {
            lastPrintTimer = millis();
            Serial.print(millis()); Serial.print(",");
            Serial.print(setpoint); Serial.print(",");
            Serial.print(plateT); Serial.print(",");
            Serial.print(outdoorT); Serial.print(",");
            Serial.print(currentPWM); Serial.print(",");
            Serial.println((int)currentState);
        }
    }
};

#endif
