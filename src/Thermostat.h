#ifndef THERMOSTAT_H
#define THERMOSTAT_H

#include <Arduino.h>
#include "Sensor.h"
#include "PIDController.h"
#include "ThermalModel.h"

/*
 * Класс Thermostat — верхнеуровневая интеграция всех компонентов
 * системы. Реализует конечный автомат (state machine):
 *
 *   BOOT -> AUTOTUNE -> HEATING -> STABILIZED
 *                              \-> ERROR (если датчик отвалился)
 *
 * Состояния:
 *   BOOT       — инициализация, прогрев датчиков (первые чтения)
 *   AUTOTUNE   — опционально: релейная автонастройка Kp/Ki/Kd
 *   HEATING    — активный прогрев к setpoint (большая ошибка)
 *   STABILIZED — температура близка к setpoint (|error| < eps),
 *                система обновляет онлайн-регрессию Kloss
 *   ERROR      — датчик отключился / данные недостоверны,
 *                актуатор переводится в безопасное состояние (0)
 *
 * Такое разделение состояний даёт:
 *  - явную защиту от аварий (ERROR всегда глушит нагреватель),
 *  - возможность включать регрессию только в стабильном режиме
 *    (когда dT/dt ~ 0, что и нужно для корректной оценки Kloss),
 *  - понятную диагностику через Serial для отчёта/видео.
 */
class Thermostat {
public:
  enum State { BOOT, AUTOTUNE, HEATING, STABILIZED, ERROR_STATE };

  Thermostat(uint8_t platePin, uint8_t outdoorPin, uint8_t actuatorPin,
             float setpointC, float emaAlpha)
    : plateSensor(platePin, emaAlpha, 12),
      outdoorSensor(outdoorPin, emaAlpha, 9), // улице точность не критична ->
                                               // 9 бит = быстрее и экономичнее
      pid(18.0f, 0.4f, 3.5f, 0.0f, 255.0f),
      thermalModel(/*initial Kloss*/ 2.5f, /*intercept*/ 0.0f),
      actuatorPin(actuatorPin),
      setpoint(setpointC),
      state(BOOT),
      bootStartTime(0),
      lastLoopTime(0),
      currentPWM(0),
      stabilizedErrorThreshold(0.5f),
      rampMaxStepPerSecond(60.0f) // ограничение скорости нарастания ШИМ
  {}

  void begin() {
    plateSensor.begin();
    outdoorSensor.begin();
    pinMode(actuatorPin, OUTPUT);
    bootStartTime = millis();
    lastLoopTime = millis();
    plateSensor.requestReading();
    outdoorSensor.requestReading();
  }

  // Вызывается в каждом проходе loop(); сам следит за интервалом.
  // Возвращает true, если в этом вызове была выполнена полная итерация
  // (обновление регулятора) — удобно для тайминга логирования.
  bool update() {
    unsigned long now = millis();
    float dt = (now - lastLoopTime) / 1000.0f;
    if (dt < 0.5f) return false; // опрос раз в 500мс, как в исходной версии
    lastLoopTime = now;

    // Неблокирующее обновление датчиков
    bool plateReady = plateSensor.update();
    bool outdoorReady = outdoorSensor.update();
    plateSensor.requestReading();   // сразу запускаем следующую конверсию
    outdoorSensor.requestReading();

    float plateTemp = plateSensor.getFiltered();
    float outdoorTemp = outdoorSensor.getFiltered();

    // --- Переходы состояний ---
    if (plateSensor.isFailed() || outdoorSensor.isFailed()) {
      state = ERROR_STATE;
    } else if (state == ERROR_STATE) {
      // Датчики восстановились — возвращаемся к обычной работе
      state = HEATING;
      pid.reset();
    } else if (state == BOOT && (now - bootStartTime > 2000)) {
      // Даём 2 секунды на прогрев/первые чтения датчиков
      state = HEATING;
    }

    float error = setpoint - plateTemp;
    if (state == HEATING && fabs(error) < stabilizedErrorThreshold) {
      state = STABILIZED;
    } else if (state == STABILIZED && fabs(error) >= stabilizedErrorThreshold * 2.0f) {
      state = HEATING; // существенное отклонение — снова активный прогрев
    }

    // --- Управляющее воздействие в зависимости от состояния ---
    float targetPWM = 0.0f;

    switch (state) {
      case ERROR_STATE:
        targetPWM = 0.0f; // безопасное состояние — нагреватель выключен
        break;

      case BOOT:
        targetPWM = 0.0f;
        break;

      case HEATING:
      case STABILIZED: {
        float pidOut = pid.compute(setpoint, plateTemp, dt);

        // Feedforward: физически обоснованная компенсация теплопотерь.
        // Используем ОЦЕНКУ Kloss из онлайн-регрессии вместо статичной
        // константы Kff — модель самонастраивается под реальные условия
        // (см. ThermalModel.h).
        float deltaT = setpoint - outdoorTemp;
        float ffOutput = 0.0f;
        if (deltaT > 0) {
          ffOutput = thermalModel.getEstimatedKloss() * deltaT
                     + thermalModel.getEstimatedBaseline();
        }

        targetPWM = pidOut + ffOutput;

        // Обновляем регрессию ТОЛЬКО в стабильном режиме, когда
        // dT/dt близко к нулю — иначе оценка Kloss будет искажена
        // переходным процессом нагрева/остывания.
        //
        // ВАЖНО: подаём в регрессию НЕ targetPWM/currentPWM целиком,
        // а "фактически потребовавшийся суммарный PWM" через связку
        // (предыдущий ffOutput + pidOut) — то есть используем pidOut
        // как независимую от ТЕКУЩЕЙ оценки Kloss меру рассогласования.
        // Если PID продолжает добавлять положительную поправку поверх
        // FF — значит FF занижен; если PID тянет вниз — FF завышен.
        // Регрессия учится на (deltaT, ffOutput + pidOut), но
        // ThermalModel сам ограничивает частоту обновлений (не чаще
        // раза в несколько секунд), разрывая мгновенную обратную связь,
        // из-за которой оценка раньше "убегала", не сходясь.
        if (state == STABILIZED) {
          thermalModel.updateRegression(deltaT, ffOutput + pidOut);
        }
        break;
      }
    }

    targetPWM = constrain(targetPWM, 0.0f, 255.0f);

    // Ограничение скорости нарастания ШИМ (rate limiting) — защищает
    // нагревательный элемент от резких скачков тока и механических
    // напряжений (полезно для реальных нагревателей/Пельтье).
    float maxStep = rampMaxStepPerSecond * dt;
    float delta = constrain(targetPWM - currentPWM, -maxStep, maxStep);
    currentPWM += delta;

    analogWrite(actuatorPin, (int)currentPWM);

    lastDt = dt;
    lastPlateTemp = plateTemp;
    lastOutdoorTemp = outdoorTemp;
    return true;
  }

  // --- Геттеры для логирования / Serial Plotter ---
  float getPlateTemp() const { return lastPlateTemp; }
  float getOutdoorTemp() const { return lastOutdoorTemp; }
  float getPWM() const { return currentPWM; }
  float getSetpoint() const { return setpoint; }
  State getState() const { return state; }
  float getEstimatedKloss() const { return thermalModel.getEstimatedKloss(); }

  const char* getStateName() const {
    switch (state) {
      case BOOT: return "BOOT";
      case AUTOTUNE: return "AUTOTUNE";
      case HEATING: return "HEATING";
      case STABILIZED: return "STABILIZED";
      case ERROR_STATE: return "ERROR";
    }
    return "UNKNOWN";
  }

  void setSetpoint(float sp) { setpoint = sp; }

private:
  Sensor plateSensor;
  Sensor outdoorSensor;
  PIDController pid;
  ThermalModel thermalModel;

  uint8_t actuatorPin;
  float setpoint;
  State state;

  unsigned long bootStartTime;
  unsigned long lastLoopTime;

  float currentPWM;
  float stabilizedErrorThreshold;
  float rampMaxStepPerSecond;

  float lastDt = 0, lastPlateTemp = 0, lastOutdoorTemp = 0;
};

#endif
