#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

/*
 * Класс PIDController — классический ПИД-регулятор с:
 *  - анти-виндапом интегральной составляющей (clamping),
 *  - защитой от "derivative kick" (дифференцирование по
 *    измерению, а не по ошибке, чтобы избежать выброса при
 *    резкой смене setpoint),
 *  - режимом автонастройки методом релейной обратной связи
 *    (упрощённый вариант Ziegler-Nichols).
 *
 * Автонастройка: подаём на выход релейный сигнал (вкл/выкл
 * актуатора), измеряем период автоколебаний Tu и амплитуду Au,
 * затем по классическим формулам Циглера-Никольса вычисляем
 * Kp, Ki, Kd. Это отдельный "продвинутый алгоритм", который
 * запускается один раз в режиме калибровки, а не постоянно.
 */
class PIDController {
public:
  PIDController(float kp, float ki, float kd, float outMin, float outMax)
    : Kp(kp), Ki(ki), Kd(kd),
      outputMin(outMin), outputMax(outMax),
      integral(0), prevMeasurement(0), firstRun(true),
      autotuneActive(false) {}

  float compute(float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measurement;

    // Интегральная составляющая с анти-виндапом
    integral += error * dt;
    float integralLimit = (outputMax - outputMin) / max(Ki, 0.0001f);
    integral = constrain(integral, -integralLimit, integralLimit);

    // Дифференцируем измерение (не ошибку!), чтобы избежать
    // выброса производной при скачке setpoint ("derivative kick")
    float derivative = 0.0f;
    if (!firstRun) {
      derivative = -(measurement - prevMeasurement) / dt;
    }
    prevMeasurement = measurement;
    firstRun = false;

    float output = Kp * error + Ki * integral + Kd * derivative;
    return constrain(output, outputMin, outputMax);
  }

  void reset() {
    integral = 0;
    firstRun = true;
  }

  void setGains(float kp, float ki, float kd) {
    Kp = kp; Ki = ki; Kd = kd;
  }

  float getKp() const { return Kp; }
  float getKi() const { return Ki; }
  float getKd() const { return Kd; }

  // ---------- Автонастройка (релейный метод Ziegler-Nichols) ----------
  // Вызывается вместо compute() в режиме калибровки.
  // relayAmplitude — амплитуда выходного сигнала реле (в единицах ШИМ)
  // Возвращает true, когда калибровка завершена (собрано minCycles циклов).
  bool autotuneStep(float setpoint, float measurement, float dt,
                     float relayAmplitude, uint8_t minCycles,
                     float &outputForActuator) {
    if (!autotuneActive) {
      autotuneActive = true;
      atRelayHigh = true;
      atLastCrossTime = millis();
      atCycleCount = 0;
      atPeakMax = measurement;
      atPeakMin = measurement;
    }

    // Релейное переключение по знаку ошибки (с небольшим гистерезисом,
    // чтобы не дребезжать на шуме датчика)
    const float hysteresis = 0.3f;
    float error = setpoint - measurement;

    if (atRelayHigh && error < -hysteresis) {
      atRelayHigh = false;
      registerCross();
    } else if (!atRelayHigh && error > hysteresis) {
      atRelayHigh = true;
      registerCross();
    }

    atPeakMax = max(atPeakMax, measurement);
    atPeakMin = min(atPeakMin, measurement);

    outputForActuator = atRelayHigh ? relayAmplitude : 0.0f;

    if (atCycleCount >= minCycles) {
      // Период автоколебаний Tu и амплитуда Au известны -> считаем Ku
      float Ku = (4.0f * relayAmplitude) / (PI * (atPeakMax - atPeakMin));
      float Tu = atAvgPeriodMs / 1000.0f;

      // Классические формулы Ziegler-Nichols для ПИД:
      Kp = 0.6f * Ku;
      Ki = 1.2f * Ku / Tu;
      Kd = 0.075f * Ku * Tu;

      autotuneActive = false;
      return true;
    }
    return false;
  }

private:
  void registerCross() {
    unsigned long now = millis();
    if (atCycleCount > 0) {
      unsigned long period = now - atLastCrossTime;
      atAvgPeriodMs = (atAvgPeriodMs * (atCycleCount - 1) + period) / atCycleCount;
    }
    atLastCrossTime = now;
    atCycleCount++;
    // Сбрасываем пики для следующего полупериода
    atPeakMax = -1000; atPeakMin = 1000;
  }

  float Kp, Ki, Kd;
  float outputMin, outputMax;
  float integral;
  float prevMeasurement;
  bool firstRun;

  // Состояние автонастройки
  bool autotuneActive;
  bool atRelayHigh;
  unsigned long atLastCrossTime;
  uint8_t atCycleCount;
  float atPeakMax, atPeakMin;
  float atAvgPeriodMs = 0;
};

#endif
