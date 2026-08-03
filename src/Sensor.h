#ifndef SENSOR_H
#define SENSOR_H

#include <OneWire.h>
#include <DallasTemperature.h>

/*
 * Класс Sensor — обёртка над DS18B20.
 *
 * Задачи:
 *  1. Инкапсулировать работу с шиной 1-Wire для одного датчика.
 *  2. Реализовать асинхронный (неблокирующий) опрос — вместо
 *     sensor.requestTemperatures() + delay(750мс), мы запускаем
 *     конверсию и проверяем готовность по таймеру. Это освобождает
 *     процессор для других задач и снижает энергопотребление
 *  3. Применить экспоненциальный фильтр (EMA) к сырым данным.
 *  4. Обеспечить отказоустойчивость: если датчик отключился или
 *     вернул физически невозможное значение, сохраняем последнее
 *     валидное показание и считаем число последовательных сбоев
 */
class Sensor {
public:
  Sensor(uint8_t pin, float emaAlpha, uint8_t resolutionBits = 12)
    : oneWire(pin),
      dallas(&oneWire),
      alpha(emaAlpha),
      resolution(resolutionBits),
      filteredValue(20.0f),
      lastRawValue(20.0f),
      consecutiveFailures(0),
      conversionInProgress(false),
      conversionRequestTime(0) {}

  void begin() {
    dallas.begin();
    dallas.setResolution(resolution);
    // Асинхронный режим: requestTemperatures() не блокирует выполнение,
    // мы сами следим за временем конверсии через millis().
    dallas.setWaitForConversion(false);
    // Время конверсии зависит от разрешения: 12 бит = 750мс, 9 бит = 94мс.
    // Понижение разрешения — один из способов энергооптимизации:
    // меньше времени датчик активен -> меньше потребление.
    conversionTimeMs = 750 / (1 << (12 - resolution));
  }

  // Запустить новое измерение (неблокирующий вызов)
  void requestReading() {
    if (!conversionInProgress) {
      dallas.requestTemperatures();
      conversionInProgress = true;
      conversionRequestTime = millis();
    }
  }

  // Проверить, готово ли измерение, и если да — обновить фильтр.
  // Возвращает true, если значение было обновлено в этом вызове.
  bool update() {
    if (!conversionInProgress) return false;
    if (millis() - conversionRequestTime < conversionTimeMs) return false;

    float raw = dallas.getTempCByIndex(0);
    conversionInProgress = false;

    // Валидация: DEVICE_DISCONNECTED_C == 85.0 или -127.0 в некоторых
    // случаях; также отбрасываем физически невозможные значения.
    bool valid = (raw != DEVICE_DISCONNECTED_C) && (raw > -50.0f) && (raw < 125.0f);

    if (valid) {
      lastRawValue = raw;
      consecutiveFailures = 0;
    } else {
      consecutiveFailures++;
      raw = lastRawValue; // используем последнее валидное значение
    }

    filteredValue = alpha * raw + (1.0f - alpha) * filteredValue;
    return true;
  }

  float getFiltered() const { return filteredValue; }
  float getRaw() const { return lastRawValue; }
  bool isFailed(uint8_t threshold = 5) const { return consecutiveFailures >= threshold; }
  uint16_t getFailureCount() const { return consecutiveFailures; }

private:
  OneWire oneWire;
  DallasTemperature dallas;
  float alpha;
  uint8_t resolution;
  uint16_t conversionTimeMs;

  float filteredValue;
  float lastRawValue;
  uint16_t consecutiveFailures;

  bool conversionInProgress;
  unsigned long conversionRequestTime;
};

#endif
