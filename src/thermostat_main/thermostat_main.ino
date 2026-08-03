/*
 * ============================================================
 *  Умный термостат: 2× DS18B20 + PID + Feedforward-компенсация
 *  с онлайн-регрессией коэффициента теплопотерь
 * ============================================================
 *
 * Архитектура (см. /src):
 *   Sensor.h         — датчик DS18B20: асинхронный опрос, EMA-фильтр,
 *                       отказоустойчивость
 *   PIDController.h  — ПИД-регулятор с анти-виндапом и автонастройкой
 *                       (релейный метод Ziegler-Nichols)
 *   ThermalModel.h   — физическая модель теплового баланса +
 *                       рекурсивная линейная регрессия (RLS) для
 *                       оценки коэффициента теплопотерь в реальном
 *                       времени
 *   Thermostat.h     — конечный автомат (state machine), интеграция
 *                       всех компонентов, ограничение скорости ШИМ
 *
 * Данный файл — тонкий "entry point": инициализация + вывод
 * телеметрии для Serial Plotter / логирования в CSV.
 */

#include "Thermostat.h"

#define DS_PLATE_PIN    2   // Датчик 1 (Пластина) на D2
#define DS_OUTDOOR_PIN  3   // Датчик 2 (Улица) на D3
#define ACTUATOR_PIN    6   // ШИМ-выход (Светодиод/MOSFET) на D6

const float SETPOINT   = 40.0f; // Целевая температура пластины (°C)
const float EMA_ALPHA  = 0.25f; // Коэффициент фильтрации EMA

Thermostat thermostat(DS_PLATE_PIN, DS_OUTDOOR_PIN, ACTUATOR_PIN,
                       SETPOINT, EMA_ALPHA, /*enableAutotune=*/true);

void setup() {
  Serial.begin(115200);
  thermostat.begin();

  // Заголовок CSV — удобно для последующего импорта в Python/Excel
  // для построения графиков "теория vs эксперимент" в отчёте.
  Serial.println("time_ms,setpoint,plate_temp,outdoor_temp,pwm,state,est_kloss,Kp,Ki,Kd");
}

void loop() {
  bool updated = thermostat.update();

  if (updated) {
    // Формат вывода совместим и с Serial Plotter (последние 4 поля),
    // и с CSV-парсингом (вся строка, если убрать текстовые метки).
    Serial.print(millis()); Serial.print(",");
    Serial.print(thermostat.getSetpoint()); Serial.print(",");
    Serial.print(thermostat.getPlateTemp()); Serial.print(",");
    Serial.print(thermostat.getOutdoorTemp()); Serial.print(",");
    Serial.print(thermostat.getPWM()); Serial.print(",");
    Serial.print(thermostat.getStateName()); Serial.print(",");
    Serial.print(thermostat.getEstimatedKloss()); Serial.print(",");
    Serial.print(thermostat.getKp()); Serial.print(",");
    Serial.print(thermostat.getKi()); Serial.print(",");
    Serial.println(thermostat.getKd());
  }

  // Никаких delay()! Весь тайминг — через millis() внутри update().
  // Это позволяет при желании добавить обработку других задач в loop()
  // (например, опрос кнопок, веб-сервер на ESP32 и т.д.) без блокировок.
}
