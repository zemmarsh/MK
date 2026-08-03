#include "Thermostat.h"

Thermostat thermostat;

void setup() {
    Serial.begin(115200);
    thermostat.setup();
    Serial.print("time_ms,setpoint,plate_temp,outdoor_temp,pwm,state"); Serial.print("\n");
}

void loop() {
    thermostat.update();
    delay(10);
}