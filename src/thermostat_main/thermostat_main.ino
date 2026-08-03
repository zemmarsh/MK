#include "Thermostat.h"

Thermostat thermostat;

void setup() {
    Serial.begin(115200);
    thermostat.setup();
}

void loop() {
    thermostat.update();
    delay(10);
}