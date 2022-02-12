
#include <EEPROM.h>
#include "RunnerUnit.h"

// Framerates
#define SERIAL_BAUD 115200

RunnerUnit runner;

void setup() {
  Serial.begin(SERIAL_BAUD);
  EEPROM.begin(EEPROM_SIZE);

  runner.init();
}

void loop() {
  runner.loop();
}
