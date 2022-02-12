
#include <EEPROM.h>
#include "RunnerUnit.h"

// Framerates
#define SERIAL_BAUD 115200

RunnerUnit *runner;

void setup() {
  Serial.begin(SERIAL_BAUD);
  EEPROM.begin(EEPROM_SIZE);

  uint8_t number;
  EEPROM.get<uint8_t>(NUMBER_ADDR, number);

  runner = new RunnerUnit(number);

  runner->init();
}

void loop() {
  runner->loop();
}
