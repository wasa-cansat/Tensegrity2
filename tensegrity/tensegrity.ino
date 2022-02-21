
#include <EEPROM.h>
#include "NavUnit.h"

// Framerates
#define SERIAL_BAUD 115200

void setup() {
  Serial.begin(SERIAL_BAUD);
  EEPROM.begin(EEPROM_SIZE);

  EEPROM.get<uint8_t>(NUMBER_ADDR, number);

  runner_init();
  nav_init();
}

void loop() {
  runner_loop();
}
