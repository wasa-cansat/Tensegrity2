
#include <EEPROM.h>
#include "RunnerUnit.h"
#include "NavUnit.h"

// Framerates
#define SERIAL_BAUD 115200


void setup() {
  Serial.begin(SERIAL_BAUD);
  EEPROM.begin(EEPROM_SIZE);

  EEPROM.get<uint8_t>(NUMBER_ADDR, number);

  runner_init();

  switch (number) {
  case 0:
    break;
  case 1:
    nav_init();
    break;
  case 2:
  case 3:
    break;
  case 4:
    break;
  case 5:
    break;
  }

}

void loop() {
  runner_loop();
}


void commSerial() {
  char buf[128];
  /* sprintf(buf, "V: %f, C: %f", battery_voltage, motor_current); */
  if (Serial.available() > 0) {
    switch (Serial.read()) {
    case  'c':
      if (!mag_calibrating) startCalibration();
      else                  finishCalibration();
      break;
    case 'n': {
      Serial.print("Set number: ");
      int n = Serial.parseInt();
      Serial.println(n);
      EEPROM.put<uint8_t>(NUMBER_ADDR, n);
      EEPROM.commit();
      runner_reset();
    }
      break;
    case 's':
      switches.record();
      switches.store_ruler(SWITCH_ADDR);
      break;
    }
  }
}


