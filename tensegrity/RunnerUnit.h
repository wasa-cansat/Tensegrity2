#pragma once

#include <math.h>
#include <Arduino.h>
#include <painlessMesh.h>
#include <Wire.h>

#include "SwitchArray.h"
#include "Indicator.h"


// EEPROM
#define EEPROM_SIZE (1 + 16 * 2)
#define NUMBER_ADDR 0
#define SWITCH_ADDR 1



enum State {
    StartUp,
    Moving,
};

extern uint8_t number;

extern Indicator indicator;
extern Scheduler scheduler;

extern float T;

extern State state;

void runner_init();
void runner_loop();
void runner_reset();

void i2c_scanner();
