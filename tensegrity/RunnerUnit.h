#pragma once

#include "SwitchArray.h"
#include "Indicator.h"
#include "Comm.h"


// EEPROM
#define EEPROM_SIZE (1 + 16 * 2 + 32 * 3)
#define NUMBER_ADDR 0
#define SWITCH_ADDR 1



enum State {
    StartUp,
    Moving,
};

extern uint8_t number;

extern SwitchArray switches;
extern Indicator indicator;
extern Scheduler scheduler;
extern MeshComm mesh;

extern float T;

extern State state;

void runner_init();
void runner_loop();
void runner_reset();

void i2c_scanner();

void commSerial();
