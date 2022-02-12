#include <Arduino.h>
#include <math.h>

#include "SwitchArray.h"
#include "Indicator.h"

// Pin assigns
#define LED_PIN 2
#define SWITCH_PIN 34
#define MOTOR_A_PIN 15
#define MOTOR_B_PIN 4
#define VOLTAGE_PIN 35
#define CURRENT_PIN 32

// Switches
#define LIMIT_R 0
#define LIMIT_L 1
#define GO_R 2
#define GO_L 3

// EEPROM addresses
#define EEPROM_SIZE (1 + 16 * 2)
#define NUMBER_ADDR 0
#define SWITCH_ADDR 1

// Motor
#define MOTOR_A_CH 0
#define MOTOR_B_CH 1
#define PWM_FREQ 12800
#define PWM_RES 8
#define PWM_DUTY_MAX 255

// Sensor
#define CUTOFF_FREQ 1

class RunnerUnit {
public:
    uint8_t number;

    SwitchArray switches = SwitchArray(SWITCH_PIN, 4);
    Indicator indicator = Indicator(LED_PIN);

    float T = 0;
    unsigned long last_millis;


    int target   = 0;
    int position = 0;
    float speed = 0;
    float accel = 1.0;
    float speed_max = 0.8;

    float battery_voltage = 0;
    float motor_current   = 0;
    float prev_voltage = 0;
    float prev_current = 0;

    enum State {
        StartUp,
        Moving,
    };

    State state = StartUp;

    RunnerUnit(uint8_t number): number(number) {};

    void init();
    void loop();
    void reset();

    float batteryVoltage(float dt);
    float motorCurrent(float dt);
};
