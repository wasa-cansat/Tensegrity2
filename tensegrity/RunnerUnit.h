#include "SwitchArray.h"
#include "Indicator.h"

// Pin assigns
#define LED_PIN 2
#define SWITCH_PIN 34
#define MOTOR_A_PIN 15
#define MOTOR_B_PIN 4

// Switches
#define LIMIT_R 0
#define LIMIT_L 1
#define GO_R 2
#define GO_L 3

// EEPROM addresses
#define EEPROM_SIZE (1 + 16 * 2)
#define NUMBER_ADDR 0
#define SWITCH_ADDR 1


class RunnerUnit {
private:
    SwitchArray switches = SwitchArray(SWITCH_PIN, 4);
    Indicator indicator = Indicator(LED_PIN);

    float T = 0;
    unsigned long last_millis;

    int target   = 0;
    int position = 0;

public:
    enum State {
        StartUp,
        Moving,
    };
    State state = StartUp;

    void init();
    void loop();
    void reset();
};
