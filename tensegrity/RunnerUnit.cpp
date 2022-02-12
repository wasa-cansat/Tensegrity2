#include "RunnerUnit.h"


void RunnerUnit::init() {

    Serial.println("Start");

    T = 0;
    state = StartUp;

    last_millis = millis();

    switches.init();
    indicator.init();

    indicator.state = Indicator::StartUp;
    indicator.update();

    pinMode(MOTOR_A_PIN, OUTPUT);
    pinMode(MOTOR_B_PIN, OUTPUT);
    digitalWrite(MOTOR_A_PIN, 0);
    digitalWrite(MOTOR_B_PIN, 0);

    switches.load_ruler(SWITCH_ADDR);
}

void RunnerUnit::loop() {
    T += (millis() - last_millis) / 1000.0;
    last_millis = millis();

    indicator.update();
    switches.update();

    Serial.println(T);
    // switches.print();

    digitalWrite(MOTOR_A_PIN, 0);
    digitalWrite(MOTOR_B_PIN, 0);

    switch (state) {
    case StartUp:
        indicator.state = Indicator::StartUp;
        if (T > 2.0) state = Moving;

        break;

    case Moving:
        indicator.state = Indicator::Moving;
        if (switches.isPressed(LIMIT_R) && switches.isPressed(LIMIT_L))
            indicator.state = Indicator::Error;
        else if (switches.isPressed(LIMIT_R)) position = 1;
        else if (switches.isPressed(LIMIT_L)) position = -1;
        else                                  position = 0;

        if (switches.isPressed(GO_R) && switches.isPressed(GO_L)) {
            reset();
            return;
        }
        else if (switches.isPressed(GO_R)) target = 1;
        else if (switches.isPressed(GO_L)) target = -1;
        else                               target = 0;

        indicator.position = position;
        indicator.moving   = target;

        if (target == 1 && position != 1) {
            digitalWrite(MOTOR_A_PIN, 1);
            digitalWrite(MOTOR_B_PIN, 0);
        }
        else if (target == -1 && position != -1) {
            digitalWrite(MOTOR_A_PIN, 0);
            digitalWrite(MOTOR_B_PIN, 1);
        }
        else {
            digitalWrite(MOTOR_A_PIN, 0);
            digitalWrite(MOTOR_B_PIN, 0);
        }

        break;
    }


    if (Serial.available() > 0) {
        switch (Serial.read()) {
        case 's':
            switches.record();
            switches.store_ruler(SWITCH_ADDR);
            break;
        }
    }


    delay(50);
}

void RunnerUnit::reset() {
    T = 0;
    Serial.println("Resetting");
    state = StartUp;
}
