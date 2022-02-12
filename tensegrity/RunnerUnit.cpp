#include "RunnerUnit.h"


void RunnerUnit::init() {

    Serial.print("Start #");
    Serial.println(number);

    T = 0;
    state = StartUp;

    last_millis = millis();

    switches.init();
    indicator.init();

    switches.load_ruler(SWITCH_ADDR);

    indicator.state = Indicator::StartUp;
    indicator.update();

    ledcSetup(MOTOR_A_CH, PWM_FREQ, PWM_RES);
    ledcSetup(MOTOR_B_CH, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_A_PIN, MOTOR_A_CH);
    ledcAttachPin(MOTOR_B_PIN, MOTOR_B_CH);
    ledcWrite(MOTOR_A_CH, 0);
    ledcWrite(MOTOR_B_CH, 0);

    pinMode(VOLTAGE_PIN, ANALOG);
    pinMode(CURRENT_PIN, ANALOG);
}

void RunnerUnit::loop() {
    float dt = (millis() - last_millis) / 1000.0;
    T += dt;
    last_millis = millis();

    indicator.update();
    switches.update();


    float A = 1 / (dt * CUTOFF_FREQ * M_PI);
    float v = analogRead(VOLTAGE_PIN) / 4095.0 * 3.3 * (100 + 220) / 100;
    float c = analogRead(CURRENT_PIN) / 4095.0 * 3.3 / 101 / 0.02 * 1000;
    battery_voltage = 1/(A+1) * (v + prev_voltage + (A - 1) * battery_voltage);
    motor_current   = 1/(A+1) * (c + prev_current + (A - 1) * motor_current);
    prev_voltage = v;
    prev_current = c;


    char buf[128];
    sprintf(buf, "V: %f, c: %f, C: %f", battery_voltage * 0, c, motor_current);
    Serial.println(buf);
    // switches.print();


    unsigned motorDutyA = 0;
    unsigned motorDutyB = 0;

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

        if (target == 1 && position != 1)
            speed = min(speed + accel * dt, speed_max);
        else if (target == -1 && position != -1)
            speed = max(speed - accel * dt, - speed_max);
        else if (target == 0 && position == 0){
            if      (speed > 0) speed = max(speed - accel * dt, 0.0f);
            else if (speed < 0) speed = min(speed + accel * dt, 0.0f);
        }
        else speed = 0;

        if      (speed > 0) motorDutyA =   speed * PWM_DUTY_MAX;
        else if (speed < 0) motorDutyB = - speed * PWM_DUTY_MAX;

        break;
    }

    ledcWrite(MOTOR_A_CH, motorDutyA);
    ledcWrite(MOTOR_B_CH, motorDutyB);


    if (Serial.available() > 0) {
        switch (Serial.read()) {
        case 'n': {
            Serial.print("Set number: ");
            int n = Serial.parseInt();
            Serial.println(n);
            EEPROM.put<uint8_t>(NUMBER_ADDR, n);
            EEPROM.commit();
        }
            break;
        case 's':
            switches.record();
            switches.store_ruler(SWITCH_ADDR);
            break;
        }
    }


    delay(10);
}

void RunnerUnit::reset() {
    T = 0;
    Serial.println("Resetting");
    state = StartUp;
}

