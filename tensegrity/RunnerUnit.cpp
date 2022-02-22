#include "RunnerUnit.h"

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

// Motor
#define MOTOR_A_CH 0
#define MOTOR_B_CH 1
#define PWM_FREQ 12800
#define PWM_RES 8
#define PWM_DUTY_MAX 255

// Sensor
#define CUTOFF_FREQ 1

// Task
#define READ_SENSOR_FREQ      100
#define UPDATE_INDICATOR_FREQ 100
#define CONTROL_MOTOR_FREQ    100
#define UPDATE_STATE_FREQ     10
#define COMM_SERIIAL_FREQ     1

uint8_t number = 0;

SwitchArray switches = SwitchArray(SWITCH_PIN, 4);
Indicator indicator = Indicator(LED_PIN);

Scheduler scheduler;
MeshComm mesh('T');

float T = 0;

State state = StartUp;


int target   = 0;
int position = 0;
float speed = 0;
float accel = 1.0;
float speed_max = 0.8;

float battery_voltage = 0;
float motor_current   = 0;

void updateTimer();
Task taskUpdateTimer(1, TASK_FOREVER, &updateTimer);

void updateState();
Task taskUpdateState(TASK_SECOND / UPDATE_STATE_FREQ,
                     TASK_FOREVER, &updateState);

void readSensors();
Task taskReadSensors(TASK_SECOND / READ_SENSOR_FREQ,
                     TASK_FOREVER, &readSensors);

void updateIndicator();
Task taskUpdateIndicator(TASK_SECOND / UPDATE_INDICATOR_FREQ,
                         TASK_FOREVER, &updateIndicator);

void controlMotor();
Task taskControlMotor(TASK_SECOND / CONTROL_MOTOR_FREQ,
                      TASK_FOREVER, &controlMotor);

void commSerial();
Task taskCommSerial(TASK_SECOND / COMM_SERIIAL_FREQ,
                      TASK_FOREVER, &commSerial);


void runner_init() {

    Serial.print("Start #");
    Serial.println(number);

    T = 0;
    state = StartUp;

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

    mesh.init(&scheduler);

    scheduler.addTask(taskUpdateTimer);
    scheduler.addTask(taskUpdateState);
    scheduler.addTask(taskReadSensors);
    scheduler.addTask(taskUpdateIndicator);
    scheduler.addTask(taskControlMotor);
    scheduler.addTask(taskCommSerial);
    taskUpdateTimer.enable();
    taskUpdateState.enable();
    taskReadSensors.enable();
    taskUpdateIndicator.enable();
    taskControlMotor.enable();
    taskCommSerial.enable();
}

void updateTimer() {
    static unsigned long last_millis = millis();
    unsigned long ms = millis();
    T += (ms - last_millis) / 1000.0;
    last_millis = ms;
}

void updateState() {
    float dt = 1.0 / UPDATE_STATE_FREQ;

    switch (state) {
    case StartUp:
        if (T > 2.0) state = Moving;
        break;

    case Moving:
        if (switches.isPressed(LIMIT_R) && switches.isPressed(LIMIT_L));
        else if (switches.isPressed(LIMIT_R)) position = 1;
        else if (switches.isPressed(LIMIT_L)) position = -1;
        else                                  position = 0;

        if (switches.isPressed(GO_R) && switches.isPressed(GO_L)) {
            runner_reset();
            return;
        }
        else if (switches.isPressed(GO_R)) target = 1;
        else if (switches.isPressed(GO_L)) target = -1;
        else                               target = 0;

        if (target == 1 && position != 1)
            speed = min(speed + accel * dt, speed_max);
        else if (target == -1 && position != -1)
            speed = max(speed - accel * dt, - speed_max);
        else if (target == 0 && position == 0){
            if      (speed > 0) speed = max(speed - accel * dt, 0.0f);
            else if (speed < 0) speed = min(speed + accel * dt, 0.0f);
        }
        else speed = 0;


        break;
    }
}

void controlMotor() {
    unsigned motorDutyA = 0;
    unsigned motorDutyB = 0;
    if      (speed > 0) motorDutyA =   speed * PWM_DUTY_MAX;
    else if (speed < 0) motorDutyB = - speed * PWM_DUTY_MAX;

    ledcWrite(MOTOR_A_CH, motorDutyA);
    ledcWrite(MOTOR_B_CH, motorDutyB);
}

void readSensors() {
    static float prev_voltage = 0;
    static float prev_current = 0;

    float A = (float)READ_SENSOR_FREQ / (CUTOFF_FREQ * M_PI);
    float v = analogRead(VOLTAGE_PIN) / 4095.0 * 3.3 * (100 + 220) / 100;
    float c = analogRead(CURRENT_PIN) / 4095.0 * 3.3 * 1000;

    // Serial.println(analogRead(CURRENT_PIN));

    battery_voltage = 1/(A+1) * (v + prev_voltage + (A - 1) * battery_voltage);
    motor_current   = 1/(A+1) * (c + prev_current + (A - 1) * motor_current);

    prev_voltage = v;
    prev_current = c;

    switches.update();
}

void updateIndicator() {
    switch (state) {
    case StartUp:
        indicator.state = Indicator::StartUp;
        break;

    case Moving:
        indicator.state = Indicator::Moving;
        if (switches.isPressed(LIMIT_R) && switches.isPressed(LIMIT_L))
            indicator.state = Indicator::Error;
        break;
    }

    indicator.position = position;
    indicator.moving   = target;

    indicator.update();
}

void commSerial() {
    char buf[128];
    sprintf(buf, "V: %f, C: %f", battery_voltage, motor_current);
    Serial.println(buf);

    if (Serial.available() > 0) {
        switch (Serial.read()) {
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

void runner_loop() {
    mesh.update();
}

void runner_reset() {
    T = 0;
    Serial.print("Resetting\n");
    state = StartUp;
}


void i2c_scanner() {
    byte error, address;
    int nDevices;

    Serial.println("I2C Scanner");
    Serial.println("Scanning...");

    nDevices = 0;
    for(address = 1; address < 127; address++ )
        {
            // The i2c_scanner uses the return value of
            // the Write.endTransmisstion to see if
            // a device did acknowledge to the address.
            Wire.beginTransmission(address);
            error = Wire.endTransmission();

            if (error == 0)
                {
                    Serial.print("I2C device found at address 0x");
                    if (address<16)
                        Serial.print("0");
                    Serial.print(address,HEX);
                    Serial.println("  !");

                    nDevices++;
                }
            else if (error==4)
                {
                    Serial.print("Unknown error at address 0x");
                    if (address<16)
                        Serial.print("0");
                    Serial.println(address,HEX);
                }
            else {
                //      Serial.println(error);
            }
        }
    if (nDevices == 0)
        Serial.println("No I2C devices found\n");
    else
        Serial.println("done\n");
}
