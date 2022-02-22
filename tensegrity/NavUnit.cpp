#include "NavUnit.h"

// Pin assigns
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define GPS_TX_PIN  5
#define GPS_RX_PIN  25

#define GPS_BAUD 9600

// #define LSM9DS1_M_ADDR_  0x1E
// #define LSM9DS1_AG_ADDR_ 0x6B

// Task
#define READ_LOCATION_FREQ     1
#define READ_IMU_FREQ          1
#define CALC_AND_SEND_NAV_FREQ 1

// Values
float altitude = 0;
float qnh = 0;
float temperature = 0;

// Task
void readLocation();
Task taskReadLocation(TASK_SECOND / READ_LOCATION_FREQ, TASK_FOREVER,
                      &readLocation);

void readIMU();
Task taskReadIMU(TASK_SECOND / READ_IMU_FREQ, TASK_FOREVER, &readIMU);

void calcAndSendNav();
Task taskCalcAndSendNav(TASK_SECOND / READ_IMU_FREQ, TASK_FOREVER,
                        &calcAndSendNav);

// Sensors
LPS lps25hb;
// LSM9DS1 lsm9ds1;



void nav_init() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);


    // bool imu_ok = imu.begin(LSM9DS1_AG_ADDR, LSM9DS1_M_ADDR);
    // if (!imu_ok) log_e("Failed to initialize IMU");

    bool prs_ok = lps25hb.init(LPS::device_25H);
    if (!prs_ok) log_e("Failed to initialize pressure sensor");
    lps25hb.enableDefault();

    qnh = lps25hb.readPressureMillibars();

    scheduler.addTask(taskReadLocation);
    scheduler.addTask(taskReadIMU);
    scheduler.addTask(taskCalcAndSendNav);
    taskReadLocation.enable();
    taskReadIMU.enable();
    taskCalcAndSendNav.enable();
}

void readLocation() {
    // while (Serial2.available()) {
    //     Serial.print(Serial2.read());
    // }
    // Serial.println("]");
}

void readIMU() {
    i2c_scanner();

    // if (imu.gyroAvailable())  imu.readGyro();
    // if (imu.accelAvailable()) imu.readAccel();
    // if (imu.magAvailable())   imu.readMag();


    float pressure = lps25hb.readPressureMillibars();
    altitude    = lps25hb.pressureToAltitudeMeters(pressure, qnh);
    temperature = lps25hb.readTemperatureC();

    // Serial.print("Ax: ");
    // Serial.print(lsm9ds1.ax);
    // Serial.print(",Ay: ");
    // Serial.print(lsm9ds1.ay);
    // Serial.print(",Az: ");
    // Serial.print(lsm9ds1.az);
    Serial.print(",prs: ");
    Serial.print(pressure);
    Serial.print(",alt: ");
    Serial.print(altitude);
    Serial.print(",tmp: ");
    Serial.print(temperature);
    Serial.println("");
}

void calcAndSendNav() {
    mesh.send('A', 0, (float)altitude);
}
