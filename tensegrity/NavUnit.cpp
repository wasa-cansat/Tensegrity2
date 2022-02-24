#include "NavUnit.h"

#include <EEPROM.h>
#include <Adafruit_LSM9DS1.h>
#include <LPS.h>
#include <TinyGPS++.h>

#include "Algebra.h"
#include "MadgwickAHRSQ.h"
#include "RunnerUnit.h"


// Pin assigns
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define GPS_TX_PIN  5
#define GPS_RX_PIN  25

#define GPS_BAUD 9600


// #define LSM9DS1_M_ADDR  0x1E
// #define LSM9DS1_AG_ADDR 0x6B


// EEPROM
#define MAG_CENTER_ADDR 33

// Task
#define READ_LOCATION_FREQ     1
#define READ_IMU_FREQ          50
#define CALC_AND_SEND_NAV_FREQ 10
#define CALIBRATION_DURATION 10

#define CALIBRATION_GAIN 0.00001

// Sensors
Adafruit_LSM9DS1 lsm9ds1;
LPS lps25hb;
TinyGPSPlus gps;

Madgwick madgwickFilter;


// Values
float prs_altitude = 0;
float gps_altitude = 0;
float qnh = 0;
float temperature = 0;

Vec3 start_LLH;
Vec3 location_LLH;
bool gps_activated = false;

Vec3 position_ENU;
Vec3 accel_Body;
Vec3 accel_ENU;
Vec3 mag_Body;
Vec3 mag_center_Body;
Vec3 gyro_Body;
bool mag_calibrating = false;
float mag_radius;

Quaternion attitude;


// Task
void readLocation();
Task taskReadLocation(TASK_SECOND / READ_LOCATION_FREQ, TASK_FOREVER,
                      &readLocation, &scheduler);

void readIMU();
Task taskReadIMU(TASK_SECOND / READ_IMU_FREQ, TASK_FOREVER, &readIMU,
                 &scheduler);

void calcAndSendNav();
Task taskCalcAndSendNav(TASK_SECOND / CALC_AND_SEND_NAV_FREQ, TASK_FOREVER,
                        &calcAndSendNav, &scheduler);


void loadCalibration();
float calibrateMag(const Vec3& mag, Vec3& center, float& radius, float gain);

void nav_init() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    i2c_scanner();

    bool imu_ok = lsm9ds1.begin();
    if (!imu_ok) log_e("Failed to initialize IMU");

    lsm9ds1.setupAccel(lsm9ds1.LSM9DS1_ACCELRANGE_8G);
    lsm9ds1.setupMag(lsm9ds1.LSM9DS1_MAGGAIN_4GAUSS);
    lsm9ds1.setupGyro(lsm9ds1.LSM9DS1_GYROSCALE_500DPS);

    loadCalibration();

    bool prs_ok = lps25hb.init(LPS::device_25H);
    if (!prs_ok) log_e("Failed to initialize pressure sensor");
    lps25hb.enableDefault();

    qnh = lps25hb.readPressureMillibars();

    taskReadLocation.enable();
    taskReadIMU.enable();
    taskCalcAndSendNav.enable();

    madgwickFilter.begin(READ_IMU_FREQ);
}

void loadCalibration() {
    EEPROM.get<float>(MAG_CENTER_ADDR,                     mag_center_Body.x);
    EEPROM.get<float>(MAG_CENTER_ADDR + sizeof(float),     mag_center_Body.y);
    EEPROM.get<float>(MAG_CENTER_ADDR + sizeof(float) * 2, mag_center_Body.z);
    log_d("load calibrated mag center: (%f, %f, %f)",
          mag_center_Body.x, mag_center_Body.y, mag_center_Body.z);
}

void startCalibration() {
    log_d("start calibration");
    mag_center_Body = Vec3();
    mag_calibrating = true;
}

void finishCalibration() {
    log_d("finish calibration");
    mag_calibrating = false;

    EEPROM.put<float>(MAG_CENTER_ADDR,                     mag_center_Body.x);
    EEPROM.put<float>(MAG_CENTER_ADDR + sizeof(float),     mag_center_Body.y);
    EEPROM.put<float>(MAG_CENTER_ADDR + sizeof(float) * 2, mag_center_Body.z);
    EEPROM.commit();

    log_d("stored calibrated mag center: (%f, %f, %f)",
          mag_center_Body.x, mag_center_Body.y, mag_center_Body.z);
}

void readLocation() {
    while(Serial2.available() > 0){
        char c = Serial2.read();
        // Serial.print(c);
        gps.encode(c);
        if(gps.location.isUpdated()){
            location_LLH.x = gps.location.lng();
            location_LLH.y = gps.location.lat();
            location_LLH.z = gps.altitude.meters();
        }
        if (!gps_activated) {
            start_LLH = location_LLH;
            gps_activated = true;
        }
    }
    // while (Serial2.available()) {
    //     Serial.print(Serial2.read());
    // }
    // Serial.println("]");
}

void readIMU() {

    lsm9ds1.read();
    sensors_event_t a, m, g, temp;
    lsm9ds1.getEvent(&a, &m, &g, &temp);

    accel_Body = Vec3(- a.acceleration.x, a.acceleration.y, - a.acceleration.z);
    gyro_Body  = Vec3(g.gyro.x, - g.gyro.y, g.gyro.z);
    mag_Body   = Vec3(- m.magnetic.x, - m.magnetic.y, m.magnetic.z)
                 - mag_center_Body;

    madgwickFilter.update(gyro_Body.x, gyro_Body.y, gyro_Body.z,
                          - accel_Body.x, - accel_Body.y, -accel_Body.z,
                          - mag_Body.x, -mag_Body.y, -mag_Body.z);

    if (mag_calibrating) {
        calibrateMag(mag_Body, mag_center_Body, mag_radius, CALIBRATION_GAIN);

        mag_Body.print('M');
        mag_center_Body.print('C');
        Serial.println("");
    }

    float pressure = lps25hb.readPressureMillibars();
    prs_altitude   = lps25hb.pressureToAltitudeMeters(pressure, qnh);
    temperature    = lps25hb.readTemperatureC();
}

void calcAndSendNav() {
    attitude = madgwickFilter.getQuaternion();
    accel_ENU = attitude.rotate(accel_Body);

    // Serial.print(",Pitch:");
    // Serial.print(madgwickFilter.getPitch());
    // Serial.print(",Roll:");
    // Serial.print(madgwickFilter.getRoll());
    // Serial.print(",Yaw:");
    // Serial.print(madgwickFilter.getYaw());
    // Serial.print(",");

    // accel_Body.print('A');
    gyro_Body.print('G');
    // mag_Body.print('M');
    // if (mag_calibrating)
    // mag_center_Body.print('C');
    accel_ENU.print('a');
    attitude.print();
    // location_LLH.print('L');
    // position_ENU.print('P');
    Serial.println("");

    mesh.send('P', 0, (float)position_ENU.x);
    mesh.send('P', 1, (float)position_ENU.y);
    mesh.send('P', 2, (float)position_ENU.z);
    mesh.send('Q', 0, (float)attitude.a);
    mesh.send('Q', 1, (float)attitude.b);
    mesh.send('Q', 2, (float)attitude.c);
    mesh.send('Q', 3, (float)attitude.d);
}

float calibrateMag(const Vec3& mag, Vec3& center, float& radius, float gain) {
    float f = mag.norm() - radius*radius;
    center += mag * (4 * gain * f);
    radius   += 4 * gain * f * radius;
    if (f > 1E10) {
        center = Vec3();
        radius   = 1;
    }
    return f;
}
