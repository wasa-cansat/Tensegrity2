#include "NavUnit.h"

// Pin assigns
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define GPS_TX_PIN  5
#define GPS_RX_PIN  25

#define GPS_BAUD 9600

// #define LSM9DS1_M_ADDR  0x1E
// #define LSM9DS1_AG_ADDR 0x6B

// Task
#define READ_LOCATION_FREQ     1
#define READ_IMU_FREQ          1
#define CALC_AND_SEND_NAV_FREQ 1

// Sensors
Adafruit_LSM9DS1 lsm9ds1;
LPS lps25hb;
TinyGPSPlus gps;



// Values
float prs_altitude = 0;
float gps_altitude = 0;
float qnh = 0;
float temperature = 0;
float latitude = 0;
float longitude = 0;
float lat_start = 0;
float lng_start = 0;
bool  gps_activated = false;

Vec3 location;
Vec3 accel;
Vec3 mag;
Vec3 gyro;

// Task
void readLocation();
Task taskReadLocation(TASK_SECOND / READ_LOCATION_FREQ, TASK_FOREVER,
                      &readLocation);

void readIMU();
Task taskReadIMU(TASK_SECOND / READ_IMU_FREQ, TASK_FOREVER, &readIMU);

void calcAndSendNav();
Task taskCalcAndSendNav(TASK_SECOND / READ_IMU_FREQ, TASK_FOREVER,
                        &calcAndSendNav);


float calibrateMag(Vec3 &mag, Vec3 &center, float &radius, float gain = 0.0001);

void nav_init() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);


    bool imu_ok = lsm9ds1.begin();
    if (!imu_ok) log_e("Failed to initialize IMU");

    lsm9ds1.setupAccel(lsm9ds1.LSM9DS1_ACCELRANGE_8G);
    lsm9ds1.setupMag(lsm9ds1.LSM9DS1_MAGGAIN_4GAUSS);
    lsm9ds1.setupGyro(lsm9ds1.LSM9DS1_GYROSCALE_500DPS);

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
    while(Serial2.available() > 0){
        char c = Serial2.read();
        Serial.print(c);
        gps.encode(c);
        if(gps.location.isUpdated()){
            latitude  = gps.location.lat();
            longitude = gps.location.lng();
            gps_altitude = gps.altitude.meters();
        }
        if (!gps_activated) {
            lat_start = latitude;
            lng_start = longitude;
            gps_activated = true;
        }
    }
    // while (Serial2.available()) {
    //     Serial.print(Serial2.read());
    // }
    Serial.println("]");
}

void readIMU() {
    i2c_scanner();

    lsm9ds1.read();
    sensors_event_t a, m, g, temp;
    lsm9ds1.getEvent(&a, &m, &g, &temp);

    accel = Vec3(a.acceleration.x, a.acceleration.y, a.acceleration.z);
    gyro  = Vec3(g.gyro.x, g.gyro.y, g.gyro.z);
    mag   = Vec3(m.magnetic.x, m.magnetic.y, m.magnetic.z);

    // if (IMU.accelerationAvailable())
    //     IMU.readAcceleration(accel.x, accel.y, accel.z);
    // if (IMU.gyroscopeAvailable())
    //     IMU.readGyroscope(gyro.x, gyro.y, gyro.z);
    // if (IMU.magneticFieldAvailable())
    //     IMU.readMagneticField(mag.x, mag.y, mag.z);

    accel.print('A');
    gyro.print('G');
    mag.print('M');

    float pressure = lps25hb.readPressureMillibars();
    prs_altitude    = lps25hb.pressureToAltitudeMeters(pressure, qnh);
    temperature = lps25hb.readTemperatureC();

    // Serial.print("Ax: ");
    // Serial.print(lsm9ds1.ax);
    // Serial.print(",Ay: ");
    // Serial.print(lsm9ds1.ay);
    // Serial.print(",Az: ");
    // Serial.print(lsm9ds1.az);
    Serial.print(",longitude: ");
    Serial.print(longitude);
    Serial.print(",latitude: ");
    Serial.print(latitude);
    Serial.print(",prs: ");
    Serial.print(pressure);
    Serial.print(",prs_alt: ");
    Serial.print(prs_altitude);
    Serial.print(",alt: ");
    Serial.print(gps_altitude);
    Serial.print(",tmp: ");
    Serial.print(temperature);
    Serial.println("");
}

void calcAndSendNav() {
    mesh.send('A', 0, (float)prs_altitude);
}

float calibrateMag(const Vec3& mag, Vec3& center, float& radius, float gain) {
    float f = mag.x * mag.x + mag.y * mag.y + mag.z * mag.z - radius*radius;
    center.x += 4 * gain * f * mag.x;
    center.y += 4 * gain * f * mag.y;
    center.z += 4 * gain * f * mag.z;
    radius   += 4 * gain * f * radius;
    if (f > 1E30) {
        center.x = 0;
        center.y = 0;
        center.z = 0;
        radius   = 1;
    }
    return f;
}
