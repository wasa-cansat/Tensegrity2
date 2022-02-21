#include "NavUnit.h"

// Pin assigns
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define GPS_TX_PIN  5
#define GPS_RX_PIN  25

#define GPS_BAUD 9600

// Task
#define READ_LOCATION_FREQ 1
#define READ_IMU_FREQ      1

// Values
float altitude = 0;
float qnh = 0;
float temperature = 0;

// Task
void readLocation();
Task taskReadLocation(TASK_SECOND / READ_LOCATION_FREQ,
                      TASK_FOREVER, &readLocation);

void readIMU();
Task taskReadIMU(TASK_SECOND / READ_IMU_FREQ, TASK_FOREVER, &readIMU);

// Sensors
LPS lps25hb;

void nav_init() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    if (!lps25hb.init(LPS::device_25H))
        log_e("Failed to initialize pressure sensor");
    lps25hb.enableDefault();

    qnh = lps25hb.readPressureMillibars();

    scheduler.addTask(taskReadLocation);
    scheduler.addTask(taskReadIMU);
    taskReadLocation.enable();
    taskReadIMU.enable();
}

void readLocation() {
    while (Serial2.available()) {
        Serial.print(Serial2.read());
    }
    Serial.println("]");
}

void readIMU() {
    i2c_scanner();

    float pressure = lps25hb.readPressureMillibars();
    altitude    = lps25hb.pressureToAltitudeMeters(pressure, qnh);
    temperature = lps25hb.readTemperatureC();

    Serial.print("p: ");
    Serial.print(pressure);
    Serial.print(" mbar\ta: ");
    Serial.print(altitude);
    Serial.print(" m\tt: ");
    Serial.print(temperature);
    Serial.println(" deg C");
}
