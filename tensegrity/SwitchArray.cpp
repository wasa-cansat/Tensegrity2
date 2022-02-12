#include "SwitchArray.h"

void SwitchArray::init() {
    pinMode(pin, ANALOG);
    analogSetPinAttenuation(pin, ADC_6db);
    powN = pow(2, N);
    ruler = (unsigned *)malloc(sizeof(unsigned) * powN);
}

void SwitchArray::record() {
    Serial.println("Start recording switch array values.");
    for (int i = 0; i < powN; i++) {
        if (i == 0) Serial.print("Don't press any switches... ");
        else {
            Serial.print("Press switch ");
            for (int n = 0; n < N; n++) {
                if (i >> n & 0b1) {
                    Serial.print(n);
                    Serial.print(" ");
                }
            }
            Serial.print("... ");
        }
        delay(3000);
        Serial.print("3 ");
        delay(1000);
        Serial.print("2 ");
        delay(1000);
        Serial.print("1 ");
        delay(1000);
        Serial.print("reading... ");
        unsigned long sum = 0;
        for (int t = 0; t < 20; t++) {
            sum += analogRead(pin);
            delay(50);
        }
        ruler[i] = sum / 20;
        Serial.print(ruler[i]);
        Serial.println(" OK");
    }
    Serial.println("Complete recording.");
}

unsigned SwitchArray::EEPROM_size() {
    return sizeof(uint16_t) * powN;
}

void SwitchArray::load_ruler(unsigned addr) {
    for (int i = 0; i < powN; i++) {
        uint16_t v;
        EEPROM.get<uint16_t>(addr + i * sizeof(uint16_t), v);
        ruler[i] = v;
    }

    Serial.print("Loaded switch array values: ");
    for (int i = 0; i < powN; i++) {
        Serial.print(ruler[i]);
        Serial.print(" ");
    }
    Serial.println("");
}
void SwitchArray::store_ruler(unsigned addr) {
    for (int i = 0; i < powN; i++)
        EEPROM.put<uint16_t>(addr + i * sizeof(uint16_t), (uint16_t)ruler[i]);
    EEPROM.commit();

    Serial.print("Stored switch array values: ");
    for (int i = 0; i < powN; i++) {
        Serial.print(ruler[i]);
        Serial.print(" ");
    }
    Serial.println("");
}

void SwitchArray::update() {
    int value = analogRead(pin);

    int minimum = 4096;
    for (int i = 0; i < powN; i++) {
        int dist = abs(value - (int)ruler[i]);
        if (dist < minimum) {
            minimum = dist;
            pressed = i;
        }
    }
}

bool SwitchArray::isPressed(uint8_t n) {
    return pressed >> n & 0b1;
}

void SwitchArray::print() {
    Serial.print("Pressed: ");
    update();
    for (int n = 0; n < N; n++) {
        if (isPressed(n)) {
            Serial.print(n);
            Serial.print(", ");
        }
    }
    Serial.println();
}
