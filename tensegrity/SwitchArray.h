#include <Arduino.h>
#include <EEPROM.h>

class SwitchArray {
private:
    uint8_t pin;
    uint8_t N;

    unsigned powN;
    unsigned *ruler;

    unsigned pressed = 0;

public:
    SwitchArray(uint8_t pin, uint8_t N): pin(pin), N(N) {};

    void init();
    void record();

    unsigned EEPROM_size();
    void load_ruler(unsigned addr);
    void store_ruler(unsigned addr);

    void update();
    bool isPressed(uint8_t n);

    void print();
};
