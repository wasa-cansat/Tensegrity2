#include <Adafruit_NeoPixel.h>

class Indicator {
private:
    uint8_t pin;
    Adafruit_NeoPixel leds;

public:
    enum State {
        StartUp,
        Moving,
        Error,
    };
    State state = StartUp;
    int position = 0; // -1, 0, 1
    int moving   = 0; // -1, 0, 1
    unsigned brightness = 10;

    Indicator(uint8_t pin): pin(pin) {};

    void init();
    void update();
};
