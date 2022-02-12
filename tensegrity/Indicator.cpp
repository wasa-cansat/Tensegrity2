#include "Indicator.h"

void Indicator::init() {
    leds = Adafruit_NeoPixel(3, pin, NEO_GRB + NEO_KHZ800);
    leds.begin();
}



void Indicator::update() {
    int time = millis();
    leds.clear();

    int b0, b1, b2;

    switch (state) {
    case StartUp:
        for (int i = 0; i < 3; i++) {
            leds.setPixelColor
                (i,
                 leds.ColorHSV((time * 100 + i * 5000)%65536, 255, brightness));
        }

        break;

    case Moving:

        b0 = max(abs(time % 500 - 250) - 100, 0) * brightness / 150;
        b1 = b0;
        b2 = b0;
        if (moving == 1) {
            b0 = max(abs((time + 100) % 500 - 250) - 100, 0) * brightness / 150;
            b2 = max(abs((time - 100) % 500 - 250) - 100, 0) * brightness / 150;
        }
        else if (moving == -1) {
            b0 = max(abs((time - 100) % 500 - 250) - 100, 0) * brightness / 150;
            b2 = max(abs((time + 100) % 500 - 250) - 100, 0) * brightness / 150;
        }
        if (position ==  1) b2 = (time % 200 >= 100) ? brightness : 0;
        if (position == -1) b0 = (time % 200 >= 100) ? brightness : 0;
        leds.setPixelColor(0, leds.ColorHSV(20000, 200, b0));
        leds.setPixelColor(1, leds.ColorHSV(20000, 200, b1));
        leds.setPixelColor(2, leds.ColorHSV(20000, 200, b2));

        break;

    case Error:
        b0 = max(abs( time % 200 - 100) - 40, 0) * brightness / 60;
        leds.setPixelColor(0, leds.ColorHSV(0, 255, b0));
        leds.setPixelColor(1, leds.ColorHSV(0, 255, b0));
        leds.setPixelColor(2, leds.ColorHSV(0, 255, b0));

        break;
    }

    leds.show();
}
