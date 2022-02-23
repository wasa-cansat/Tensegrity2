
#include <Arduino.h>

class Vec3 {
public:
    float x;
    float y;
    float z;

    Vec3(): x(0), y(0), z(0) {};
    Vec3(float x, float y, float z): x(x), y(y), z(z) {};

    void print(char prefix = '\0');
};
