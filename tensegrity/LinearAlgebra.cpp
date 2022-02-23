#include "LinearAlgebra.h"

void Vec3::print(char prefix) {
    if (prefix == '\0') Serial.printf("x:%f,y:%f,z:%f,", x, y, z);
    else Serial.printf("%cx:%f,%cy:%f,%cz:%f,",
                       prefix, x, prefix, y, prefix, z);
}
