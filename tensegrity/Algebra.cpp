#include "Algebra.h"

#include <Arduino.h>

float Vec3::operator[](unsigned i) const {
    switch (i) {
    case 0:
        return x;
    case 1:
        return y;
    case 2:
        return z;
    default:
        return 0;
    }
}
float & Vec3::operator[](unsigned i) {
    switch (i) {
    case 0:
        return x;
    case 1:
        return y;
    case 2:
        return z;
    default:
        return x;
    }
}

Vec3 & Vec3::operator+=(const Vec3 &v) {
    x += v.x;
    y += v.y;
    z += v.z;
    return *this;
}

Vec3 & Vec3::operator-=(const Vec3 &v) {
    x -= v.x;
    y -= v.y;
    z -= v.z;
    return *this;
}

Vec3 & Vec3::operator*=(float a) {
    x *= a;
    y *= a;
    z *= a;
    return *this;
}

Vec3 & Vec3::operator/=(float a) {
    x /= a;
    y /= a;
    z /= a;
    return *this;
}

float Vec3::norm() const {
    return sqrt(x * x + y * y + z * z);
}

Vec3 Vec3::normalized() const {
    float n = norm();
    return Vec3(x / n, y / n, z / n);
}


void Vec3::print(char prefix) {
    if (prefix == '\0') Serial.printf("x:%f,y:%f,z:%f,", x, y, z);
    else Serial.printf("%cx:%f,%cy:%f,%cz:%f,",
                       prefix, x, prefix, y, prefix, z);
}

float Quaternion::operator[](unsigned i) const {
    switch (i) {
    case 0:
        return a;
    case 1:
        return b;
    case 2:
        return c;
    case 3:
        return d;
    default:
        return 0;
    }
}
float & Quaternion::operator[](unsigned i) {
    switch (i) {
    case 0:
        return a;
    case 1:
        return b;
    case 2:
        return c;
    case 3:
        return d;
    default:
        return a;
    }
}

Quaternion & Quaternion::operator*=(const Quaternion &q) {
    Quaternion ret;
    ret.a = a*q.a - b*q.b - c*q.c - d*q.d;
    ret.b = b*q.a + a*q.b + c*q.d - d*q.c;
    ret.c = a*q.c - b*q.d + c*q.a + d*q.b;
    ret.d = a*q.d + b*q.c - c*q.b + d*q.a;
    return (*this = ret);
}

Quaternion Quaternion::inverse() const {
    return Quaternion(a, -b, -c, -d);
}

Vec3 Quaternion::rotate(Vec3 &v) const {
    Quaternion q = (*this) * Quaternion(v) * inverse();
    return Vec3(q.b, q.c, q.d);
}

void Quaternion::print() {
    Serial.printf("a:%f,b:%f,c:%f,d:%f,", a, b, c, d);
}

