#pragma once

class Vec3 {
public:
    float x;
    float y;
    float z;

    Vec3(): x(0), y(0), z(0) {};
    Vec3(float x, float y, float z): x(x), y(y), z(z) {};

    Vec3 operator+(const Vec3 &v) const { return Vec3(*this) += v; };
    Vec3 operator-(const Vec3 &v) const { return Vec3(*this) -= v; };
    Vec3 operator*(float a) const { return Vec3(*this) *= a; };
    Vec3 operator/(float a) const { return Vec3(*this) /= a; };
    Vec3 operator-() const { return Vec3() -= (*this); };
    Vec3 operator+() const { return Vec3(*this); };
    float operator[](unsigned i) const;
    float & operator[](unsigned i);

    Vec3 & operator=(const Vec3 &rhs) {
        x = rhs.x;
        y = rhs.y;
        z = rhs.z;
        return *this;
    }
    Vec3 & operator+=(const Vec3 &v);
    Vec3 & operator-=(const Vec3 &v);
    Vec3 & operator*=(float a);
    Vec3 & operator/=(float a);

    float norm() const;
    Vec3 normalized() const;

    void print(char prefix = '\0');
};

class Quaternion {
public:
    float a;
    float b;
    float c;
    float d;

    Quaternion() { a = 1; b = c = d = 0; };
    Quaternion(float a, float b, float c, float d): a(a), b(b), c(c), d(d) {};
    Quaternion(const Vec3 &v) { a = 0; b = v.x; c = v.y; d = v.z; };

    Quaternion operator*(const Quaternion &q) const {
        return Quaternion(*this) *= q;
    };
    Quaternion operator*(const Vec3 &v) const {
        return Quaternion(*this) *= Quaternion(v);
    }

    float operator[](unsigned i) const;
    float & operator[](unsigned i);

    Quaternion & operator=(const Quaternion &rhs) {
        a = rhs.a;
        b = rhs.b;
        c = rhs.c;
        d = rhs.d;
        return *this;
    }
    Quaternion & operator*=(const Quaternion &q);

    Quaternion inverse() const;
    Vec3 rotate(Vec3 &v) const;

    void print();
};
