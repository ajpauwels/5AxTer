#ifndef UTILITY
#define UTILITY

#include <math.h>

// Defines multiple dimensional array types for representing matrices
// typedef float (*Matrix3x8)[8];

struct Matrix3x1 {
  float m[3][1];
};

struct Matrix3x3 {
  float m[3][3];
};

struct Matrix3x8 {
  float m[3][8];
};

// Stores a vector of 3 floating-point values
// NOTE: This is Cura's code, we should replace this
class FPoint3 {
public:
    float x,y,z;
    FPoint3() {}
    FPoint3(float _x, float _y, float _z): x(_x), y(_y), z(_z) {}

    FPoint3 operator+(const FPoint3& p) const { return FPoint3(x+p.x, y+p.y, z+p.z); }
    FPoint3 operator-(const FPoint3& p) const { return FPoint3(x-p.x, y-p.y, z-p.z); }
    FPoint3 operator*(const float f) const { return FPoint3(x*f, y*f, z*f); }
    FPoint3 operator/(const float f) const { return FPoint3(x/f, y/f, z/f); }

    FPoint3& operator += (const FPoint3& p) { x += p.x; y += p.y; z += p.z; return *this; }
    FPoint3& operator -= (const FPoint3& p) { x -= p.x; y -= p.y; z -= p.z; return *this; }
    FPoint3& operator *= (const float f) { x *= f; y *= f; z *= f; return *this; }

    bool operator==(FPoint3& p) const { return x==p.x&&y==p.y&&z==p.z; }
    bool operator!=(FPoint3& p) const { return x!=p.x||y!=p.y||z!=p.z; }

    float max() const
    {
        if (x > y && x > z) return x;
        if (y > z) return y;
        return z;
    }

    bool testLength(float len) const
    {
        return vSize2() <= len*len;
    }

    float vSize2() const
    {
        return x*x+y*y+z*z;
    }

    float vSize() const
    {
        return sqrt(vSize2());
    }

    inline FPoint3 normalized() const
    {
        return (*this)/vSize();
    }

    FPoint3 cross(const FPoint3& p) const
    {
        return FPoint3(
            y*p.z-z*p.y,
            z*p.x-x*p.z,
            x*p.y-y*p.x);
    }
};

#endif
