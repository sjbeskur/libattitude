#ifndef AML_VECTOR3_H
#define AML_VECTOR3_H

#include <iostream>

namespace AML
{

    class Vector3
    {
    public:
        // Vector Data
        union
        {
            double data[3];
            struct
            {
                double x, y, z;
            };
        };

        // Constructor
        Vector3();
        Vector3(double val);
        Vector3(double x, double y, double z);
        Vector3(const double data[3]);

        // Operator Assignment (vector)
        // v1 += v2;
        // v1 += 1.0;
        Vector3 &operator+=(const Vector3 &rhs);
        Vector3 &operator-=(const Vector3 &rhs);
        Vector3 &operator*=(const Vector3 &rhs);
        Vector3 &operator/=(const Vector3 &rhs);

        // Operator Assignment (vector)
        Vector3 &operator+=(double s);
        Vector3 &operator-=(double s);
        Vector3 &operator*=(double s);
        Vector3 &operator/=(double s);

        // Special Object Creators
        static Vector3 xAxis();
        static Vector3 yAxis();
        static Vector3 zAxis();
    };

    // Elementwise operations
    Vector3 operator-(const Vector3 &rhs);

    Vector3 operator+(const Vector3 &lhs, const Vector3 &rhs);
    Vector3 operator-(const Vector3 &lhs, const Vector3 &rhs);
    Vector3 operator*(const Vector3 &lhs, const Vector3 &rhs);
    Vector3 operator/(const Vector3 &lhs, const Vector3 &rhs);

    // Scalar operations
    Vector3 operator+(const Vector3 &lhs, double s);
    Vector3 operator-(const Vector3 &lhs, double s);
    Vector3 operator*(const Vector3 &lhs, double s);
    Vector3 operator/(const Vector3 &lhs, double s);

    Vector3 operator+(double s, const Vector3 &rhs);
    Vector3 operator-(double s, const Vector3 &rhs);
    Vector3 operator*(double s, const Vector3 &rhs);
    Vector3 operator/(double s, const Vector3 &rhs);

    // Vector operations
    double norm(const Vector3 &rhs);
    void normalize(Vector3 &rhs);
    Vector3 unit(const Vector3 &rhs);
    Vector3 cross(const Vector3 &rhs, const Vector3 &lhs);
    double dot(const Vector3 &rhs, const Vector3 &lhs);

    // Stream functions
    std::ostream &operator<<(std::ostream &os, const Vector3 &obj);
}

#endif