#ifndef MATRIX3_H
#define MATRIX3_H

#include <iostream>

namespace AML
{
    class Vector3;

    class Matrix3x3
    {
    public:
        union
        {
            double data[3][3];
            struct
            {
                double m11, m12, m13, m21, m22, m23, m31, m32, m33;
            };
        };
        // Constructors
        Matrix3x3();
        explicit Matrix3x3(double val);
        explicit Matrix3x3(const double data[9]);
        explicit Matrix3x3(const double data[3][3]);
        explicit Matrix3x3(const Vector3 &v1, const Vector3 &v2, const Vector3 &v3);

        Matrix3x3 &operator+=(const Matrix3x3 &rhs);
        Matrix3x3 &operator-=(const Matrix3x3 &rhs);
        Matrix3x3 &operator*=(const Matrix3x3 &rhs);
        Matrix3x3 &operator/=(const Matrix3x3 &rhs);

        // Scaler
        Matrix3x3 &operator+=(double rhs);
        Matrix3x3 &operator-=(double rhs);
        Matrix3x3 &operator*=(double rhs);
        Matrix3x3 &operator/=(double rhs);

        static Matrix3x3 identity();
    };

    // Matrix Matrix Operations
    Matrix3x3 operator-(const Matrix3x3 &rhs);
    Matrix3x3 operator+(const Matrix3x3 &lhs, const Matrix3x3 &rhs);
    Matrix3x3 operator-(const Matrix3x3 &lhs, const Matrix3x3 &rhs);
    Matrix3x3 operator*(const Matrix3x3 &lhs, const Matrix3x3 &rhs);
    Matrix3x3 operator/(const Matrix3x3 &lhs, const Matrix3x3 &rhs);

    // Matrix Vector Operations
    Vector3 operator*(const Matrix3x3 &lhs, const Vector3 &rhs);

    // Matrix Scalar Operations
    Matrix3x3 operator+(const Matrix3x3 &lhs, const double s);
    Matrix3x3 operator-(const Matrix3x3 &lhs, const double s);
    Matrix3x3 operator*(const Matrix3x3 &lhs, const double s);
    Matrix3x3 operator/(const Matrix3x3 &lhs, const double s);

    Matrix3x3 operator+(const double s, const Matrix3x3 &rhs);
    Matrix3x3 operator-(const double s, const Matrix3x3 &rhs);
    Matrix3x3 operator*(const double s, const Matrix3x3 &rhs);
    Matrix3x3 operator/(const double s, const Matrix3x3 &rhs);

    // Matrix Operations
    Matrix3x3 transpose(const Matrix3x3 &rhs);
    double determinant(const Matrix3x3 &rhs);
    Vector3 diag(const Matrix3x3 &rhs);
    Matrix3x3 diag(const Vector3 &rhs);
    Matrix3x3 inverse(const Matrix3x3 &rhs);

    // Stream fuctions
    std::ostream &operator<<(std::ostream &os, const Matrix3x3 &obj);
}

#endif // MATRIX3_H