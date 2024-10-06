#include "AML.h"
#include "matrix3.h"
#include <cmath>

namespace AML
{
    // class functions:
    // clang-format on
    Matrix3x3::Matrix3x3() : Matrix3x3(0.0) {}

    Matrix3x3::Matrix3x3(double val) : m11(val), m12(val), m13(val),
                                       m21(val), m22(val), m23(val),
                                       m31(val), m32(val), m33(val) {}

    Matrix3x3::Matrix3x3(const double data[9]) : m11(data[0]), m12(data[1]), m13(data[2]),
                                                 m21(data[3]), m22(data[4]), m23(data[5]),
                                                 m31(data[6]), m32(data[7]), m33(data[8]) {}

    Matrix3x3::Matrix3x3(const double data[3][3]) : m11(data[0][0]), m12(data[0][1]), m13(data[0][2]),
                                                    m21(data[1][0]), m22(data[1][1]), m23(data[1][2]),
                                                    m31(data[2][0]), m32(data[2][1]), m33(data[2][2]) {}

    Matrix3x3::Matrix3x3(const Vector3 &v1, const Vector3 &v2, const Vector3 &v3) : m11(v1.x), m12(v2.x), m13(v3.x),
                                                                                    m21(v1.y), m22(v2.y), m23(v3.y),
                                                                                    m31(v1.z), m32(v2.z), m33(v3.z) {}
    // clang-format off

    // Operator Assignments    
    Matrix3x3 &Matrix3x3::operator+=(const Matrix3x3 &rhs) { 
         m11 += rhs.m11; 
         m12 += rhs.m12;
         m13 += rhs.m13;
         m21 += rhs.m21;
         m22 += rhs.m22;
         m23 += rhs.m23;
         m31 += rhs.m31;
         m32 += rhs.m32;
         m33 += rhs.m33;
         return *this;
    }

    Matrix3x3 &Matrix3x3::operator-=(const Matrix3x3 &rhs) {
         m11 -= rhs.m11; 
         m12 -= rhs.m12;
         m13 -= rhs.m13;
         m21 -= rhs.m21;
         m22 -= rhs.m22;
         m23 -= rhs.m23;
         m31 -= rhs.m31;
         m32 -= rhs.m32;
         m33 -= rhs.m33;
         return *this;

    }

    Matrix3x3 &Matrix3x3::operator*=(const Matrix3x3 &rhs) {
        double m11_temp = (m11 * rhs.m11) + (m12 * rhs.m21) + (m13 * rhs.m31);
        double m12_temp = (m11 * rhs.m12) + (m12 * rhs.m22) + (m13 * rhs.m32);
        double m13_temp = (m11 * rhs.m13) + (m12 * rhs.m23) + (m13 * rhs.m33);

        double m21_temp = (m21 * rhs.m11) + (m22 * rhs.m21) + (m23 * rhs.m31);
        double m22_temp = (m21 * rhs.m12) + (m22 * rhs.m22) + (m23 * rhs.m32);
        double m23_temp = (m21 * rhs.m13) + (m22 * rhs.m23) + (m23 * rhs.m33);

        double m31_temp = (m31 * rhs.m11) + (m32 * rhs.m21) + (m33 * rhs.m31);
        double m32_temp = (m31 * rhs.m12) + (m32 * rhs.m22) + (m33 * rhs.m32);
        double m33_temp = (m31 * rhs.m13) + (m32 * rhs.m23) + (m33 * rhs.m33);

         m11 = m11_temp; 
         m12 = m12_temp; 
         m13 = m13_temp; 
         m21 = m21_temp; 
         m22 = m22_temp; 
         m23 = m23_temp; 
         m31 = m31_temp; 
         m32 = m32_temp;
         m33 = m33_temp;
         return *this;
    }

    Matrix3x3 &Matrix3x3::operator/=(const Matrix3x3 &rhs) {

         (*this) *= inverse(rhs);
         return *this;
    }

    // Scaler
    Matrix3x3 &Matrix3x3::operator+=(double rhs) {
         m11 += rhs; 
         m12 += rhs;
         m13 += rhs;
         m21 += rhs;
         m22 += rhs;
         m23 += rhs;
         m31 += rhs;
         m32 += rhs;
         m33 += rhs; 
         return *this;       
    }

    Matrix3x3 &Matrix3x3::operator-=(double rhs) {
         m11 -= rhs; 
         m12 -= rhs;
         m13 -= rhs;
         m21 -= rhs;
         m22 -= rhs;
         m23 -= rhs;
         m31 -= rhs;
         m32 -= rhs;
         m33 -= rhs; 
         return *this;       
    }    

    Matrix3x3 &Matrix3x3::operator*=(double rhs) {
         m11 *= rhs; 
         m12 *= rhs;
         m13 *= rhs;
         m21 *= rhs;
         m22 *= rhs;
         m23 *= rhs;
         m31 *= rhs;
         m32 *= rhs;
         m33 *= rhs; 
         return *this;       
    } 

    Matrix3x3 &Matrix3x3::operator/=(double rhs) {
         m11 /= rhs; 
         m12 /= rhs;
         m13 /= rhs;
         m21 /= rhs;
         m22 /= rhs;
         m23 /= rhs;
         m31 /= rhs;
         m32 /= rhs;
         m33 /= rhs; 
         return *this;       
    }

    Matrix3x3 Matrix3x3::identity(){
        double data[3][3] = {{ 1,0,0 }, { 0,1,0 }, {0,0,1 }};
        return Matrix3x3(data);
    }

    // Negation
    Matrix3x3 operator-(const Matrix3x3 &rhs){
        double data[3][3];
        data[0][0] = -rhs.data[0][0];
        data[0][1] = -rhs.data[0][1];
        data[0][2] = -rhs.data[0][2];
        data[1][0] = -rhs.data[1][0];
        data[1][1] = -rhs.data[1][1];
        data[1][2] = -rhs.data[1][2];
        data[2][0] = -rhs.data[2][0];
        data[2][1] = -rhs.data[2][1];
        data[2][2] = -rhs.data[2][2];
        return Matrix3x3(data);
    }
    Matrix3x3 operator+(const Matrix3x3 &lhs, const Matrix3x3 &rhs){ return Matrix3x3(lhs) += rhs; }
    Matrix3x3 operator-(const Matrix3x3 &lhs, const Matrix3x3 &rhs){ return Matrix3x3(lhs) -= rhs; };
    Matrix3x3 operator*(const Matrix3x3 &lhs, const Matrix3x3 &rhs){ return Matrix3x3(lhs) *= rhs; };
    Matrix3x3 operator/(const Matrix3x3 &lhs, const Matrix3x3 &rhs){ return Matrix3x3(lhs) /= rhs; };


    // Matrix Vector Operations
    Vector3 operator*(const Matrix3x3 &lhs, const Vector3 &rhs){
         double x = lhs.m11 * rhs.x + lhs.m12 * rhs.y + lhs.m13 * rhs.z;
         double y = lhs.m21 * rhs.x + lhs.m22 * rhs.y + lhs.m23 * rhs.z;
         double z = lhs.m31 * rhs.x + lhs.m32 * rhs.y + lhs.m33 * rhs.z;
         return Vector3(x,y,z);
    }

    // Matrix Scalar Operations
    Matrix3x3 operator+(const Matrix3x3 &lhs, const double s){ return Matrix3x3(lhs) += s; }
    Matrix3x3 operator-(const Matrix3x3 &lhs, const double s){ return Matrix3x3(lhs) -= s; }
    Matrix3x3 operator*(const Matrix3x3 &lhs, const double s){ return Matrix3x3(lhs) *= s; }
    Matrix3x3 operator/(const Matrix3x3 &lhs, const double s){ return Matrix3x3(lhs) /= s; }

    Matrix3x3 operator+(const double s, const Matrix3x3 &rhs){ return Matrix3x3(s) += rhs; }
    Matrix3x3 operator-(const double s, const Matrix3x3 &rhs){ return Matrix3x3(s) -= rhs; }
    Matrix3x3 operator*(const double s, const Matrix3x3 &rhs){ return Matrix3x3(rhs) *= s; }

    Matrix3x3 operator/(const double s, const Matrix3x3 &rhs){ 
        double data[3][3];
        data[0][0] = s / rhs.data[0][0];
        data[0][1] = s / rhs.data[0][1];
        data[0][2] = s / rhs.data[0][2];
        data[1][0] = s / rhs.data[1][0];
        data[1][1] = s / rhs.data[1][1];
        data[1][2] = s / rhs.data[1][2];
        data[2][0] = s / rhs.data[2][0];
        data[2][1] = s / rhs.data[2][1];
        data[2][2] = s / rhs.data[2][2];
        return Matrix3x3(data);
    };

    // Matrix Operations
    Matrix3x3 transpose(const Matrix3x3 &rhs){
        double result[9];
        result[0] = rhs.m11;
        result[1] = rhs.m21;
        result[2] = rhs.m31;
        result[3] = rhs.m12;
        result[4] = rhs.m22;
        result[5] = rhs.m32;
        result[6] = rhs.m13;
        result[7] = rhs.m23;
        result[8] = rhs.m33;
        return Matrix3x3(result);
    }

    double determinant(const Matrix3x3 &rhs){
        double det = rhs.m11 * (rhs.m22 * rhs.m33 - rhs.m32 * rhs.m23) - 
                     rhs.m12 * (rhs.m21 * rhs.m33 - rhs.m23 * rhs.m31) + 
                     rhs.m13 * (rhs.m21 * rhs.m32 - rhs.m22 * rhs.m31);
        return det; 
    }

    Vector3 diag(const Matrix3x3 &rhs){
        return Vector3(rhs.m11, rhs.m22, rhs.m33);
    }

    Matrix3x3 diag(const Vector3 &rhs){
        double data[3][3]{0.0};
        data[0][0] = rhs.x;
        data[1][1] = rhs.y;
        data[2][2] = rhs.z;
        return Matrix3x3(data);
    }

    Matrix3x3 inverse(const Matrix3x3 &rhs){
        double det = determinant(rhs);
        if (fabs(det) > 0.0){
            double result[9];
            double invdet = 1.0 / det;
            result[0] = (rhs.m22 * rhs.m33 - rhs.m32 * rhs.m23) * invdet;
            result[1] = (rhs.m13 * rhs.m32 - rhs.m12 * rhs.m33) * invdet;
            result[2] = (rhs.m12 * rhs.m23 - rhs.m13 * rhs.m22) * invdet;
            result[3] = (rhs.m23 * rhs.m31 - rhs.m21 * rhs.m33) * invdet;
            result[4] = (rhs.m11 * rhs.m33 - rhs.m13 * rhs.m31) * invdet;
            result[5] = (rhs.m21 * rhs.m13 - rhs.m11 * rhs.m23) * invdet;
            result[6] = (rhs.m21 * rhs.m32 - rhs.m31 * rhs.m22) * invdet;
            result[7] = (rhs.m31 * rhs.m12 - rhs.m11 * rhs.m32) * invdet;            
            result[8] = (rhs.m11 * rhs.m22 - rhs.m21 * rhs.m12) * invdet;
            return Matrix3x3(result);
        }
        return Matrix3x3(NAN);
    }

    // Stream fuctions
    std::ostream &operator<<(std::ostream &os, const Matrix3x3 &obj){
        os << "[[ " << obj.m11 << " " << obj.m12 << " " << obj.m13 << "]\n"  
           << " [ " << obj.m21 << " " << obj.m22 << " " << obj.m23 << "]\n"
           << " [ " << obj.m31 << " " << obj.m32 << " " << obj.m33 << "]]\n";
        return os;
    }

}