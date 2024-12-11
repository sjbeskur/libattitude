#ifndef QUATERNION_H
#define QUATERNION_H

#include <cmath>
#include <iostream>
#include "euler.h"
#include "vector3.h"
#include "matrix3.h"
#include "dcm.h"

class Vector3;
class EulerAngles;
class DCM;

namespace AML
{
    class Quaternion
    {
    public:
        // Quaternion Data
        union{        
            double data[4];
            struct { double q0, q1, q2, q3; };
        };

        Quaternion();
        explicit Quaternion(double q0, double q1, double q2, double q3);
        explicit Quaternion(double val);
        explicit Quaternion(const double data[4]);
        explicit Quaternion(double scalar, const Vector3& vec);
        explicit Quaternion(const Vector3& rhs);
        
        ~Quaternion();

        // Operator Assignments (Quaternion)
        Quaternion& operator+=(const Quaternion& rhs);
        Quaternion& operator-=(const Quaternion& rhs);
        Quaternion& operator*=(const Quaternion& rhs);

        // Operator Assignments (scalar)
        Quaternion& operator+=(double rhs);
        Quaternion& operator-=(double rhs);
        Quaternion& operator*=(double rhs);
        Quaternion& operator/=(double rhs);

        // Special
        static const Quaternion identity();

    private:
        /* data */

    };

// Quaternion / Quaternion Operations
Quaternion operator-(const Quaternion& rhs);
Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs);
Quaternion operator-(const Quaternion& lhs, const Quaternion& rhs);
Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs);


// Quaternion / Scalar Operations
Quaternion operator+(const Quaternion& lhs, double s);
Quaternion operator-(const Quaternion& lhs, double s);
Quaternion operator*(const Quaternion& lhs, double s);
Quaternion operator/(const Quaternion& lhs, double s);
Quaternion operator+(double s, const Quaternion& rhs);
Quaternion operator-(double s, const Quaternion& rhs);
Quaternion operator*(double s, const Quaternion& rhs);
Quaternion operator/(double s, const Quaternion& rhs);

// Quaternion / Vector Operations
Vector3 operator*(const Quaternion& lhs, const Vector3& rhs);

// Quaternion operations
Quaternion conjugate(const Quaternion& rhs);
double norm(const Quaternion& rhs);
void normalize(Quaternion& rhs);
Quaternion inverse(const Quaternion& rhs);
Quaternion unit(const Quaternion& rhs);
bool isUnitQuat(const Quaternion& rhs, double tol = std::numeric_limits<double>::epsilon());
double dot(const Quaternion& lhs, const Quaternion& rhs);

// Attitude Conversion Functions
Matrix3x3 Quat2DCM(const Quaternion& rhs);
Quaternion DCM2Quat(const Matrix3x3& rhs);
EulerAngles quat2EulerAngles(const Quaternion& rhs, const EulerAngles::EulerSequence seq = EulerAngles::EulerSequence::XYZ);
Quaternion eulerAngles2Quat(const EulerAngles& rhs);

// Euler Angles to Quaternion
Quaternion eulerAngles2Quat_ZXZ(double phi, double theta, double psi);
Quaternion eulerAngles2Quat_XYX(double phi, double theta, double psi);
Quaternion eulerAngles2Quat_YZY(double phi, double theta, double psi);
Quaternion eulerAngles2Quat_ZYZ(double phi, double theta, double psi);
Quaternion eulerAngles2Quat_XZX(double phi, double theta, double psi);
Quaternion eulerAngles2Quat_YXY(double phi, double theta, double psi);
Quaternion eulerAngles2Quat_XYZ(double phi, double theta, double psi);
Quaternion eulerAngles2Quat_YZX(double phi, double theta, double psi);
Quaternion eulerAngles2Quat_ZXY(double phi, double theta, double psi);
Quaternion eulerAngles2Quat_XZY(double phi, double theta, double psi);
Quaternion eulerAngles2Quat_ZYX(double phi, double theta, double psi);
Quaternion eulerAngles2Quat_XZY(double phi, double theta, double psi);

// Quaternion to Euler 
EulerAngles quat2EulerAngles(double phi, double theta, double psi);
EulerAngles quat2EulerAngles_XYX(double phi, double theta, double psi);
EulerAngles quat2EulerAngles_YZY(double phi, double theta, double psi);
EulerAngles quat2EulerAngles_ZYZ(double phi, double theta, double psi);
EulerAngles quat2EulerAngles_XZX(double phi, double theta, double psi);
EulerAngles quat2EulerAngles_YXY(double phi, double theta, double psi);
EulerAngles quat2EulerAngles_XYZ(double phi, double theta, double psi);
EulerAngles quat2EulerAngles_YZX(double phi, double theta, double psi);
EulerAngles quat2EulerAngles_ZXY(double phi, double theta, double psi);
EulerAngles quat2EulerAngles_XZY(double phi, double theta, double psi);
EulerAngles quat2EulerAngles_ZYX(double phi, double theta, double psi);
EulerAngles quat2EulerAngles_XZY(double phi, double theta, double psi);

// Quaternion Kinematic Functions
Quaternion integrateQuat(const Quaternion& quat, const Quaternion& quatRates, double dt);
Quaternion quatKinematicRates_BodyRates(const Quaternion& quat, const Vector3& bodyRates);
Quaternion quatKinematicRates_WorldRates(const Quaternion& quat, const Vector3& bodyRates);

// Quaternion Interpolation
Quaternion linearInterpolate(const Quaternion& startQuat, const Quaternion& endQuat, double t );
Quaternion slerpInterpolate(const Quaternion& startQuat, const Quaternion& endQuat, double t );


// Stream Functions
std::ostream& operator<<(std::ostream& os, const Quaternion& obj);

};


#endif
