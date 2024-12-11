#include "quaternion.h"

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

    Quaternion::Quaternion():
        Quaternion(0.0, 0.0, 0.0, 0.0){}
        //q0(0.0), q1(0.0), q2(0.0), q3(0.0) {}

    Quaternion::Quaternion(double q0_, double q1_, double q2_, double q3_):
        q0(q0_), q1(q1_), q2(q2_), q3(q3_) {}

    Quaternion::Quaternion(double val):
        Quaternion(val, val, val, val){}

    Quaternion::Quaternion(const double data[4]):
        Quaternion(data[0],data[1],data[2],data[3] ){}

    Quaternion::Quaternion(double scalar, const Vector3& vec):
        Quaternion(scalar, vec.x, vec.y, vec.z){}

    Quaternion::Quaternion(const Vector3& rhs):
        Quaternion(0.0, rhs){}

    Quaternion::~Quaternion(){}

    // Operator Assignments (Quaternion)
    Quaternion& Quaternion::operator+=(const Quaternion& rhs){
        q0 += rhs.q0;
        q1 += rhs.q1;
        q2 += rhs.q2;
        q3 += rhs.q3;
        return *this;
    }

    Quaternion& Quaternion::operator-=(const Quaternion& rhs){
        q0 -= rhs.q0;
        q1 -= rhs.q1;
        q2 -= rhs.q2;
        q3 -= rhs.q3;
        return *this;
    }

    Quaternion& Quaternion::operator*=(const Quaternion& rhs){        
        double q0_new = (rhs.q0 * q0) - (rhs.q1 * q1) - (rhs.q2 * q2) - (rhs.q3 * q3);
        double q1_new = (rhs.q0 * q1) + (rhs.q1 * q0) - (rhs.q2 * q3) + (rhs.q3 * q2);
        double q2_new = (rhs.q0 * q2) + (rhs.q1 * q3) + (rhs.q2 * q0) - (rhs.q3 * q1);
        double q3_new = (rhs.q0 * q3) - (rhs.q1 * q2) + (rhs.q2 * q1) + (rhs.q3 * q0);
        q0 = q0_new;
        q1 = q1_new;
            q2 = q2_new;
        q3 = q3_new;
        return *this;
    }

    // Operator Assignments (scalar)
    Quaternion& Quaternion::operator+=(double rhs){
        q0 += rhs;
        q1 += rhs;
        q2 += rhs;
        q3 += rhs;
        return *this;
    }

    Quaternion& Quaternion::operator-=(double rhs){
        q0 -= rhs;
        q1 -= rhs;
        q2 -= rhs;
        q3 -= rhs;
        return *this;
    }

    Quaternion& Quaternion::operator*=(double rhs){
        q0 *= rhs;
        q1 *= rhs;
        q2 *= rhs;
        q3 *= rhs;
        return *this;
    }

    Quaternion& Quaternion::operator/=(double rhs){
        q0 /= rhs;
        q1 /= rhs;
        q2 /= rhs;
        q3 /= rhs;
        return *this;
    }

    // Special
    const Quaternion Quaternion::identity(){
        return Quaternion(1.0, 0.0, 0.0, 0.0);
    }

    // Stream Functions
    std::ostream& operator<<(std::ostream& os, const Quaternion& obj){
        os << "QUAT [" << obj.q0 << ", " << obj.q1 << ", " << obj.q2 << ", " << obj.q3 << "]";
        return os;
    }


    // Quaternion / Quaternion Operations
    Quaternion operator-(const Quaternion& rhs){ return Quaternion(rhs) *= -1.0; }
    Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs){ return Quaternion(lhs) += rhs;}
    Quaternion operator-(const Quaternion& lhs, const Quaternion& rhs){ return Quaternion(lhs) -= rhs;}
    Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs){ return Quaternion(lhs) *= rhs;}

    // Quaternion / Scalar Operations
    Quaternion operator+(const Quaternion& lhs, double s){ return Quaternion(lhs) += s; } 
    Quaternion operator-(const Quaternion& lhs, double s){ return Quaternion(lhs) -= s; }
    Quaternion operator*(const Quaternion& lhs, double s){ return Quaternion(lhs) *= s; }
    Quaternion operator/(const Quaternion& lhs, double s){ return Quaternion(lhs) -= s; }
    Quaternion operator+(double s, const Quaternion& rhs){ 
        return Quaternion(s + rhs.q0, s + rhs.q1, s + rhs.q2, s + rhs.q3 ); 
    } 

    Quaternion operator-(double s, const Quaternion& rhs){
        return Quaternion(s - rhs.q0, s - rhs.q1, s - rhs.q2, s - rhs.q3 ); 
    }

    Quaternion operator*(double s, const Quaternion& rhs){
        return Quaternion(s * rhs.q0, s * rhs.q1, s * rhs.q2, s * rhs.q3 ); 
    }

    Quaternion operator/(double s, const Quaternion& rhs){
        return Quaternion(s / rhs.q0, s / rhs.q1, s / rhs.q2, s / rhs.q3 ); 
    }

    // Quaternion / Vector Operations
    Vector3 operator*(const Quaternion& lhs, const Vector3& rhs){
        return Quat2DCM(lhs) * rhs; 
    }

    // Quaternion operations
    Quaternion conjugate(const Quaternion& rhs){
        return Quaternion(rhs.q0, -rhs.q1, -rhs.q2, -rhs.q3 );
    }

    double norm(const Quaternion& rhs){
        return sqrt(rhs.q0 * rhs.q0 
                  + rhs.q1 * rhs.q1 
                  + rhs.q2 * rhs.q2 
                  + rhs.q3 * rhs.q3 );
    }

    Quaternion inverse(const Quaternion& rhs){
        return conjugate(rhs) / norm(rhs);
    }

    Quaternion unit(const Quaternion& rhs){
        double mag =  norm(rhs);
        if (mag > 0.0 ) return Quaternion(rhs) /  mag;
        return Quaternion(rhs);
    }

    void normalize(Quaternion& rhs){
        double mag = norm(rhs);
        if (mag > 0.0 ) rhs /= mag;
    }

    bool isUnitQuat(const Quaternion& rhs, double tol){
        return (fabs(norm(rhs) -1.0) < 2.0 * tol);
    }

    double dot(const Quaternion& lhs, const Quaternion& rhs){
        return (  rhs.q0 * lhs.q0
                + rhs.q1 * lhs.q1
                + rhs.q2 * lhs.q2
                + rhs.q3 * lhs.q3);
    }


}