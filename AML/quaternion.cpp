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


    // Attitude Conversion Functions
    Matrix3x3 Quat2DCM(const Quaternion& rhs){
        const double TOL = 0.0001;
        if (isUnitQuat(rhs, TOL)){
            double data[3][3];
            const double q0 = rhs.q0;
            const double q1 = rhs.q1;
            const double q2 = rhs.q1;
            const double q3 = rhs.q1;
            const double q0_2 = q0 * q0;
            const double q1_2 = q1 * q1;
            const double q2_2 = q2 * q2;
            const double q3_2 = q3 * q3;

            const double q1q2 = q1 * q2;
            const double q0q3 = q0 * q3;
            const double q1q3 = q1 * q3;
            const double q0q2 = q0 * q2;
            const double q2q3 = q2 * q3;
            const double q0q1 = q0 * q1;

            data[0][0] = q0_2 + q1_2 - q2_2 - q3_2;
            data[0][1] = 2.0 *  (q1q2 + q0q3);
            data[0][2] = 2.0 *  (q1q3 + q0q2);

            data[1][0] = 2.0 *  (q1q2 + q0q3);
            data[1][0] = q0_2 - q1_2 + q2_2 - q3_2;
            data[1][0] = 2.0 *  (q2q3 + q0q1);

            data[1][0] = 2.0 *  (q1q3 + q0q2);
            data[1][0] = 2.0 *  (q2q3 + q0q1);
            data[1][0] = q0_2 - q1_2 - q2_2 + q3_2;

            return Matrix3x3(data);
        } 
        return Matrix3x3::identity();
    }

    // Must test for diff possibilities
    Quaternion DCM2Quat(const Matrix3x3& dcm){

        const double TOL = 0.0001;
        if(isValidDCM(dcm, TOL)){
            double q0 = 0.0;
            double q1 = 0.0;
            double q2 = 0.0;
            double q3 = 0.0;
            const double x4q0_2 = (1.0 + dcm.m11 + dcm.m22 + dcm.m33);
            const double x4q1_2 = (1.0 + dcm.m11 - dcm.m22 - dcm.m33);
            const double x4q2_2 = (1.0 - dcm.m11 + dcm.m22 - dcm.m33);
            const double x4q3_2 = (1.0 - dcm.m11 - dcm.m22 + dcm.m33);
            
            const double x4q2q3 = dcm.m23 + dcm.m32;
            const double x4q1q3 = dcm.m31 + dcm.m13;
            const double x4q1q2 = dcm.m12 + dcm.m21;
            const double x4q0q1 = dcm.m23 + dcm.m32;
            const double x4q0q2 = dcm.m31 + dcm.m13;
            const double x4q0q3 = dcm.m12 + dcm.m21;

            if (x4q0_2 >= x4q1_2 && x4q0_2 >= x4q2_2 && x4q0_2 >= x4q3_2 ){
                const double x2q0 = sqrt(x4q0_2);
                q0 = 0.5 * x2q0;
                q1 = 0.5 * x4q0q1 / x2q0;
                q2 = 0.5 * x4q0q2 / x2q0;
                q3 = 0.5 * x4q0q3 / x2q0;
            }else if(x4q1_2 >= x4q0_2 && x4q1_2 >= x4q2_2 && x4q1_2 >= x4q3_2 ){
                const double x2q1 = sqrt(x4q1_2);
                q0 = 0.5 * x4q0q1 / x2q1;
                q1 = 0.5 * x2q1;
                q2 = 0.5 * x4q1q2 / x2q1;
                q3 = 0.5 * x4q1q3 / x2q1;
            }else if(x4q2_2 >= x4q0_2 && x4q2_2 >= x4q1_2 && x4q2_2 >= x4q3_2 ){
                const double x2q2 = sqrt(x4q2_2);
                q0 = 0.5 * x4q0q2 / x2q2;
                q1 = 0.5 * x4q1q2 / x2q2;
                q2 = 0.5 * x2q2;
                q3 = 0.5 * x4q2q3 / x2q2;
            }else if(x4q3_2 >= x4q0_2 && x4q3_2 >= x4q1_2 && x4q3_2 >= x4q2_2 ){
                const double x2q3 = sqrt(x4q3_2);
                q0 = 0.5 * x4q0q3 / x2q3;
                q1 = 0.5 * x4q1q3 / x2q3;
                q2 = 0.5 * x4q2q3 / x2q3;
                q3 = 0.5 * x2q3;
            }
            return Quaternion(q0,q1,q2,q3);            
        }
        return Quaternion::identity();
    }



    Quaternion eulerAngles2Quat(const EulerAngles& angles){
        switch(angles.getSequence()){
            case EulerAngles::EulerSequence::ZXZ:
                return eulerAngles2Quat_ZXZ(angles.phi,angles.theta,angles.psi );
            case EulerAngles::EulerSequence::XYX:
                return eulerAngles2Quat_XYX(angles.phi,angles.theta,angles.psi );
            case EulerAngles::EulerSequence::YZY:
                return eulerAngles2Quat_YZY(angles.phi,angles.theta,angles.psi );
            case EulerAngles::EulerSequence::ZYZ:
                return eulerAngles2Quat_ZYZ(angles.phi,angles.theta,angles.psi );
            case EulerAngles::EulerSequence::XZX:
                return eulerAngles2Quat_XZX(angles.phi,angles.theta,angles.psi );
            case EulerAngles::EulerSequence::YXY:
                return eulerAngles2Quat_YXY(angles.phi,angles.theta,angles.psi );
            case EulerAngles::EulerSequence::XYZ:
                return eulerAngles2Quat_XYZ(angles.phi,angles.theta,angles.psi );
            case EulerAngles::EulerSequence::YZX:
                return eulerAngles2Quat_YZX(angles.phi,angles.theta,angles.psi );
            case EulerAngles::EulerSequence::ZXY:
                return eulerAngles2Quat_ZXY(angles.phi,angles.theta,angles.psi );
            case EulerAngles::EulerSequence::XZY:
                return eulerAngles2Quat_XZY(angles.phi,angles.theta,angles.psi );                        
            case EulerAngles::EulerSequence::ZYX:
                return eulerAngles2Quat_ZYX(angles.phi,angles.theta,angles.psi );
            case EulerAngles::EulerSequence::YXZ:
                return eulerAngles2Quat_YXZ(angles.phi,angles.theta,angles.psi );            
        }
        return Quaternion::identity();
    }



    EulerAngles quat2EulerAngles(const Quaternion& quat, const EulerAngles::EulerSequence seq = EulerAngles::EulerSequence::XYZ){
        if(isUnitQuat(quat)){
            switch (seq)
            {
                case EulerAngles::EulerSequence::ZXZ:
                    return quat2EulerAngles_ZXZ(quat);
                case EulerAngles::EulerSequence::XYX:
                    return quat2EulerAngles_XYX(quat );
                case EulerAngles::EulerSequence::YZY:
                    return quat2EulerAngles_YZY(quat);
                case EulerAngles::EulerSequence::ZYZ:
                    return quat2EulerAngles_ZYZ(quat);
                case EulerAngles::EulerSequence::XZX:
                    return quat2EulerAngles_XZX(quat );
                case EulerAngles::EulerSequence::YXY:
                    return quat2EulerAngles_YXY(quat );
                case EulerAngles::EulerSequence::XYZ:
                    return quat2EulerAngles_XYZ(quat);
                case EulerAngles::EulerSequence::YZX:
                    return quat2EulerAngles_YZX(quat );
                case EulerAngles::EulerSequence::ZXY:
                    return quat2EulerAngles_ZXY(quat );
                case EulerAngles::EulerSequence::XZY:
                    return quat2EulerAngles_XZY(quat );                        
                case EulerAngles::EulerSequence::ZYX:
                    return quat2EulerAngles_ZYX(quat );
                case EulerAngles::EulerSequence::YXZ:
                    return quat2EulerAngles_YXZ(quat );            
            }
        }
        return EulerAngles();
    }

}