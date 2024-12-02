
#include "euler.h"
#include "vector3.h"
#include "matrix3.h"
#include "dcm.h"
#include <cmath>

namespace AML{

    // Constructors
    EulerAngles::EulerAngles()
        :phi(0.0), theta(0.0), psi(0.0), seq_(EulerSequence::XYZ){ }

    EulerAngles::EulerAngles(double phi_, double theta_, double psi_, EulerSequence seq = EulerSequence::XYZ)
    : phi(phi_), theta(theta_), psi(psi_), seq_(seq){ }

    // Stream Functions
    std::ostream& operator<<(std::ostream& os, const EulerAngles& obj){
        os << "EULER [" << obj.phi << ", " << obj.theta << ", " << obj.psi <<  "]";
        return os;
    }

    // Euler angle conversions
    Matrix3x3 Euler2DCM(const EulerAngles& angles){
        switch(angles.getSequence()){
                case EulerAngles::EulerSequence::ZXZ: {
                    return euler2DCM_ZXZ(angles.phi, angles.theta, angles.psi);
                }
                case EulerAngles::EulerSequence::XYX: {
                    return euler2DCM_XYX(angles.phi, angles.theta, angles.psi);

                }
                case EulerAngles::EulerSequence::YZY: {
                    return euler2DCM_YZY(angles.phi, angles.theta, angles.psi);
                }
                case EulerAngles::EulerSequence::ZYZ: {
                    return euler2DCM_ZYZ(angles.phi, angles.theta, angles.psi);
                }
                case EulerAngles::EulerSequence::XZX: {
                    return euler2DCM_XZX(angles.phi, angles.theta, angles.psi);
                }
                case EulerAngles::EulerSequence::YXY: {
                    return euler2DCM_YXY(angles.phi, angles.theta, angles.psi);
                }
                case EulerAngles::EulerSequence::XYZ: { // the most common
                    return euler2DCM_XYZ(angles.phi, angles.theta, angles.psi);
                }
                case EulerAngles::EulerSequence::YZX: {
                    return euler2DCM_YZX(angles.phi, angles.theta, angles.psi);
                }
                case EulerAngles::EulerSequence::ZXY: {
                    return euler2DCM_ZXY(angles.phi, angles.theta, angles.psi);
                }
                case EulerAngles::EulerSequence::XZY: {
                    return euler2DCM_XZY(angles.phi, angles.theta, angles.psi);
                }
                case EulerAngles::EulerSequence::ZYX: {
                    return euler2DCM_ZYX(angles.phi, angles.theta, angles.psi);
                }
                case EulerAngles::EulerSequence::YXZ: {
                    return euler2DCM_YXZ(angles.phi, angles.theta, angles.psi);
                }
        }
        return Matrix3x3::identity();
    }

    
    EulerAngles DCM2Euler(const Matrix3x3& dcm, const EulerAngles::EulerSequence seq = EulerAngles::EulerSequence::XYZ){
        // check that the dcm is valid
        const double TOL = 0.0001;
        if (isValidDCM(dcm, TOL)){
            switch (seq)
            {
                case EulerAngles::EulerSequence::ZXZ:  
                    return DCM2euler_ZXZ(dcm);
                case EulerAngles::EulerSequence::XYX:  
                    return DCM2euler_XYX(dcm);
                case EulerAngles::EulerSequence::YZY:  
                    return DCM2euler_YZY(dcm);
                case EulerAngles::EulerSequence::ZYZ:  
                    return DCM2euler_ZYZ(dcm);
                case EulerAngles::EulerSequence::XZX:  
                    return DCM2euler_XZX(dcm);
                case EulerAngles::EulerSequence::YXY:  
                    return DCM2euler_YXY(dcm);

                case EulerAngles::EulerSequence::XYZ:  
                    return DCM2euler_XYZ(dcm);
                case EulerAngles::EulerSequence::YZX:  
                    return DCM2euler_YZX(dcm);
                case EulerAngles::EulerSequence::ZXY:  
                    return DCM2euler_ZXY(dcm);
                case EulerAngles::EulerSequence::XZY:  
                    return DCM2euler_XZY(dcm);
                case EulerAngles::EulerSequence::ZYX:  
                    return DCM2euler_ZYX(dcm);
                case EulerAngles::EulerSequence::YXZ:  
                    return DCM2euler_YXZ(dcm);
                
                default:
                    break;
                }
        }
        return EulerAngles();
    }
    
    EulerAngles convertEulerAngleSequence(const EulerAngles& angles, EulerAngles::EulerSequence newseq ){
        if( angles.getSequence() == newseq) return angles;
        else if(angles.getSequence() == EulerAngles::EulerSequence::XYZ && newseq == EulerAngles::EulerSequence::ZXZ){
            return convertEulerAngleXYZ_ZXZ(angles.phi, angles.theta, angles.psi);
        }
        else if(angles.getSequence() == EulerAngles::EulerSequence::ZXZ && newseq == EulerAngles::EulerSequence::XYZ){
            return convertEulerAngleZXZ_XYZ(angles.phi, angles.theta, angles.psi);
        }
        else{
            Matrix3x3 dcm = Euler2DCM(angles);
            return DCM2Euler(dcm, newseq);
        }
    }

    // Euler to DCM
    Matrix3x3 euler2DCM_ZXZ(double phi, double the, double psi){
        double data[9];
        double cos_phi = cos(phi);
        double sin_phi = cos(phi);

        double cos_the = cos(the);
        double sin_the = sin(the);

        double cos_psi = cos(psi);
        double sin_psi = sin(psi);

        data[0] = cos_psi * cos_the;
        data[1] = cos_the * sin_psi;

        data[2] = -sin_the;
        data[3] = cos_psi  * sin_the* cos_the;
        data[4] = cos_psi * cos_the;
        data[5] = cos_psi * cos_the;
        data[6] = cos_psi * cos_the;
        data[7] = cos_psi * cos_the;
        data[8] = cos_psi * cos_the;

    }

    Matrix3x3 euler2DCM_XYX(double phi, double theta, double psi);
    Matrix3x3 euler2DCM_YZY(double phi, double theta, double psi);

    Matrix3x3 euler2DCM_ZYZ(double phi, double theta, double psi);
    Matrix3x3 euler2DCM_XZX(double phi, double theta, double psi);
    Matrix3x3 euler2DCM_YXY(double phi, double theta, double psi);

    Matrix3x3 euler2DCM_XYZ(double phi, double theta, double psi);
    Matrix3x3 euler2DCM_YZX(double phi, double theta, double psi);
    Matrix3x3 euler2DCM_ZXY(double phi, double theta, double psi);

    Matrix3x3 euler2DCM_XZY(double phi, double theta, double psi);
    Matrix3x3 euler2DCM_ZYX(double phi, double theta, double psi);
    Matrix3x3 euler2DCM_YXZ(double phi, double theta, double psi);

    // DCM to Euler
    EulerAngles DCM2euler_ZXZ(Matrix3x3& dcm);
    EulerAngles DCM2euler_XYX(Matrix3x3& dcm);
    EulerAngles DCM2euler_YZY(Matrix3x3& dcm);
    EulerAngles DCM2euler_ZYZ(Matrix3x3& dcm);
    EulerAngles DCM2euler_XZX(Matrix3x3& dcm);
    EulerAngles DCM2euler_YXY(Matrix3x3& dcm);
    EulerAngles DCM2euler_XYZ(Matrix3x3& dcm);
    EulerAngles DCM2euler_YZX(Matrix3x3& dcm);
    EulerAngles DCM2euler_ZXY(Matrix3x3& dcm);
    EulerAngles DCM2euler_XZY(Matrix3x3& dcm);
    EulerAngles DCM2euler_ZYX(Matrix3x3& dcm);
    EulerAngles DCM2euler_YXZ(Matrix3x3& dcm);


    // Euler Angle Sequence Conversions
    EulerAngles convertEulerAngleXYZ_ZXZ(double phi, double theta, double psi);
    EulerAngles convertEulerAngleZXZ_XYZ(double phi, double theta, double psi);

    // Euler Angle Kinematics
    EulerAngles integrateEulerAngles(const EulerAngles& angles, const EulerAngles& angleRates, double dt);
    EulerAngles eulerAngleKinematicRates(const EulerAngles& angles, const Vector3& bodyRates);

    Matrix3x3 eulerAngleRatesMatrix_ZXZ(double phi, double theta, double psi);
    Matrix3x3 eulerAngleRatesMatrix_XYX(double phi, double theta, double psi);
    Matrix3x3 eulerAngleRatesMatrix_YZY(double phi, double theta, double psi);
    Matrix3x3 eulerAngleRatesMatrix_ZYZ(double phi, double theta, double psi);
    Matrix3x3 eulerAngleRatesMatrix_XZX(double phi, double theta, double psi);
    Matrix3x3 eulerAngleRatesMatrix_YXY(double phi, double theta, double psi);
    Matrix3x3 eulerAngleRatesMatrix_XYZ(double phi, double theta, double psi);
    Matrix3x3 eulerAngleRatesMatrix_YZX(double phi, double theta, double psi);
    Matrix3x3 eulerAngleRatesMatrix_ZXY(double phi, double theta, double psi);
    Matrix3x3 eulerAngleRatesMatrix_XZY(double phi, double theta, double psi);
    Matrix3x3 eulerAngleRatesMatrix_ZYX(double phi, double theta, double psi);
    Matrix3x3 eulerAngleRatesMatrix_YXZ(double phi, double theta, double psi);
    
    // Euler Angle Interpolation
    EulerAngles interpolation(const EulerAngles& startAngles, const EulerAngles& endAngles, double t);


}