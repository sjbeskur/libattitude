
#include "euler.h"
#include "vector3.h"
#include "matrix3.h"
#include "dcm.h"
#include <cmath>

namespace AML{

    // Constructors
    EulerAngles::EulerAngles()
        :phi(0.0), theta(0.0), psi(0.0), seq_(EulerSequence::XYZ){ }

    EulerAngles::EulerAngles(double phi_, double theta_, double psi_, EulerSequence seq)
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

    
    EulerAngles DCM2Euler(const Matrix3x3& dcm, const EulerAngles::EulerSequence seq){
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
        data[3] = cos_psi * sin_the * sin_phi - sin_psi * cos_phi;
        data[4] = sin_psi * sin_the * sin_phi + cos_psi * cos_phi;
        data[5] = sin_phi * cos_the;
        data[6] = cos_psi * sin_the * cos_phi + sin_psi * sin_phi;
        data[7] = sin_psi * sin_the * cos_phi - cos_psi * sin_phi;
        data[8] = cos_the * cos_phi;
        return Matrix3x3(data);

    }

    Matrix3x3 euler2DCM_XYX(double phi, double theta, double psi){ return (DCM::rotationX(phi) * DCM::rotationY(theta) * DCM::rotationX(psi)); }
    Matrix3x3 euler2DCM_YZY(double phi, double theta, double psi){ return (DCM::rotationY(phi) * DCM::rotationZ(theta) * DCM::rotationY(psi)); }
    Matrix3x3 euler2DCM_ZYZ(double phi, double theta, double psi){ return (DCM::rotationZ(phi) * DCM::rotationY(theta) * DCM::rotationZ(psi)); }
    Matrix3x3 euler2DCM_XZX(double phi, double theta, double psi){ return (DCM::rotationX(phi) * DCM::rotationZ(theta) * DCM::rotationX(psi)); }
    Matrix3x3 euler2DCM_YXY(double phi, double theta, double psi){ return (DCM::rotationY(phi) * DCM::rotationX(theta) * DCM::rotationY(psi)); }
    Matrix3x3 euler2DCM_XYZ(double phi, double theta, double psi){ return (DCM::rotationX(phi) * DCM::rotationY(theta) * DCM::rotationZ(psi)); }
    Matrix3x3 euler2DCM_YZX(double phi, double theta, double psi){ return (DCM::rotationY(phi) * DCM::rotationZ(theta) * DCM::rotationX(psi)); }
    Matrix3x3 euler2DCM_ZXY(double phi, double theta, double psi){ return (DCM::rotationZ(phi) * DCM::rotationX(theta) * DCM::rotationY(psi)); }
    Matrix3x3 euler2DCM_XZY(double phi, double theta, double psi){ return (DCM::rotationX(phi) * DCM::rotationZ(theta) * DCM::rotationY(psi)); }
    Matrix3x3 euler2DCM_ZYX(double phi, double theta, double psi){ return (DCM::rotationZ(phi) * DCM::rotationY(theta) * DCM::rotationX(psi)); }
    Matrix3x3 euler2DCM_YXZ(double phi, double theta, double psi){ return (DCM::rotationY(phi) * DCM::rotationX(theta) * DCM::rotationZ(psi)); }

    // DCM to Euler
    EulerAngles DCM2euler_XYZ(const Matrix3x3& dcm){
        double phi = atan2(dcm.m23, dcm.m33);
        double the = -asin(dcm.m13);
        double psi = atan2(dcm.m12, dcm.m11);
        return EulerAngles(phi, the, psi, EulerAngles::EulerSequence::XYZ);
    }

    EulerAngles DCM2euler_ZXZ(const Matrix3x3& dcm){
        double phi   = atan2(dcm.m13, dcm.m23);
        double theta = acos(dcm.m33);
        double psi   = atan2(dcm.m31, -dcm.m32);   
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::ZXZ);
    }

    EulerAngles DCM2euler_XYX(const Matrix3x3& dcm){
        double phi   = atan2(dcm.m21, dcm.m31);
        double theta = acos(dcm.m11);
        double psi   = atan2(dcm.m12, -dcm.m13);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::XYX);
    }

    EulerAngles DCM2euler_YZY(const Matrix3x3& dcm){
        double phi   = atan2(dcm.m32, dcm.m12);
        double theta = acos(dcm.m22);
        double psi   = atan2(dcm.m23, -dcm.m21);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::YZY);
    }

    EulerAngles DCM2euler_ZYZ(const Matrix3x3& dcm){
        double phi   = atan2(dcm.m23, -dcm.m13);
        double theta = acos(dcm.m33);
        double psi   = atan2(dcm.m32, dcm.m31);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::ZYZ);
    }

    EulerAngles DCM2euler_XZX(const Matrix3x3& dcm){
        double phi   = atan2(dcm.m31, -dcm.m21);
        double theta = acos(dcm.m11);
        double psi   = atan2(dcm.m13, dcm.m12);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::XZX);
    }

    EulerAngles DCM2euler_YXY(const Matrix3x3& dcm){
        double phi   = atan2(dcm.m12, -dcm.m32);
        double theta = acos(dcm.m22);
        double psi   = atan2(dcm.m21, dcm.m23);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::YXY);
    }

    EulerAngles DCM2euler_YZX(const Matrix3x3& dcm){
        double phi   = atan2(dcm.m31, dcm.m11);
        double theta = -asin(dcm.m21);
        double psi   = atan2(dcm.m23, dcm.m22);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::YZX);
    }

    EulerAngles DCM2euler_ZXY(const Matrix3x3& dcm){
        double phi   = atan2(dcm.m12, dcm.m22);
        double theta = -asin(dcm.m32);
        double psi   = atan2(dcm.m31, dcm.m33);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::ZXY);
    }

    EulerAngles DCM2euler_XZY(const Matrix3x3& dcm){
        double phi   = atan2(-dcm.m32, dcm.m22);
        double theta = asin(dcm.m12);
        double psi   = atan2(-dcm.m13, dcm.m11);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::XZY);
    }
    
    EulerAngles DCM2euler_ZYX(const Matrix3x3& dcm){
        double phi = atan2(-dcm.m21, dcm.m11);
        double the = asin(dcm.m31);
        double psi = atan2(-dcm.m32, dcm.m33);
        return EulerAngles(phi, the, psi, EulerAngles::EulerSequence::ZYX);

    }
    
    EulerAngles DCM2euler_YXZ(const Matrix3x3& dcm){
        double phi = atan2(-dcm.m13, dcm.m33);
        double the = asin(dcm.m23);
        double psi = atan2(-dcm.m21, dcm.m22);
        return EulerAngles(phi, the, psi, EulerAngles::EulerSequence::YXZ);
    }


    // Euler Angle Sequence Conversions
    EulerAngles convertEulerAngleXYZ_ZXZ(double phi, double theta, double psi){
        const double cosPhi   = cos(phi);
        const double sinPhi   = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cosPsi   = cos(psi);
        const double sinPsi   = sin(psi);
        const double phiZXZ   = atan2(-sinTheta, sinPhi*cosTheta);
        const double thetaZXZ = acos(cosPhi*cosTheta);
        const double psiZXZ   = atan2(cosPhi*sinTheta*cosPsi + sinPhi*sinPsi, -cosPhi*sinTheta*sinPsi + sinPhi*cosPsi);
        return EulerAngles(phiZXZ, thetaZXZ, psiZXZ, EulerAngles::EulerSequence::ZXZ);
    }

    EulerAngles convertEulerAngleZXZ_XYZ(double phi, double theta, double psi){
        const double cosPhi   = cos(phi);
        const double sinPhi   = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cosPsi   = cos(psi);
        const double sinPsi   = sin(psi);
        const double phiXYZ   = atan2(cosPhi*sinTheta, cosTheta);
        const double thetaXYZ = -asin(sinPhi*sinTheta);
        const double psiXYZ   = atan2(cosPhi*sinPsi + sinPhi*cosTheta*cosPsi, cosPhi*cosPsi - sinPhi*cosTheta*sinPsi);
        return EulerAngles(phiXYZ, thetaXYZ, psiXYZ, EulerAngles::EulerSequence::XYZ);
    }

    // Euler Angle Kinematics
    EulerAngles integrateEulerAngles(const EulerAngles& angles, const EulerAngles& angleRates, double dt){
        EulerAngles::EulerSequence seq = angles.getSequence();
        if (seq == angleRates.getSequence())
        {
            double phiNew   = angles.phi + angleRates.phi * dt;
            double thetaNew = angles.theta + angleRates.theta * dt;
            double psiNew   = angles.psi + angleRates.psi * dt;
            return EulerAngles(phiNew, thetaNew, psiNew, seq);
        }
        return angles;
    }

    EulerAngles eulerAngleKinematicRates(const EulerAngles& angles, const Vector3& bodyRates){
        Vector3 eulerRates;
        EulerAngles::EulerSequence seq = angles.getSequence();
        switch(seq)
        {
            case EulerAngles::EulerSequence::XYZ:
                eulerRates = eulerAngleRatesMatrix_XYZ(angles.phi, angles.theta, angles.psi) * bodyRates;
                break;  
            case EulerAngles::EulerSequence::ZXZ:
                eulerRates = eulerAngleRatesMatrix_ZXZ(angles.phi, angles.theta, angles.psi) * bodyRates;
                break; 
            case EulerAngles::EulerSequence::YZY:
                eulerRates = eulerAngleRatesMatrix_YZY(angles.phi, angles.theta, angles.psi) * bodyRates;
                break; 
            case EulerAngles::EulerSequence::XYX:
                eulerRates = eulerAngleRatesMatrix_XYX(angles.phi, angles.theta, angles.psi) * bodyRates;
                break; 
            case EulerAngles::EulerSequence::ZYZ:
                eulerRates = eulerAngleRatesMatrix_ZYZ(angles.phi, angles.theta, angles.psi) * bodyRates;
                break; 
            case EulerAngles::EulerSequence::XZX:
                eulerRates = eulerAngleRatesMatrix_XZX(angles.phi, angles.theta, angles.psi) * bodyRates;
                break; 
            case EulerAngles::EulerSequence::YXY:
                eulerRates = eulerAngleRatesMatrix_YXY(angles.phi, angles.theta, angles.psi) * bodyRates;
                break; 
            case EulerAngles::EulerSequence::YZX:
                eulerRates = eulerAngleRatesMatrix_YZX(angles.phi, angles.theta, angles.psi) * bodyRates;
                break; 
            case EulerAngles::EulerSequence::ZXY:
                eulerRates = eulerAngleRatesMatrix_ZXY(angles.phi, angles.theta, angles.psi) * bodyRates;
                break; 
            case EulerAngles::EulerSequence::XZY:
                eulerRates = eulerAngleRatesMatrix_XZY(angles.phi, angles.theta, angles.psi) * bodyRates;
                break; 
            case EulerAngles::EulerSequence::ZYX:
                eulerRates = eulerAngleRatesMatrix_ZYX(angles.phi, angles.theta, angles.psi) * bodyRates;
                break; 
            case EulerAngles::EulerSequence::YXZ:
                eulerRates = eulerAngleRatesMatrix_YXZ(angles.phi, angles.theta, angles.psi) * bodyRates;
                break; 
        }
        return EulerAngles(eulerRates.x, eulerRates.y, eulerRates.z, seq);
    }

    Matrix3x3 eulerAngleRatesMatrix_XYZ(double phi, double theta, double psi){
        const double cosPhi   = cos(phi);
        const double sinPhi   = sin(phi);
        const double cosTheta = cos(theta);
        const double tanTheta = tan(theta);
        const double secTheta = 1.0 / cosTheta;
        
        double data[3][3] = {{1.0, sinPhi*tanTheta, cosPhi*tanTheta},
                             {0.0, cosPhi, -sinPhi},
                             {0.0, sinPhi*secTheta, cosPhi*secTheta}};
                             
        return Matrix3x3(data);
    }

    Matrix3x3 eulerAngleRatesMatrix_ZXZ(double phi, double theta, double psi){
        const double cosPhi   = cos(phi);
        const double sinPhi   = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cscTheta = 1.0 / sinTheta;
        
        double data[3][3] = {{-sinPhi*cosTheta*cscTheta, -cosPhi*cosTheta*cscTheta, sinTheta*cscTheta},
                             {cosPhi*sinTheta*cscTheta, -sinPhi*sinTheta*cscTheta, 0.0},
                             {sinPhi*cscTheta, cosPhi*cscTheta, 0.0}};
                             
        return Matrix3x3(data);
    }

    Matrix3x3 eulerAngleRatesMatrix_YZY(double phi, double theta, double psi){
        const double cosPhi   = cos(phi);
        const double sinPhi   = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cscTheta = 1.0 / sinTheta;
        
        double data[3][3] = {{-cosPhi*cosTheta*cscTheta, sinTheta*cscTheta, -sinPhi*cosTheta*cscTheta},
                             {-sinPhi*sinTheta*cscTheta, 0.0, cosPhi*sinTheta*cscTheta},
                             {cosPhi*cscTheta, 0.0, sinPhi*cscTheta}};
                             
        return Matrix3x3(data);        
    }

    Matrix3x3 eulerAngleRatesMatrix_XYX(double phi, double theta, double psi){
        const double cosPhi   = cos(phi);
        const double sinPhi   = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cscTheta = 1.0 / sinTheta;
        
        double data[3][3] = {{sinTheta*cscTheta, -sinPhi*cosTheta*cscTheta, -cosPhi*cosTheta*cscTheta},
                             {0.0, cosPhi*sinTheta*cscTheta, -sinPhi*sinTheta*cscTheta},
                             {0.0, sinPhi*cscTheta, cosPhi*cscTheta}};
                             
        return Matrix3x3(data);
    }


    Matrix3x3 eulerAngleRatesMatrix_ZYZ(double phi, double theta, double psi){
        const double cosPhi   = cos(phi);
        const double sinPhi   = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cscTheta = 1.0 / sinTheta;
        
        double data[3][3] = {{cosPhi*cosTheta*cscTheta, -sinPhi*cosTheta*cscTheta, sinTheta},
                             {sinPhi*sinTheta*cscTheta, cosPhi*sinTheta*cscTheta, 0.0},
                             {-cosPhi*cscTheta, sinPhi*cscTheta, 0.0}};
                             
        return Matrix3x3(data);        
    }

    Matrix3x3 eulerAngleRatesMatrix_XZX(double phi, double theta, double psi){
        const double cosPhi   = cos(phi);
        const double sinPhi   = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cscTheta = 1.0 / sinTheta;
        
        double data[3][3] = {{sinTheta*cscTheta, cosPhi*cosTheta*cscTheta, -sinPhi*cosTheta*cscTheta},
                             {0.0, sinPhi*sinTheta*cscTheta, cosPhi*sinTheta*cscTheta},
                             {0.0, -cosPhi*cscTheta, sinPhi*cscTheta}};
                             
        return Matrix3x3(data);        
    }

    Matrix3x3 eulerAngleRatesMatrix_YXY(double phi, double theta, double psi){
        const double cosPhi   = cos(phi);
        const double sinPhi   = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cscTheta = 1.0 / sinTheta;
        
        double data[3][3] = {{-sinPhi*cosTheta*cscTheta, sinTheta*cscTheta, cosPhi*cosTheta*cscTheta},
                             {sinTheta*cosPhi*cscTheta, 0.0, sinTheta*sinPhi*cscTheta},
                             {sinPhi*cscTheta, 0.0, -cosPhi*cscTheta}};
                             
        return Matrix3x3(data);
    }
    
    Matrix3x3 eulerAngleRatesMatrix_YZX(double phi, double theta, double psi){
        const double cosPhi   = cos(phi);
        const double sinPhi   = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double secTheta = 1.0 / cosTheta;
        
        double data[3][3] = {{cosPhi*sinTheta*secTheta, cosTheta*secTheta, sinPhi*sinTheta*secTheta},
                             {-sinPhi*cosTheta*secTheta, 0.0, cosPhi*cosTheta*secTheta},
                             {cosPhi*secTheta, 0.0, sinPhi*secTheta}};
                             
        return Matrix3x3(data);        
    }
 
    Matrix3x3 eulerAngleRatesMatrix_ZXY(double phi, double theta, double psi){
        const double cosPhi   = cos(phi);
        const double sinPhi   = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double secTheta = 1.0 / cosTheta;
        
        double data[3][3] = {{sinPhi*sinTheta*secTheta, cosPhi*sinTheta*secTheta, cosTheta*secTheta},
                             {cosTheta*cosPhi*secTheta, -sinPhi*cosTheta*secTheta, 0.0},
                             {sinPhi*secTheta, cosPhi*secTheta, 0.0}};
                             
        return Matrix3x3(data);        
    }
 
    Matrix3x3 eulerAngleRatesMatrix_XZY(double phi, double theta, double psi){
        const double cosPhi   = cos(phi);
        const double sinPhi   = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double secTheta = 1.0 / cosTheta;
        
        double data[3][3] = {{cosTheta*secTheta, -cosPhi*sinTheta*secTheta, sinPhi*sinTheta*secTheta},
                             {0.0, sinPhi*cosTheta*secTheta, cosPhi*cosTheta*secTheta},
                             {0.0, cosPhi*secTheta, -sinPhi*secTheta}};
                             
        return Matrix3x3(data);
    }
 
    Matrix3x3 eulerAngleRatesMatrix_ZYX(double phi, double theta, double psi){
        const double cosPhi   = cos(phi);
        const double sinPhi   = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double secTheta = 1.0 / cosTheta;
        
        double data[3][3] = {{-cosPhi*sinTheta*secTheta, sinPhi*sinTheta*secTheta, cosTheta*secTheta},
                             {sinPhi*cosTheta*secTheta, cosPhi*cosTheta*secTheta, 0.0},
                             {cosPhi*secTheta, -sinPhi*secTheta, 0.0}};
                             
        return Matrix3x3(data);
    }
 
    Matrix3x3 eulerAngleRatesMatrix_YXZ(double phi, double theta, double psi){
        const double cosPhi   = cos(phi);
        const double sinPhi   = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double secTheta = 1.0 / cosTheta;
        
        double data[3][3] = {{sinPhi*sinTheta*secTheta, cosTheta*secTheta, -cosPhi*sinTheta*secTheta},
                             {cosPhi*cosTheta*secTheta, 0.0, sinPhi*cosTheta*secTheta},
                             {-sinPhi*secTheta, 0.0, cosPhi*secTheta}};
                             
        return Matrix3x3(data);
    };
    
    // Euler Angle Interpolation
    EulerAngles linearInterpolate(const EulerAngles& startAngles, const EulerAngles& endAngles, double t)
    {
        if (startAngles.getSequence() == endAngles.getSequence())
        {
            if (t < 0.0){return startAngles;}
            if (t > 1.0){return endAngles;}
            double phiNew = (1-t) * startAngles.phi + t * endAngles.phi;
            double thetaNew = (1-t) * startAngles.theta + t * endAngles.theta;
            double psiNew = (1-t) * startAngles.psi + t * endAngles.psi;
            return EulerAngles(phiNew, thetaNew, psiNew, startAngles.getSequence());
        }
        return EulerAngles();
    }

    EulerAngles smoothInterpolate(const EulerAngles& startAngles, const EulerAngles& endAngles, double t)
    {
        if (startAngles.getSequence() == endAngles.getSequence())
        {
            if (t < 0.0){return startAngles;}
            if (t > 1.0){return endAngles;}

            double t2 = t*t;
            double t3 = t2*t;
            double t4 = t3*t;
            double t5 = t4*t;

            double deltaPhi = endAngles.phi - startAngles.phi;
            double deltaTheta = endAngles.theta - startAngles.theta;
            double deltaPsi = endAngles.psi - startAngles.psi;

            double phiNew   = 6*deltaPhi*t5 + -15*deltaPhi*t4 + 10*deltaPhi*t3 + startAngles.phi;
            double thetaNew = 6*deltaTheta*t5 + -15*deltaTheta*t4 + 10*deltaTheta*t3 + startAngles.theta;
            double psiNew   = 6*deltaPsi*t5 + -15*deltaPsi*t4 + 10*deltaPsi*t3 + startAngles.psi;

            return EulerAngles(phiNew, thetaNew, psiNew, startAngles.getSequence());
        }
        return EulerAngles();
    }
};