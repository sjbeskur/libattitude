#ifndef EULER_H
#define EULER_H

#include <iostream>
#include <limits>

namespace AML{
    class Matrix3x3;
    class Vector3;
    class EulerAngles{

        public:
            // Data
            enum class EulerSequence{
                ZXZ,
                XYX,
                YZY,
                ZYZ,
                XZX,
                YXY,
                XYZ,
                YZX,
                ZXY,
                XZY,
                ZYX,
                YXZ
            };

            double phi;
            double theta; 
            double psi;

            // Constructors
            EulerAngles();
            EulerAngles(double phi, double theta, double psi, EulerSequence seq = EulerSequence::XYZ);

            // Data Access
            EulerSequence getSequence() const { return seq_; }

        private:
            //Euler angle Sequences
            EulerSequence seq_;
    };

    // Stream Functions
    std::ostream& operator<<(std::ostream& os, const EulerAngles& obj);

    // Euler angle conversions
    Matrix3x3 Euler2DCM(const EulerAngles& angles);
    EulerAngles DCM2Euler(const Matrix3x3& dcm, const EulerAngles::EulerSequence seq = EulerAngles::EulerSequence::XYZ);
    EulerAngles convertEulerAngleSequence(const EulerAngles& angles, EulerAngles::EulerSequence newseq );

    // Euler to DCM
    Matrix3x3 euler2DCM_ZXZ(double phi, double theta, double psi);
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
    EulerAngles DCM2euler_ZXZ(const Matrix3x3& dcm);
    EulerAngles DCM2euler_XYX(const Matrix3x3& dcm);
    EulerAngles DCM2euler_YZY(const Matrix3x3& dcm);
    EulerAngles DCM2euler_ZYZ(const Matrix3x3& dcm);
    EulerAngles DCM2euler_XZX(const Matrix3x3& dcm);
    EulerAngles DCM2euler_YXY(const Matrix3x3& dcm);
    EulerAngles DCM2euler_XYZ(const Matrix3x3& dcm);
    EulerAngles DCM2euler_YZX(const Matrix3x3& dcm);
    EulerAngles DCM2euler_ZXY(const Matrix3x3& dcm);
    EulerAngles DCM2euler_XZY(const Matrix3x3& dcm);
    EulerAngles DCM2euler_ZYX(const Matrix3x3& dcm);
    EulerAngles DCM2euler_YXZ(const Matrix3x3& dcm);


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
    EulerAngles smoothInterpolate(const EulerAngles& startAngles, const EulerAngles& endAngles, double t);

};

#endif