#ifndef DCM_H
#define DCM_H

#include <iostream>
#include <limits>
#include "matrix3.h"
#include "vector3.h"

namespace AML
{
    class DCM
    {
    public:
        static const Matrix3x3 rotationX(double theta);
        static const Matrix3x3 rotationY(double theta);
        static const Matrix3x3 rotationZ(double theta);
    };

    // DCM Functions
    bool isValidDCM(const Matrix3x3 &dcm, double tolarance = std::numeric_limits<double>::epsilon());

    void normalizeDCM(Matrix3x3 &dcm);

    // DCM Kinematic Functions
    Matrix3x3 integrateDCM(const Matrix3x3 &dcm, const Matrix3x3 &dcmRates, double dt);

    Matrix3x3 dcmKinimaticRates_BodyRates(const Matrix3x3 &dcm, const Vector3 &bodyRates);
    Matrix3x3 dcmKinimaticRates_WorldRates(const Matrix3x3 &dcm, const Vector3 &worldRates);

}

#endif // DCM_H