#include "dcm.h"
#include "vector3.h"
#include "matrix3.h"
#include <cmath>

namespace AML
{

    const Matrix3x3 DCM::rotationX(double theta)
    {
        double data[3][3] = {
            {1.0, 0.0, 0.0},
            {0.0, cos(theta), sin(theta)},
            {0.0, -sin(theta), cos(theta)}};
        return Matrix3x3(data);
    }

    const Matrix3x3 DCM::rotationY(double theta)
    {
        double data[3][3] = {
            {cos(theta), 0.0, -sin(theta)},
            {0.0, 1.0, 0.0},
            {sin(theta), 0.0, cos(theta)}

        };
        return Matrix3x3(data);
    }

    const Matrix3x3 DCM::rotationZ(double theta)
    {
        double data[3][3] = {
            {cos(theta), sin(theta), 0.0},
            {-sin(theta), cos(theta), 0.0},
            {0.0, 0.0, 1.0},

        };
        return Matrix3x3(data);
    }

    // DCM Functions
    void normalizeDCM(Matrix3x3 &dcm)
    {
        Vector3 x = Vector3(dcm.m11, dcm.m12, dcm.m13); // top x row
        Vector3 y = Vector3(dcm.m21, dcm.m22, dcm.m23); // y row
        double error = dot(x, y);

        Vector3 x_orth = x - 0.5 * error * y;
        Vector3 y_orth = y - 0.5 * error * x;
        Vector3 z_orth = cross(x_orth, y_orth);

        Vector3 x_norm = 0.5 * (3.0 - dot(x_orth, x_orth)) * x_orth;
        Vector3 y_norm = 0.5 * (3.0 - dot(y_orth, y_orth)) * y_orth;
        Vector3 z_norm = 0.5 * (3.0 - dot(z_orth, z_orth)) * z_orth;
        dcm = transpose(Matrix3x3(x_norm, y_norm, z_norm));
    }

    bool isValidDCM(const Matrix3x3 &dcm, double tolarance)
    {
        double tol_limit = tolarance * 2.0;
        // test det
        bool det_test = fabs(determinant(dcm) - 1.0) < tol_limit;
        bool neg_det_test = determinant(dcm) > 0.0;

        // test orthog
        Matrix3x3 test_identity = (dcm * transpose(dcm) - Matrix3x3::identity());
        bool m11_id_test = (fabs(test_identity.m11) < tol_limit);
        bool m12_id_test = (fabs(test_identity.m12) < tol_limit);
        bool m13_id_test = (fabs(test_identity.m13) < tol_limit);
        bool m21_id_test = (fabs(test_identity.m21) < tol_limit);
        bool m22_id_test = (fabs(test_identity.m22) < tol_limit);
        bool m23_id_test = (fabs(test_identity.m23) < tol_limit);
        bool m31_id_test = (fabs(test_identity.m31) < tol_limit);
        bool m32_id_test = (fabs(test_identity.m32) < tol_limit);
        bool m33_id_test = (fabs(test_identity.m33) < tol_limit);

        // cpplint disable
        bool identity_check = m11_id_test && m12_id_test && m13_id_test && m21_id_test && m22_id_test && m23_id_test && m31_id_test && m32_id_test && m33_id_test;
        // cpplint enable

        return det_test && neg_det_test && identity_check;
    }

    // DCM Kinematic Functions
    Matrix3x3 integrateDCM(const Matrix3x3 &dcm, const Matrix3x3 &dcmRates, double dt)
    {
        Matrix3x3 dcmNew = dcm + dcmRates * dt;
        normalizeDCM(dcmNew);
        return dcmNew;
    }

    Matrix3x3 dcmKinimaticRates_BodyRates(const Matrix3x3 &dcm, const Vector3 &bodyRates)
    {
        const double p = bodyRates.x;
        const double q = bodyRates.y;
        const double r = bodyRates.z;
        // cpplint disable
        const double skew_data[3][3] = {{0.0, -r, q}, {r, 0.0, -p}, {-q, p, 0.0}};
        // cpplint enable
        const Matrix3x3 skewMatrix = Matrix3x3(skew_data);
        return -skewMatrix * dcm;
    }

    Matrix3x3 dcmKinimaticRates_WorldRates(const Matrix3x3 &dcm, const Vector3 &worldRates)
    {
        const double p = worldRates.x;
        const double q = worldRates.y;
        const double r = worldRates.z;
        // cpplint disable
        const double skew_data[9] = {
            0.0, -r, q,
            r, 0.0, -p,
            -q, p, 0.0};
        // cpplint enable
        const Matrix3x3 skewMatrix = Matrix3x3(skew_data);
        return dcm * skewMatrix;
    }

};