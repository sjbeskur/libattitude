#include "catch/catch.hpp"
#include "AML.h"

using namespace AML;

TEST_CASE("DCM Rotation X", "[DCM]")
{
    // Case 1
    Matrix3x3 m = DCM::rotationX(0.5);
    CHECK(m.m11 == 1.0);
    CHECK(m.m12 == 0.0);
    CHECK(m.m13 == 0.0);
    CHECK(m.m21 == 0.0);
    CHECK(m.m22 == cos(0.5));
    CHECK(m.m23 == sin(0.5));
    CHECK(m.m31 == 0.0);
    CHECK(m.m32 == -sin(0.5));
    CHECK(m.m33 == cos(0.5));
}

TEST_CASE("DCM Rotation Y", "[DCM]")
{
    // Case 1
    Matrix3x3 m = DCM::rotationY(0.5);
    CHECK(m.m11 == cos(0.5));
    CHECK(m.m12 == 0.0);
    CHECK(m.m13 == -sin(0.5));
    CHECK(m.m21 == 0.0);
    CHECK(m.m22 == 1.0);
    CHECK(m.m23 == 0.0);
    CHECK(m.m31 == sin(0.5));
    CHECK(m.m32 == 0.0);
    CHECK(m.m33 == cos(0.5));
}

TEST_CASE("DCM Rotation Z", "[DCM]")
{
    // Case 1
    Matrix3x3 m = DCM::rotationZ(0.5);
    CHECK(m.m11 == cos(0.5));
    CHECK(m.m12 == sin(0.5));
    CHECK(m.m13 == 0.0);
    CHECK(m.m21 == -sin(0.5));
    CHECK(m.m22 == cos(0.5));
    CHECK(m.m23 == 0.0);
    CHECK(m.m31 == 0.0);
    CHECK(m.m32 == 0.0);
    CHECK(m.m33 == 1.0);
}

TEST_CASE("DCM Valid", "[Matrix3x3]")
{
    Matrix3x3 m1 = Matrix3x3::identity();
    Matrix3x3 m2 = m1 + 0.1;
    Matrix3x3 m3 = -Matrix3x3::identity();
    CHECK(isValidDCM(m1));
    CHECK(!isValidDCM(m2));
    CHECK(!isValidDCM(m3));
}
