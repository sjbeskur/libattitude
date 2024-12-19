#include "catch/catch.hpp"
#include "AML.h"

using namespace AML;

TEST_CASE("Constructors", "[Quat]")
{
    // Case 1
    Quaternion q(0.0, 1.0, 2.0, 3.0);
    CHECK(q.q0 == 0.0);
    CHECK(q.q1 == 1.0);
    CHECK(q.q2 == 2.0);
    CHECK(q.q3 == 3.0);

}