#include <iostream>
#include <cmath>
#include <string>
#include "AML.h"

using namespace AML;
using namespace std;

int main(int argc, char **argv){
    std::cout << "Quaternion Examples:" << std::endl;
    Quaternion quat(0,0,0,0);

    double deg2Rad = M_PI / 180.0;
    EulerAngles angles1( 10 * deg2Rad, -20*deg2Rad, 15*deg2Rad);
    EulerAngles angles2( 40 * deg2Rad, -60*deg2Rad, 135*deg2Rad);

    Quaternion q1 = eulerAngles2Quat(angles1);
    Quaternion q2 = eulerAngles2Quat(angles2);

    std::cout << q1 << std::endl;
    std::cout << q2 << std::endl;

    Quaternion q_interp = slerpInterpolate(q1, q2, 0.5);
    std::cout << q_interp << std::endl;

    EulerAngles ang_interp = linearInterpolate(angles1, angles2, 0.5);

    std::cout << angles1 << std::endl;
    std::cout << angles2 << std::endl;
    std::cout << ang_interp << std::endl;

    std::cout << quat2EulerAngles(q_interp) << std::endl;
    return 0;

}
