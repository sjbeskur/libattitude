#include <iostream>
#include <cmath>
#include <string>
#include "AML.h"

using namespace AML;
using namespace std;

void euler_example();

int main(int argc, char **argv)
{
    Matrix3x3 R = Matrix3x3::identity();
    std::cout << R << std::endl;
    for (unsigned int i = 0; i < 100; i++)
    {
        Matrix3x3 Rdot = dcmKinimaticRates_BodyRates(R, Vector3(1.0, 0.0, 0.0)); // x rotation at 1 rad / sec
        R = integrateDCM(R, Rdot, 0.01);                                         // 0.01 of a second
        std::cout << "R: " << R << std::endl;
    }

    std::cout << "running euler example" << std::endl;
    euler_example();

    return 0;
}
void euler_example(){
    EulerAngles angles(0.1, -0.3, -0.5);
    Matrix3x3 dcm = Euler2DCM(angles);

    std::cout << angles << std::endl;
    std::cout << dcm << std::endl;

    EulerAngles recovered = DCM2Euler(dcm);
    std::cout << recovered << std::endl;

}

void matrix_example()
{

    auto v0 = Vector3::xAxis();
    Vector3 v1{1.0, 2.0, 3.0};
    Vector3 v2{4.0, 5.0, 6.0};

    Matrix3x3 matrix{v0, v1, v2};
    cout << matrix << endl;
    cout << inverse(matrix) << endl;
    cout << Matrix3x3::identity() << endl;
    cout << matrix.data << endl;

    double data[3][3] = {{1.2, 3.2, 0.5}, {0.1, 1.0, 6.0}, {-4.1, 6.0, 8.0}};
    Matrix3x3 mat1(data);

    Vector3 v_1 = Vector3(1, 0, 0);
    Vector3 v_2 = mat1 * v_1;

    cout << v_1 << endl;
    cout << mat1 << endl;
    cout << v_2 << endl;

    Vector3 vmiddle = Vector3(0, 1, 0);
    Vector3 vm = mat1 * vmiddle;
    cout << vm << endl; // middle values of the matrix

    Vector3 vlast = Vector3(0, 0, 1);
    Vector3 vl = mat1 * vlast;
    cout << vl << endl; // middle values of the matrix

    //    cout << inverse(random_matrix) << endl;

    //    cout << random_matrix + matrix << endl;
}

int vector_example(int argc, char **argv)
{

    auto v0 = Vector3::xAxis();
    Vector3 v1{1.0, 2.0, 3.0};
    Vector3 v2{4.0, 5.0, 6.0};
    AML::Vector3 result = v1 + v2;

    Vector3 v3 = result * v1;
    cout << v1 << endl;
    cout << v2 << endl;
    cout << v3 << endl;

    return 0;
}