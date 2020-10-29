// pbr_opGrupowa.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;

Matrix<float, 4, 1> operacjaGrupowa(Matrix <float, 4, 1> mat_g, Matrix <float, 4, 1> mat_h) {
    
    Matrix4f T_gfir;
    Matrix4f T_gfil;

    T_gfir << cos(mat_g(2)), -sin(mat_g(2)), 0, 0,
        sin(mat_g(2)), cos(mat_g(2)), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    // mat_g(i) - odenisienie do i-tego elementu macierzy mat_g
    T_gfil << cos(mat_g(3)), -sin(mat_g(3)), 0, 0,
        sin(mat_g(3)), cos(mat_g(3)), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    return mat_g + (T_gfir - T_gfil) * mat_h;
}

int main()
{
    Matrix <float, 4, 1> g;
    g << 1, 1, 3, 2;

    Matrix <float, 4, 1> h;
    h << 10, 10, 20, 8;

    MatrixXf mata;
    mata = operacjaGrupowa(g, h);
    cout << mata;
}
