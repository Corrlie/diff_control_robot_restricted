// pbr_opGrupowa.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
// x - 0, y - 1, fi r - 2, fi l - 3
Matrix4f operatorAd(Matrix <float, 4, 1> mat_g) {
    Matrix2f R_gfir, R_gfil, R, G, Zeros_22, Ident_22; 
    R_gfir << cos(mat_g(2)), -sin(mat_g(2)),
        sin(mat_g(2)), cos(mat_g(2));
    R_gfil << cos(mat_g(3)), -sin(mat_g(3)),
        sin(mat_g(3)), cos(mat_g(3));
    R = R_gfir - R_gfil;
    G << mat_g(1), -mat_g(1),
        -mat_g(0), mat_g(0);
   // Zeros_22.setZero();
    // Ident_22.setIdentity();
    Matrix4f res;
    res << R(0, 0), R(0, 1), G(0, 0), G(0, 1),
        R(1, 0), R(1, 1), G(1, 0), G(1, 1),
        0, 0, 1, 0,
        0, 0, 0, 1;

      return res;
    // return R, G, Zeros_22, Ident_22;

}

Matrix<float, 4, 1> operacjaGrupowa(Matrix <float, 4, 1> mat_g, Matrix <float, 4, 1> mat_h) {
    
    Matrix4f T_gfir;
    Matrix4f T_gfil;

    T_gfir << cos(mat_g(2)), -sin(mat_g(2)), 0, 0,
        sin(mat_g(2)), cos(mat_g(2)), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    // mat_g(2) - g fi r
    // mat_g(3) - g fi l
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

    cout << "\n\n Operacja grupowa: \n" << endl;
    MatrixXf mata;
    mata = operacjaGrupowa(g, h);
    cout << mata;

    cout << "\n\n Operator AD: \n" << endl; 
    Matrix4f ada;
    ada = operatorAd(g);
    cout << ada;
}
