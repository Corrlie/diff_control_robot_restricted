// pbr_opGrupowa.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <Eigen/Dense>
using namespace std;
using namespace Eigen;
// gx - 0, gy - 1, gfi r - 2, gfi l - 3 :: zmienna - indeks w macierzy g
/*
const int r = 0.026; // wheels radius
const int b = 0.066; // distance between wheels
const float fi_R0 = 0; // init revol right wheel
const float fi_L0 = 0; // init revol left wheel
const float theta0 = -(r / b)*(fi_R0 - fi_L0);
*/
Matrix4f operatorAd(Vector4f mat_g) {
    Matrix2f R, G, Zeros_22, Ident_22; 
    float fi = mat_g(2) - mat_g(3);
    R << cos(fi), -sin(fi),
        sin(fi), cos(fi);

    G << mat_g(1), -mat_g(1),
        -mat_g(0), mat_g(0);

    Matrix4f res;
    res << R(0, 0), R(0, 1), G(0, 0), G(0, 1),
        R(1, 0), R(1, 1), G(1, 0), G(1, 1),
        0, 0, 1, 0,
        0, 0, 0, 1;

      return res;
}

Vector4f operacjaGrupowa(Vector4f mat_g, Vector4f mat_h) {
    
    Matrix4f T;
    float fi = mat_g(2) - mat_g(3);
    T << cos(fi), -sin(fi), 0, 0,
        sin(fi), cos(fi), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    // mat_g(2) - g fi r
    // mat_g(3) - g fi l
    // mat_g(i) - odenisienie do i-tego elementu macierzy mat_g
    return mat_g + T * mat_h;
}


Vector4f inv_g(Vector4f mat_g) {
    Matrix4f T_gfir;
    Matrix4f T_gfil;

    Matrix4f T;
    float fi = mat_g(2) - mat_g(3);
    T << cos(fi), -sin(fi), 0, 0,
        sin(fi), cos(fi), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    MatrixXf T_trans = T.transpose();
    // cout << "\n\nMacierz T:\n\n" << T << endl;
    // cout << "\n\nMacierz T trans:\n\n" << T_trans << endl;
    Vector4f  res = -T_trans * mat_g;
    // cout << "\n\g_odwr:\n\n" << res;
    return res;

}

int main()
{
    Vector4f g;
    g << 1, 2, 3, 4;

    Vector4f h;
    h << 10, 20, 30, 40;

    cout << "\n\n Operacja grupowa: \n" << endl;
    Vector4f mat_opgrup;
    mat_opgrup = operacjaGrupowa(g, h);
    cout << mat_opgrup;

    cout << "\n\n Operator AD: \n" << endl; 
    Matrix4f mat_opAd;
    mat_opAd = operatorAd(g);
    cout << mat_opAd;

    /*
    MatrixXf inv_g_auto;
    inv_g_auto = g.inverse(); // nie dziala xd
    cout << "\n Maciez odwrotna do g:\n" << inv_g_auto;
    */

    cout << "\nMacierz g: \n" << g;
    Vector4f g_odwr= inv_g(g);
    cout << "\n Macierz odwrotna do g: \n" << g_odwr;

    cout << "\nOperacja grupowa g i g^-1:\n" << operacjaGrupowa(g, g_odwr);

    //cout << "\nMnozenie g i g^-1: \n" << g.dot(g_odwr);

}
