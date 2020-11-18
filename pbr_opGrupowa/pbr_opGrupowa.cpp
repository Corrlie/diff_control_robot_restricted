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
    Matrix4f res;
    float fi = mat_g(2) - mat_g(3);

    R << cos(fi), -sin(fi),
        sin(fi), cos(fi);

    G << mat_g(1), -mat_g(1),
        -mat_g(0), mat_g(0);

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
    Matrix4f T, T_trans;
    Vector4f res;
    float fi = mat_g(2) - mat_g(3);

    T << cos(fi), -sin(fi), 0, 0,
        sin(fi), cos(fi), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    T_trans = T.transpose();
    res = -T_trans * mat_g;

    return res;
}

Matrix4f X(float e) {
    Vector4f X1, X2, X3, X4;
    Matrix4f res;

    X1 << 0.5 * cos(e), 0.5 * sin(e), 1, 0;
    X2 << 0.5 * cos(e), 0.5 * sin(e), 0, 1;
    X3 << -sin(e), cos(e), 0, 0;
    X4 << cos(e), sin(e), 0, 0;
    
    res.col(0) = X1;
    res.col(1) = X2;
    res.col(2) = X3;
    res.col(3) = X4;

    return res;
}

Matrix4f opAdX(Vector4f mat_g) {

    Matrix4f opAd, mat_X_e, inv_mat_X_e, res;
    float e = 0;

    opAd = operatorAd(mat_g);
    mat_X_e = X(e);
    cout << "\n\nMacierz X(e)\n\n"<<mat_X_e<<"\n\n" << endl;

    inv_mat_X_e = mat_X_e.inverse();
  
    res = inv_mat_X_e * opAd * mat_X_e; // result
    return res;
}


int main()
{
    Vector4f g, h, mat_opgrup, g_odwr;
    Matrix4f mat_opAd, mat_opAd_X;
 
    g << 1, 2, 3, 4;

    h << 10, 20, 30, 40;

    cout << "\n\nOperacja grupowa: \n" << endl;
    mat_opgrup = operacjaGrupowa(g, h);
    cout << mat_opgrup;

    cout << "\n\nOperator AD: \n" << endl; 
    
    mat_opAd = operatorAd(g);
    cout << mat_opAd;

    cout << "\n\nMacierz g:\n" << g;
    
    g_odwr= inv_g(g);
    cout << "\n\n Macierz odwrotna do g: \n\n" << g_odwr;

    cout << "\n\nOperacja grupowa g i g^-1:\n\n" << operacjaGrupowa(g, g_odwr);
    
    // inna kolejnosc - sprawdzenie - ok
    cout << "\n\nInna kolejnosc:" << endl;
    cout << "\nOperacja grupowa g i g^-1:\n" << operacjaGrupowa(g_odwr, g);

    // sprawdzenie operator AdX 
    mat_opAd_X = opAdX(g);
    cout << "\n\nZ operatora AdX:\n\n" << mat_opAd_X;
}
