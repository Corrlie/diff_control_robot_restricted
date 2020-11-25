// pbr_opGrupowa.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <Eigen/Dense>
# include <cmath>
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
Matrix4d operatorAd(Vector4d mat_g) {
    Matrix2d R, G, Zeros_22, Ident_22; 
    Matrix4d res;
    double fi = mat_g(2) - mat_g(3);

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

Vector4d operacjaGrupowa(Vector4d mat_g, Vector4d mat_h) {
    Matrix4d T;
    double fi = mat_g(2) - mat_g(3);

    T << cos(fi), -sin(fi), 0, 0,
        sin(fi), cos(fi), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    // mat_g(2) - g fi r
    // mat_g(3) - g fi l
    // mat_g(i) - odenisienie do i-tego elementu macierzy mat_g
    return mat_g + T * mat_h;
}

Vector4d inv_g(Vector4d mat_g) {
    Matrix4d T, T_trans;
    Vector4d res;
    double fi = mat_g(2) - mat_g(3);

    T << cos(fi), -sin(fi), 0, 0,
        sin(fi), cos(fi), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    T_trans = T.transpose();
    res = -T_trans * mat_g;

    return res;
}

Matrix4d X(double e) {
    Vector4d X1, X2, X3, X4;
    Matrix4d res;

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

Matrix4d opAdX(Vector4d mat_g) {

    Matrix4d opAd, mat_X_e, inv_mat_X_e, res;
    double e = 0;

    opAd = operatorAd(mat_g);
    mat_X_e = X(e);
    cout << "\n\nMacierz X(e)\n\n"<<mat_X_e<<"\n\n" << endl;

    inv_mat_X_e = mat_X_e.inverse();
  
    res = inv_mat_X_e * opAd * mat_X_e; // result
    return res;
}

Vector4d fun_f_alfa(Vector2d alfa) {

    double alfa1, alfa2, beta1, beta11, beta12, beta21, beta22, gamma1;

    Vector4d f_alfa;

    alfa1 = alfa(0);
    alfa2 = alfa(1);

    alfa1 = 1;
    alfa2 = 1;
    beta1 = 1;
    beta11 = 1;
    beta12 = 1;
    beta21 = 1;
    beta22 = 1;
    gamma1 = 1;
    
    
    f_alfa(0) = 0.25 * beta1 * sin(alfa1 + gamma1) * (2 - (pow(beta22, 2)) * (pow(cos(alfa2), 2)) - beta1 * beta22 * sin(alfa1 - gamma1) * cos(alfa2) - (pow(beta1, 2)) * (pow(sin(alfa1 - gamma1), 2))) - 0.25 * beta21 * beta22 * sin(2 * alfa2);
    f_alfa(1) = 0.25 * beta1 * sin(alfa1 + gamma1) * (beta1 * sin(alfa1 - gamma1) + 2 * beta22 * cos(alfa2)) + beta21 * sin(alfa2);
    f_alfa(2) = beta11 * sin(alfa1) + 0.5 * beta22 * cos(alfa2);
    f_alfa(3) = beta12 * cos(alfa1) - 0.5 * beta22 * cos(alfa2);
    return f_alfa;
}

Matrix<double, 4, 2> df_alfa(Vector2d alfa) {

    double alfa1, alfa2, beta1, beta11, beta12, beta21, beta22, gamma1;

    //alfa1 = 1;
    //alfa2 = 1;
    beta1 = 1;
    beta11 = 1;
    beta12 = 1;
    beta21 = 1;
    beta22 = 1;
    gamma1 = 1;

    Matrix<double, 4, 2> dfalfa;
    RowVector4d dalfa1, dalfa2;

    alfa1 = alfa(0);
    alfa2 = alfa(1);

    dalfa1 << -(beta1 * cos(alfa1 + gamma1) * ((pow(beta22, 2)) * (pow(cos(alfa2), 2)) + (pow(beta1, 2)) * (pow((sin(alfa1 - gamma1)), 2)) + beta1 * beta22 * cos(alfa2) * sin(alfa1 - gamma1) - 2)) / 4 - (beta1 * sin(alfa1 + gamma1) * (2 * (pow((cos(alfa1 - gamma1) * sin(alfa1 - gamma1) * beta1), 2)) + beta22 * cos(alfa2) * cos(alfa1 - gamma1) * beta1)) / 4,
        ((pow(beta1, 2)) * sin(alfa1 + gamma1) * cos(alfa1 - gamma1)) / 4 + (beta1 * cos(alfa1 + gamma1) * (2 * beta22 * cos(alfa2) + beta1 * sin(alfa1 - gamma1))) / 4,
        beta11* cos(alfa1),
        -beta11 * sin(alfa1);

    dalfa2 << (beta1 * sin(alfa1 + gamma1) * (2 * cos(alfa2) * sin(alfa2) * (pow(beta22, 2)) + beta1 * sin(alfa2) * sin(alfa1 - gamma1) * beta22)) / 4 - (beta21 * beta22 * cos(alfa2)) / 2,
        beta21* cos(alfa2) - (beta1 * beta22 * sin(alfa1 + gamma1) * sin(alfa2)) / 2,
        -(beta22 * sin(alfa2)) / 2,
        (beta22 * sin(alfa2)) / 2;

    dfalfa.col(0) = dalfa1;
    dfalfa.col(1) = dalfa2;

    return dfalfa;
}


int main()
{
    Vector4d g, h, mat_opgrup, g_odwr, f_alfa;
    Matrix4d mat_opAd, mat_opAd_X;

    Vector2d alfa;

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


    // sprawdzenie f(alfa)
    alfa << 1, 2;
    f_alfa = fun_f_alfa(alfa);

    cout << "\n\n\n funkcja f alfa:\n" << f_alfa;

    // sprawdzenie df/dalfa
    Matrix<double, 4, 2> df_dalfa;
    df_dalfa = df_alfa(alfa);
    cout << "\n df / dalfa:\n" << df_dalfa;
}
