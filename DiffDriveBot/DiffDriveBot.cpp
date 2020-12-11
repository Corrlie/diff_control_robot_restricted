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
Matrix4d calc_opAd(Vector4d vec_g) {
	/*
		R - Rotation matrix
		G - Lie group
		fi - diff between angles
	*/
	Matrix2d R, G;

	Matrix4d res;
	double fi = vec_g(2) - vec_g(3);

	R << cos(fi), -sin(fi),
		sin(fi), cos(fi);

	G << vec_g(1), -vec_g(1),
		-vec_g(0), vec_g(0);

	res << R(0, 0), R(0, 1), G(0, 0), G(0, 1),
		R(1, 0), R(1, 1), G(1, 0), G(1, 1),
		0, 0, 1, 0,
		0, 0, 0, 1;

	return res;
}

Vector4d calc_opGroup(Vector4d vec_g, Vector4d vec_h) {
	Matrix4d T;
	double fi = vec_g(2) - vec_g(3); // wheels orientation
	T << cos(fi), -sin(fi), 0, 0,
		sin(fi), cos(fi), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	// vec_g(2) - g fi r
	// vec_g(3) - g fi l
	// vec_g(i) - odenisienie do i-tego elementu macierzy vec_g
	return vec_g + T * vec_h;
}

Vector4d inv_g(Vector4d vec_g) { // calculates inversion of vector g
	Matrix4d T, T_trans;
	Vector4d res;
	double fi = vec_g(2) - vec_g(3);

	T << cos(fi), -sin(fi), 0, 0,
		sin(fi), cos(fi), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	T_trans = T.transpose();
	res = -T_trans * vec_g;

	return res;
}

Matrix4d calc_X(Vector4d e) { // calculates control matrix
	Vector4d X1, X2, X3, X4; 
	Matrix4d res;
	double fi;
	fi = e(2) - e(3);

	X1 << 0.5 * cos(fi), 0.5 * sin(fi), 1, 0; 
	X2 << 0.5 * cos(fi), 0.5 * sin(fi), 0, 1;
	X3 << -sin(fi), cos(fi), 0, 0;
	X4 << cos(fi), sin(fi), 0, 0;

	//control matrix - compound of X1,2,3,4
	res.col(0) = X1;
	res.col(1) = X2;
	res.col(2) = X3;
	res.col(3) = X4;

	return res;
}

Matrix4d calc_opAdX(Vector4d vec_g) {

	Matrix4d opAd, mat_X_e, inv_mat_X_e, res;
	Vector4d e_v; // neutral element - zero vec
	e_v << 0, 0, 0, 0;
	opAd = calc_opAd(vec_g);
	mat_X_e = calc_X(e_v); // control matrix

	// cout << "\n\nMacierz X(e)\n\n"<<mat_X_e<<"\n\n" << endl;
	inv_mat_X_e = mat_X_e.inverse(); // inverse - MUST CREATE NEW MAT!

	res = inv_mat_X_e * opAd * mat_X_e; // result
	return res;
}

Vector4d fun_f(Vector2d alfa) { // function f(alfa)
	double alfa1, alfa2, beta1, beta11, beta12, beta21, beta22, gamma1, beta_h;
	Vector4d f_alfa;

	// parameters
	alfa1 = alfa(0);
	alfa2 = alfa(1);

	beta11 = 1;
	beta12 = 1;
	beta_h = pow(beta11, 2) + pow(beta12, 2);
	beta1 = sqrt(beta_h);
	beta21 = 1;
	beta22 = 1;
	gamma1 = atan2(beta11, beta12);

	f_alfa(0) = 0.25 * beta1 * sin(alfa1 + gamma1) * (2 - (pow(beta22, 2)) * (pow(cos(alfa2), 2)) - beta1 * beta22 * sin(alfa1 - gamma1) * cos(alfa2) - (1.0 / 3.0) * (pow(beta1, 2)) * (pow(sin(alfa1 - gamma1), 2))) - 0.25 * beta21 * beta22 * sin(2 * alfa2),
		f_alfa(1) = 0.25 * beta1 * sin(alfa1 + gamma1) * (beta1 * sin(alfa1 - gamma1) + 2 * beta22 * cos(alfa2)) + beta21 * sin(alfa2);
	f_alfa(2) = beta11 * sin(alfa1) + 0.5 * beta22 * cos(alfa2);
	f_alfa(3) = beta12 * cos(alfa1) - 0.5 * beta22 * cos(alfa2);
	return f_alfa;
}

Matrix<double, 4, 2> fun_df(Vector2d alfa) { // df/dalfa function 
	double alfa1, alfa2, beta1, beta11, beta12, beta21, beta22, gamma1, beta_h;
	Matrix<double, 4, 2> dfalfa;
	RowVector4d dalfa1, dalfa2;

	// parameters
	alfa1 = alfa(0);
	alfa2 = alfa(1);

	beta11 = 1;
	beta12 = 1;
	beta_h = pow(beta11, 2) + pow(beta12, 2);
	beta1 = sqrt(beta_h);
	beta21 = 1;
	beta22 = 1;
	gamma1 = atan2(beta11, beta12);

	dalfa1 << -1.0 / 4.0 * (beta11 * cos(alfa1) - beta12 * sin(alfa1)) * ((pow(beta22, 2)) * (pow(cos(alfa2), 2)) + 1.0 / 3.0 * (pow(beta12 * cos(alfa1) - beta11 * sin(alfa1), 2)) - beta22 * cos(alfa2) * (beta12 * cos(alfa1) - beta11 * sin(alfa1)) - 2.0) - 1.0 / 3.0 * ((beta11 * cos(alfa1) + beta12 * sin(alfa1)) * (beta12 * cos(alfa1) + beta11 * sin(alfa1)) / 4.0 * (3.0 * beta22 * cos(alfa2) - 2.0 * beta12 * cos(alfa1) + 2.0 * beta11 * sin(alfa1))),
		(pow(beta1, 2) * sin(alfa1 + gamma1) * cos(alfa1 - gamma1)) / 4 + (beta1 * cos(alfa1 + gamma1) * (2 * beta22 * cos(alfa2) + beta1 * sin(alfa1 - gamma1))) / 4,
		beta11* cos(alfa1),
		-beta12 * sin(alfa1);

	dalfa2 << 1.0 / 4.0 * beta22 * sin(alfa2) * (beta12 * cos(alfa1) + beta11 * sin(alfa1)) * (2.0 * beta22 * cos(alfa2) - beta12 * cos(alfa1) + beta11 * sin(alfa1)) - 1.0 / 2.0 * beta21 * beta22 * cos(2 * alfa2),
		beta21* cos(alfa2) - (beta1 * beta22 * sin(alfa1 + gamma1) * sin(alfa2)) / 2,
		-(beta22 * sin(alfa2)) / 2,
		(beta22 * sin(alfa2)) / 2;

	dfalfa.col(0) = dalfa1;
	dfalfa.col(1) = dalfa2;

	return dfalfa;
}


Matrix4d calc_C_hat(Vector2d alfa) { // estimated C matrix
	Matrix<double, 4, 2> C;
	Matrix<double, 4, 2> dAlfa;
	Vector4d fAlfa;
	Matrix4d mat_X_f;
	Matrix4d xInv;
	Matrix<double, 4, 2> AAlfa;
	Matrix4d res;

	dAlfa = fun_df(alfa);	// 4x2 
	fAlfa = fun_f(alfa);	// 4x1
	mat_X_f = calc_X(fAlfa);	// 4x4
	xInv = mat_X_f.inverse();	// 4x4
	AAlfa = xInv * dAlfa;	// 4x4*4x2 = 4x2

	C << 1, 0,
		0, 1,
		0, 0,
		0, 0;

	res.col(0) = C.col(0);
	res.col(1) = C.col(1);
	res.col(2) = -AAlfa.col(0);
	res.col(3) = -AAlfa.col(1);

	//	cout << "\n\n\n===================C========================\n" << C;
	//	cout << "\n\n\n===================AAlfa========================\n" << AAlfa;
	//	cout << "\n\n\n===================Czdaszkiem========================\n" << res<<"\n\n";
	return res;
}

Vector4d calc_dG(Vector4d g, Vector2d v, Vector2d alfa) { // calculate g derivative
	Matrix<double, 4, 2> C;
	Matrix4d mat_X_g;
	Vector4d res;

	mat_X_g = calc_X(g);	// 4x4
	C << 1, 0, 0, 0,	// = 4x2 
		0, 1, 0, 0;

	res = mat_X_g * C * v;	// 4x4 * 4x2 = 4x2*2x1 = 4x1

	return res;
}


int main()
{
	Vector4d g, h, mat_opgrup, vec_gInv, f_alfa;
	// g - scaled robot configuration
	Matrix4d mat_opAd, mat_opAd_X, C_hat;

	Vector2d alfa, v;
	double ts = 0;

	g << 1, 2, 3, 4;

	h << 10, 20, 30, 40;

	cout << "\n\n=============== Operacja grupowa: ===============n" << endl;
	mat_opgrup = calc_opGroup(g, h);
	cout << mat_opgrup;

	cout << "\n\n=============== Operator AD: ===============\n" << endl;

	mat_opAd = calc_opAd(g);
	cout << mat_opAd;

	cout << "\n\n=============== Macierz g: ===============\n" << g << "\n-------------------------";

	vec_gInv = inv_g(g);
	cout << "\n\n=============== Macierz odwrotna do g: ===============\n\n" << vec_gInv << "\n-------------------------";

	cout << "\n\n=============== Operacja grupowa g i g^-1: ===============\n\n" << calc_opGroup(g, vec_gInv) << "\n-------------------------";

	// inna kolejnosc - sprawdzenie - ok
	cout << "\n\n=============== Inna kolejnosc: ===============" << endl;
	cout << "\n\n=============== Operacja grupowa g i g^-1: ===============\n" << calc_opGroup(vec_gInv, g) << "\n-------------------------";

	// sprawdzenie operator AdX 
	mat_opAd_X = calc_opAdX(g);
	cout << "\n\n=============== Z operatora AdX: ===============\n\n" << mat_opAd_X << "\n-------------------------";


	// sprawdzenie f(alfa)
	alfa << 1, 1;
	f_alfa = fun_f(alfa);

	cout << "\n\n=============== funkcja f alfa: ===============\n" << f_alfa << "\n-------------------------";

	// sprawdzenie df/dalfa
	Matrix<double, 4, 2> df_dalfa;
	df_dalfa = fun_df(alfa);
	cout << "\n\n=============== df / dalfa: ===============\n" << df_dalfa << "\n-------------------------";


	// C z daszkiem

	C_hat = calc_C_hat(alfa);
	cout << "\n\n=============== C_hat: ===============\n" << C_hat << "\n-------------------------";


	v << 1, 1;
	/*

	for (int i = 0; i < 1000; i++)
	{
		mat_opGr = calc_opGroup(g, h);
		mat_operatorAd = calc_opAd(g);
		mat_gInv = inv_g(g);
		operacjaGrupowa2 = calc_opGroup(g, mat_gInv);
		Ckreska = calc_C_hat(alfa);
		dG = calc_dG(g, v, alfa);

		g = g + ts * dG;

		ts = +0.001;
		cout << "============================================================" << endl;
		cout << "Macierz g: \n" << g << "\n";
	}
	*/
}
