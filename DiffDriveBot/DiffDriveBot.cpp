#define _USE_MATH_DEFINES
#include <iostream>						// Strumien wejscia/wyjscia
#include <Eigen/Dense>					// Biblioteka Eigen
#include <cmath>						// Biblioteka do obliczen matematycznych
#include <fstream>

using namespace std;					// Przedrostek std
using namespace Eigen;					// Przedrostek Eigen

// gx - 0, 
// gy - 1, 
// gfi r - 2, 
// gfi l - 3 :: zmienna - indeks w macierzy g

/*
const int r = 0.026;							// Srednica kol
const int b = 0.066;							// Rozstaw kol
const float fi_R0 = 0;							// Wartosc poczatkowa orientacji kola prawego
const float fi_L0 = 0;							// Wartosc poczatkowa orientacji kola prawego
const float theta0 = -(r / b)*(fi_R0 - fi_L0);	// Wartosc poczatkowa orientacji robota
*/

/// ================================================= FUNKCJE ================================================== ///

// ------------------------------------------------ Operator Ad -------------------------------------------------- //
Matrix4d obliczOperatorAd(Vector4d g) {				// Operator Ad	(9)
	Matrix4d Wynik;									// Macierz wynikowa

	double fi = g(2) - g(3);						// Orientacja kol

	Wynik << cos(fi), -sin(fi), g(1), -g(1),
		sin(fi), cos(fi), -g(0), g(0),
		0, 0, 1, 0,
		0, 0, 0, 1;

	return Wynik;
}

// --------------------------------------------- Operacja grupowa ------------------------------------------------ //
Vector4d obliczOperacjaGrupowa(Vector4d g, Vector4d h) {		// Operacja grupowa
	Matrix4d T;													// Macierz klatkowa (R 0;  I 2x2 0)
	Vector4d gh;												// Macierz wynikowa
	double fi = g(2) - g(3);									// Orientacja kol

	T << cos(fi), -sin(fi), 0, 0,								// Macierz klatkowa (R 0;  I 2x2 0)
		sin(fi), cos(fi), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	gh = g + T * h;											// Operacja grupowa

	return gh;													// g - konfiguracja przeskalowana, h - operator translacji lewej
}

// ------------------------------------------------ Sterowania --------------------------------------------------- //
Matrix4d obliczX(Vector4d e) {
	Vector4d X1;							// Pole wektorowe sterowania X1
	Vector4d X2;							// Pole wektorowe sterowania X2
	Vector4d X3;							// Nawias Liego pierwszego rzedu X3
	Vector4d X4;							// Nawias Liego drugiego rzedu X4
	Matrix4d Wynik;							// Macierz wynikowa
	double fi;
	fi = e(2) - e(3);

	X1 << 0.5 * cos(fi), 0.5 * sin(fi), 1, 0;   // Pole wektorowe sterowania X1
	X2 << 0.5 * cos(fi), 0.5 * sin(fi), 0, 1; // Pole wektorowe sterowania X2
	X3 << -sin(fi), cos(fi), 0, 0;			// Nawias Liego pierwszego rzedu X3
	X4 << cos(fi), sin(fi), 0, 0;				// Nawias Liego drugiego rzedu X4

	Wynik.col(0) = X1;
	Wynik.col(1) = X2;
	Wynik.col(2) = X3;
	Wynik.col(3) = X4;

	return Wynik;												// Macierz sterowañ
}

// ------------------------------- Operator Ad na podstawie elementu neutralnego --------------------------------- //
Matrix4d obliczOperatorAdx(Vector4d g) {

	Matrix4d operatorAd;											// Operator Ad
	Matrix4d Xe;														// Macierz sterowan
	Matrix4d xeInv;													// Odwrocona macierz sterowan
	Matrix4d Wynik;													// Macierz wynikowa
	Vector4d e;														// Element neutralny

	e << 0, 0, 0, 0;
	operatorAd = obliczOperatorAd(g);								// Obliczanie operatora Ad
	Xe = obliczX(e);													// Obliczanie X

	xeInv = Xe.inverse();												// Obliczenie  macierzy odwrotnej X

	Wynik = xeInv * operatorAd * Xe;									// Obliczenie operatora Adx
	return Wynik;
}

// -------------------------------- Funkcja poprzeczna w sasiedztwie poczatku ------------------------------------ //
Vector4d obliczFAlfa(Vector2d alfa) {

	double alfa1;			// ??
	double alfa2;			// ??
	double beta1;			// Parametr beta1 = sqrt(beta11^2+beta12^2)
	double beta11;			// Parametr beta11
	double beta12;			// Parametr beta12
	double beta21;			// Parametr beta21
	double beta22;			// Parametr beta22
	double gamma1;			// ??

	Vector4d fAlfa;			// Funkcja wynikowa

	alfa1 = alfa(0);		// Przypisanie wartosci alfa1
	alfa2 = alfa(1);		// Przypisanie wartosci alfa1

	beta11 = 0.075;
	beta12 = 0.075;
	beta21 = 0.0124;
	beta22 = 0.4;
	beta1 = sqrt(pow(beta11, 2) + pow(beta12, 2));
	gamma1 = atan2(beta11, beta12);	// (18)

	fAlfa(0) = 0.25 * beta1 * sin(alfa1 + gamma1) * (2 - (pow(beta22, 2)) * (pow(cos(alfa2), 2)) - beta1 * beta22 * sin(alfa1 - gamma1) * cos(alfa2) - (1.0 / 3.0) * (pow(beta1, 2)) * (pow(sin(alfa1 - gamma1), 2))) - 0.25 * beta21 * beta22 * sin(2 * alfa2),
		fAlfa(1) = 0.25 * beta1 * sin(alfa1 + gamma1) * (beta1 * sin(alfa1 - gamma1) + 2 * beta22 * cos(alfa2)) + beta21 * sin(alfa2);
	fAlfa(2) = beta11 * sin(alfa1) + 0.5 * beta22 * cos(alfa2);
	fAlfa(3) = beta12 * cos(alfa1) - 0.5 * beta22 * cos(alfa2);
	return fAlfa;
}

// ----------------------------- Pochodna funkcji poprzecznej w sasiedztwie poczatku ----------------------------- //
Matrix<double, 4, 2> obliczDFAlfa(Vector2d alfa) {

	double alfa1;			// ??
	double alfa2;			// ??
	double beta1;			// Parametr beta1 = sqrt(beta11^2+beta12^2)
	double beta11;			// Parametr beta11
	double beta12;			// Parametr beta12
	double beta21;			// Parametr beta21
	double beta22;			// Parametr beta22
	double gamma1;			// ??

	beta11 = 0.075;
	beta12 = 0.075;
	beta21 = 0.0124;
	beta22 = 0.4;
	beta1 = sqrt(pow(beta11, 2) + pow(beta12, 2));
	gamma1 = atan2(beta11, beta12);

	Matrix<double, 4, 2> dfAlfa;
	RowVector4d dalfa1, dalfa2;

	alfa1 = alfa(0);
	alfa2 = alfa(1);

	dalfa1 << -1.0 / 4.0 * (beta11 * cos(alfa1) - beta12 * sin(alfa1)) * ((pow(beta22, 2)) * (pow(cos(alfa2), 2)) + 1.0 / 3.0 * (pow(beta12 * cos(alfa1) - beta11 * sin(alfa1), 2)) - beta22 * cos(alfa2) * (beta12 * cos(alfa1) - beta11 * sin(alfa1)) - 2.0) - 1.0 / 3.0 * ((beta11 * cos(alfa1) + beta12 * sin(alfa1)) * (beta12 * cos(alfa1) + beta11 * sin(alfa1)) / 4.0 * (3.0 * beta22 * cos(alfa2) - 2.0 * beta12 * cos(alfa1) + 2.0 * beta11 * sin(alfa1))),
		(pow(beta1, 2) * sin(alfa1 + gamma1) * cos(alfa1 - gamma1)) / 4 + (beta1 * cos(alfa1 + gamma1) * (2 * beta22 * cos(alfa2) + beta1 * sin(alfa1 - gamma1))) / 4,
		beta11* cos(alfa1),
		-beta12 * sin(alfa1);

	dalfa2 << 1.0 / 4.0 * beta22 * sin(alfa2) * (beta12 * cos(alfa1) + beta11 * sin(alfa1)) * (2.0 * beta22 * cos(alfa2) - beta12 * cos(alfa1) + beta11 * sin(alfa1)) - 1.0 / 2.0 * beta21 * beta22 * cos(2 * alfa2),
		beta21* cos(alfa2) - (beta1 * beta22 * sin(alfa1 + gamma1) * sin(alfa2)) / 2,
		-(beta22 * sin(alfa2)) / 2,
		(beta22 * sin(alfa2)) / 2;

	/*
		dalfa1 << -(pow(beta1, 2)*(2 * alfa1 - 2 * gamma1)) / 3 - (beta1*cos(alfa1 + gamma1)*pow(cos(alfa2), 2)*(pow(beta22, 2) - 2)) / 4 - beta1 * beta22*cos(alfa2)*cos(alfa1 - gamma1),
			(pow(beta1, 2)*sin(alfa1 + gamma1)*cos(alfa1 - gamma1)) / 4 + (beta1*cos(alfa1 + gamma1)*(2 * beta22*cos(alfa2) + beta1 * sin(alfa1 - gamma1))) / 4,
			beta11*cos(alfa1),
			-beta12 * sin(alfa1);

		dalfa2 << beta1 * beta22*sin(alfa2)*sin(alfa1 - gamma1) - (beta21*beta22*cos(2 * alfa2)) / 2 + (beta1*sin(alfa1 + gamma1)*cos(alfa2)*sin(alfa2)*(pow(beta22, 2) - 2)) / 2,
			beta21*cos(alfa2) - (beta1*beta22*sin(alfa1 + gamma1)*sin(alfa2)) / 2,
			-(beta22*sin(alfa2)) / 2,
			(beta22*sin(alfa2)) / 2;
	*/
	dfAlfa.col(0) = dalfa1;
	dfAlfa.col(1) = dalfa2;

	return dfAlfa;
}

// ------------------------------------------------- Wyznacznik c ------------------------------------------------ //
Matrix4d obliczC(Vector2d alfa) {
	Matrix<double, 4, 2> C;
	Matrix<double, 4, 2> dAlfa;
	Vector4d fAlfa;
	Matrix4d X;
	Matrix4d xInv;
	Matrix<double, 4, 2> AAlfa;
	Matrix4d Wynik;

	dAlfa = obliczDFAlfa(alfa);			// 4x2 
	fAlfa = obliczFAlfa(alfa);			// 4x1
	X = obliczX(fAlfa);					// 4x4
	xInv = X.inverse();					// 4x4
	AAlfa = xInv * dAlfa;				// 4x4*4x2 = 4x2 

	C << 1, 0,
		0, 1,
		0, 0,
		0, 0;


	Wynik.col(0) = C.col(0);
	Wynik.col(1) = C.col(1);
	Wynik.col(2) = -AAlfa.col(0);
	Wynik.col(3) = -AAlfa.col(1);

	return Wynik;
}

// ------------------------------------------------- T ------------------------------------------------ //
Matrix4d obliczT(double gfi) {
	Matrix2d I;
	Matrix4d Wynik;

	Wynik << cos(gfi), -sin(gfi), 0, 0,
		sin(gfi), cos(gfi), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	return Wynik;
}
/// ================================================= MAIN ================================================== ///

int main()
{
	Vector4d eta;					// sygnal referencyjny
	Matrix4d X;						// sterowania
	Vector4d g;
	Vector4d dg;
	Vector2d v;						// sygnal sterujacy
	Vector2d alfa;
	Vector2d dAlfa;
	Matrix4d cKreska;
	Matrix4d cKreskaInv;
	Vector4d f;
	Matrix<double, 4, 2> C;
	Matrix4d adX;
	Matrix4d adXInv;
	Matrix4d T;
	Vector4d vKreska;
	Vector2d gCord;
	Vector2d Cord;
	Matrix2d Rot;
	Matrix2d RotInv;
	Vector4d gInv;
	Vector4d z;
	Matrix<double, 4, 2> G;
	//Vector4d dq;
	//Vector4d q;
	//const int Ts = 10000;
	const int Ts = 10000;
	double ts = 0.1;
	double R = 0.5;
	double omega = M_PI / 1000;
	double r = 0.05;
	double b = 0.245;
	double theta0 = 0;
	double gfi = 0;
	//double x[Ts];
	//double y[Ts];
	double u1[Ts];
	double u2[Ts];
	double gx[Ts];
	double gy[Ts];
	double xr[Ts];
	double yr[Ts];
	double time[Ts];
	double da1[Ts];
	double da2[Ts];
	double z1[Ts];
	double z2[Ts];
	//double theta[Ts];
	//double thetar[Ts];
	//double thetal[Ts];

	int liczbaProbek = 0;

	Rot << cos(theta0), -sin(theta0), sin(theta0), cos(theta0);

	C << 1, 0,
		0, 1,
		0, 0,
		0, 0;

	alfa << 1, 1;

	//q << 0, 0, 0, 0;

	g << 0, 0, 0, 0;

	for (int i = 0; i < Ts; i++)									// 1000 - 1s (przy kroku 0.001)
	{
		time[i] = ts * i;

		eta << 0, 0, R* omega* cos(omega * time[i]), -R * omega * sin(omega * time[i]);

		f = obliczFAlfa(alfa);
		adX = obliczOperatorAdx(f);
		adXInv = adX.inverse();

		cKreska = obliczC(alfa);
		cKreskaInv = cKreska.inverse();

		gfi = g(2) - g(3);
		T = obliczT(gfi);
		gInv = -T.transpose() * g;
		z = obliczOperacjaGrupowa(g, gInv);

		vKreska = cKreskaInv * adXInv * eta;

		v << vKreska(0), vKreska(1);
		dAlfa << vKreska(2), vKreska(3);
		alfa = alfa + ts * dAlfa;

		X = obliczX(g);
		dg = X * C * v;				// (10)
		g = g + ts * dg;

		//theta[i] = r / b * gfi + theta0;
		//G << r/2*cos(theta[i]), r/2*cos(theta[i]), r/2*sin(theta[i]), r/2*sin(theta[i]), 1, 0, 0, 1;

		//dq = G * v;
		//q = q + ts * dq;

		//x[i] = q(0);
		//y[i] = q(1);
		//thetar[i] = q(2);
		//thetal[i] = q(3);

		gx[i] = g(0);
		gy[i] = g(1);

		xr[i] = eta(2);
		yr[i] = eta(3);
		u1[i] = v(0);
		u2[i] = v(1);
		da1[i] = dAlfa(0);
		da2[i] = dAlfa(1);
		z1[i] = z(0);
		z2[i] = z(1);

		//cout << "============================== \n";
		//cout << "         Czas: " << time[i] << " s             ";
		//cout << "\n============================== \n \n";
		//cout << q << "\n \n";
		liczbaProbek += 1;

	}
	
	ofstream myfile("x.txt");
	if (myfile.is_open())
	{
		myfile << "Czas" << " " << "X" << " " << "Y" << " " << "XR" << " " << "YR" << " " << "u1" << " " << "u2" << " " << "dalfa1" << " " << "dalfa2" << " " << "z1"<< " " << "z2"<< "\n";
		for (int count = 0; count < Ts; count++)
		{
			myfile << time[count] << " " << gx[count] << " " << gy[count] << " " << xr[count] << " " << yr[count] << " " << u1[count] << " " << u2[count] << " " << da1[count] << " " << da2[count]<< " "<<z1[count]<<" "<<z2[count]<< "\n";
		}
		myfile.close();
	}
	else cout << "Unable to open file";
	

}
