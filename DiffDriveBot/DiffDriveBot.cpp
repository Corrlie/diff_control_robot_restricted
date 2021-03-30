#define _USE_MATH_DEFINES
#include <iostream>			
#include <Eigen/Dense>				
#include <cmath>						
#include <fstream>

using namespace std;				
using namespace Eigen;					

// ------------------------------------------------ Operator Ad -------------------------------------------------- //
Matrix4d calcOperatorAd(Vector4d g) {				// Operator Ad	(9)
	Matrix4d result;									

	double fi = g(2) - g(3);						// Wheels orientation

	result << cos(fi), -sin(fi), g(1), -g(1),
		sin(fi), cos(fi), -g(0), g(0),
		0, 0, 1, 0,
		0, 0, 0, 1;

	return result;
}

// --------------------------------------------- Group operation ------------------------------------------------ //
Vector4d calcGroupOperation(Vector4d g, Vector4d h) {		// Operacja grupowa
	Matrix4d T;												
	Vector4d gh;												
	double fi = g(2) - g(3);									// Wheels orientation

	T << cos(fi), -sin(fi), 0, 0,								
		sin(fi), cos(fi), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	gh = g + T * h;										

	return gh;													// g - scaled configuration, h - left translation operator
}

// ------------------------------------------------ Controls --------------------------------------------------- //
Matrix4d calcX(Vector4d e) {
	Vector4d X1;							// Control vector field X1
	Vector4d X2;							// Control vector field X2
	Vector4d X3;							// Lie Bracket - first rank X3
	Vector4d X4;							// Lie Bracket - second rank X4
	Matrix4d result;							
	double fi;
	fi = e(2) - e(3);

	X1 << 0.5 * cos(fi), 0.5 * sin(fi), 1, 0; 
	X2 << 0.5 * cos(fi), 0.5 * sin(fi), 0, 1; 
	X3 << -sin(fi), cos(fi), 0, 0;			
	X4 << cos(fi), sin(fi), 0, 0;				

	result.col(0) = X1;
	result.col(1) = X2;
	result.col(2) = X3;
	result.col(3) = X4;

	return result;												// control Matrix
}

// ------------------------------- Ad operator based on neutral element --------------------------------- //
Matrix4d calcOperatorAdX(Vector4d g) {

	Matrix4d operatorAd;											// Operator Ad
	Matrix4d Xe;													// Control matrix
	Matrix4d xeInv;													// inversed control matrix
	Matrix4d res;													
	Vector4d e;														// neutral element

	e << 0, 0, 0, 0;
	operatorAd = calcOperatorAd(g);								
	Xe = calcX(e);													

	xeInv = Xe.inverse();												

	res = xeInv * operatorAd * Xe;								
	return res;
}
// ------------------------------- Calculate f(alfa) function --------------------------------- //
Vector4d calcFAlfa(Vector2d alfa) {

	double alfa1;			
	double alfa2;			
	double beta1;			
	double beta11;			
	double beta12;			
	double beta21;			
	double beta22;			
	double gamma1;			

	Vector4d fAlfa;			// Result function

	alfa1 = alfa(0);		
	alfa2 = alfa(1);		

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

// ----------------------------- Derivative of f(alfa) function ----------------------------- //
Matrix<double, 4, 2> calcDFAlfa(Vector2d alfa) {

	double alfa1;			
	double alfa2;			
	double beta1;		
	double beta11;		
	double beta12;	
	double beta21;		
	double beta22;
	double gamma1;	

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

	dfAlfa.col(0) = dalfa1;
	dfAlfa.col(1) = dalfa2;

	return dfAlfa;
}

// ------------------------------------------------- C determinant ------------------------------------------------ //
Matrix4d calcC(Vector2d alfa) {
	Matrix<double, 4, 2> C;
	Matrix<double, 4, 2> dAlfa;
	Vector4d fAlfa;
	Matrix4d X;
	Matrix4d xInv;
	Matrix<double, 4, 2> AAlfa;
	Matrix4d result;

	dAlfa = calcDFAlfa(alfa);			// 4x2 
	fAlfa = calcFAlfa(alfa);			// 4x1
	X = calcX(fAlfa);					// 4x4
	xInv = X.inverse();					// 4x4
	AAlfa = xInv * dAlfa;				// 4x4*4x2 = 4x2 

	C << 1, 0,
		0, 1,
		0, 0,
		0, 0;


	result.col(0) = C.col(0);
	result.col(1) = C.col(1);
	result.col(2) = -AAlfa.col(0);
	result.col(3) = -AAlfa.col(1);

	return result;
}

// ------------------------------------------------- T ------------------------------------------------ //
Matrix4d calcT(double gfi) {
	Matrix2d I;
	Matrix4d result;

	result << cos(gfi), -sin(gfi), 0, 0,
		sin(gfi), cos(gfi), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	return result;
}

int main()
{
	Vector4d eta;					// reference signal
	Matrix4d X;						// controls
	Vector4d g;
	Vector4d dg;
	Vector2d v;						// control signals
	Vector2d alfa;
	Vector2d dAlfa;
	Matrix4d cHat;
	Matrix4d cHatInv;
	Vector4d f;
	Matrix<double, 4, 2> C;
	Matrix4d adX;
	Matrix4d adXInv;
	Matrix4d T;
	Vector4d vHat;
	Vector2d gCord;
	Vector2d Cord;
	Matrix2d Rot;
	Matrix2d RotInv;
	Vector4d gInv;
	Vector4d z;
	Vector4d dz;
	Matrix<double, 4, 2> G;
	ofstream myfile("x.txt");

	const int Ts = 200000;
	double ts = 0.01;
	double R = 0.5;
	double omega = M_PI / 1000;
	double r = 0.05;
	double b = 0.245;
	double theta0 = 0;
	double gfi = 0;
	double time;

	int numOfSamples = 0;

	Rot << cos(theta0), -sin(theta0), sin(theta0), cos(theta0);

	C << 1, 0,
		0, 1,
		0, 0,
		0, 0;

	alfa << 1, 1;

	g << 0, 0, 0, 0;
	z << 0, 0, 0, 0;

	if (myfile.is_open())
	{

		for (int i = 0; i < Ts; i++)									// 1000 - 1s (if step is 0.001)
		{
			time = ts * i;

			eta << 0, 0, R* omega* cos(omega * time), -R * omega * sin(omega * time);

			f = calcFAlfa(alfa);
			adX = calcOperatorAdX(f);
			adXInv = adX.inverse();

			cHat = calcC(alfa);
			cHatInv = cHat.inverse();

			gfi = g(2) - g(3);
			T = calcT(gfi);
			gInv = -T.transpose() * g;

			vHat = cHatInv * adXInv * eta;

			v << vHat(0), vHat(1);
			dAlfa << vHat(2), vHat(3);
			alfa = alfa + ts * dAlfa;

			X = calcX(g);
			dg = X * C * v;				// (10)
			g = g + ts * dg;

			dz = X * adX * cHat * vHat;
			z = z + ts * dz;

			numOfSamples += 1;
			myfile << time << " " << g(0) << " " << g(1) << " " << eta(2) << " " << eta(3) << " " << v(0) << " " << v(1) << " " << dAlfa(0) << " " << dAlfa(1) << " " << z(0) << " " << z(1) << "\n";
		}
		myfile.close();
	}
	else cout << "Unable to open file";


}
