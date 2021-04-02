#include "MobileRobot.h"

MobileRobot::MobileRobot()
{
}

Eigen::Matrix4d MobileRobot::calcOperatorAd(Eigen::Vector4d g)
{
	Eigen::Matrix4d result;

	double fi = g(2) - g(3);						// Wheels orientation

	result << cos(fi), -sin(fi), g(1), -g(1),
		sin(fi), cos(fi), -g(0), g(0),
		0, 0, 1, 0,
		0, 0, 0, 1;

	return result;
}

Eigen::Vector4d MobileRobot::calcGroupOperation(Eigen::Vector4d g, Eigen::Vector4d h)
{
	Eigen::Matrix4d T;
	Eigen::Vector4d gh;
	double fi = g(2) - g(3);									// Wheels orientation

	T << cos(fi), -sin(fi), 0, 0,
		sin(fi), cos(fi), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	gh = g + T * h;

	return gh;													// g - scaled configuration, h - left translation operator
}

Eigen::Matrix4d MobileRobot::calcX(Eigen::Vector4d e)
{
	Eigen::Vector4d X1;							// Control vector field X1
	Eigen::Vector4d X2;							// Control vector field X2
	Eigen::Vector4d X3;							// Lie Bracket - first rank X3
	Eigen::Vector4d X4;							// Lie Bracket - second rank X4
	Eigen::Matrix4d result;
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

Eigen::Matrix4d MobileRobot::calcOperatorAdX(Eigen::Vector4d g)
{
	Eigen::Matrix4d operatorAd;											// Operator Ad
	Eigen::Matrix4d Xe;													// Control matrix
	Eigen::Matrix4d xeInv;													// inversed control matrix
	Eigen::Matrix4d res;
	Eigen::Vector4d e;														// neutral element

	e << 0, 0, 0, 0;
	operatorAd = calcOperatorAd(g);
	Xe = calcX(e);

	xeInv = Xe.inverse();

	res = xeInv * operatorAd * Xe;
	return res;
}

Eigen::Vector4d MobileRobot::calcFAlfa(Eigen::Vector2d alfa)
{
	double alfa1;
	double alfa2;
	double beta1;
	double beta11;
	double beta12;
	double beta21;
	double beta22;
	double gamma1;

	Eigen::Vector4d fAlfa;			// Result function

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

Eigen::Matrix<double, 4, 2> MobileRobot::calcDFAlfa(Eigen::Vector2d alfa)
{
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

	Eigen::Matrix<double, 4, 2> dfAlfa;
	Eigen::RowVector4d dalfa1, dalfa2;

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

Eigen::Matrix4d MobileRobot::calcC(Eigen::Vector2d alfa)
{
	Eigen::Matrix<double, 4, 2> C;
	Eigen::Matrix<double, 4, 2> dAlfa;
	Eigen::Vector4d fAlfa;
	Eigen::Matrix4d X;
	Eigen::Matrix4d xInv;
	Eigen::Matrix<double, 4, 2> AAlfa;
	Eigen::Matrix4d result;

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

Eigen::Matrix4d MobileRobot::calcT(double gfi)
{
	Eigen::Matrix2d I;
	Eigen::Matrix4d result;

	result << cos(gfi), -sin(gfi), 0, 0,
		sin(gfi), cos(gfi), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	return result;
}
