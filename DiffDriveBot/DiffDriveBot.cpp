#include <iostream>			
#include <Eigen/Dense>										
#include <fstream>
#include "MobileRobot.h"				
using namespace Eigen;					

int main()
{
	MobileRobot mobileRobot;	// custom object of mobile robot with restricted wheels movement

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
	std::ofstream myfile("x.txt");	// write results to txt file

	const int Ts = 200000;	// total number of samples
	double ts = 0.01;	// step size
	double time;

	// robots parameters
	double r = mobileRobot.get_r();
	double b = mobileRobot.get_b();
	double theta0 = mobileRobot.get_theta0();
	double omega = mobileRobot.get_omega();

	double gfi = 0;
	double R = 0.5;

	
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

			f = mobileRobot.calcFAlfa(alfa);
			adX = mobileRobot.calcOperatorAdX(f);
			adXInv = adX.inverse();

			cHat = mobileRobot.calcC(alfa);
			cHatInv = cHat.inverse();

			gfi = g(2) - g(3);
			T = mobileRobot.calcT(gfi);
			gInv = -T.transpose() * g;

			vHat = cHatInv * adXInv * eta;

			v << vHat(0), vHat(1);
			dAlfa << vHat(2), vHat(3);
			alfa = alfa + ts * dAlfa;

			X = mobileRobot.calcX(g);
			dg = X * C * v;				// (10)
			g = g + ts * dg;

			dz = X * adX * cHat * vHat;
			z = z + ts * dz;

			numOfSamples += 1;
			myfile << time << " " << g(0) << " " << g(1) << " " << eta(2) << " " << eta(3) << " " << v(0) << " " << v(1) << " " << dAlfa(0) << " " << dAlfa(1) << " " << z(0) << " " << z(1) << "\n";
		}
		myfile.close();
	}
	else std::cout << "Unable to open file";


}
