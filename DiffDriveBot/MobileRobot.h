#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/Dense>	
class MobileRobot
{
private: 
	double r, b, theta0, omega;
public:
	MobileRobot(double r_wheel = 0.05, double b_wheels = 0.245, double theta0_orient = 0, double omega_vel = M_PI / 1000);
	double get_r();
	double get_b();
	double get_theta0();
	double get_omega();
	Eigen::Matrix4d calcOperatorAd(Eigen::Vector4d g);
	Eigen::Vector4d calcGroupOperation(Eigen::Vector4d g, Eigen::Vector4d h);
	Eigen::Matrix4d calcX(Eigen::Vector4d e);
	Eigen::Matrix4d calcOperatorAdX(Eigen::Vector4d g);
	Eigen::Vector4d calcFAlfa(Eigen::Vector2d alfa);
	Eigen::Matrix<double, 4, 2> calcDFAlfa(Eigen::Vector2d alfa);
	Eigen::Matrix4d calcC(Eigen::Vector2d alfa);
	Eigen::Matrix4d calcT(double gfi);

};

