#pragma once
#include <Eigen/Dense>	
class MobileRobot
{
public:
	MobileRobot();
	Eigen::Matrix4d calcOperatorAd(Eigen::Vector4d g);
	Eigen::Vector4d calcGroupOperation(Eigen::Vector4d g, Eigen::Vector4d h);
	Eigen::Matrix4d calcX(Eigen::Vector4d e);
	Eigen::Matrix4d calcOperatorAdX(Eigen::Vector4d g);
	Eigen::Vector4d calcFAlfa(Eigen::Vector2d alfa);
	Eigen::Matrix<double, 4, 2> calcDFAlfa(Eigen::Vector2d alfa);
	Eigen::Matrix4d calcC(Eigen::Vector2d alfa);
	Eigen::Matrix4d calcT(double gfi);

};

