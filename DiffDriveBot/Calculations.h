#pragma once
#include <Eigen/Dense>	
#include <fstream>
#include "MobileRobot.h"

using Eigen::Vector2d;
using Eigen::Vector4d;
using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::Matrix;


class Calculations
{
private:
    double time;
    unsigned int count_samples;
	MobileRobot m_oMobileRobot;

	Vector2d v;						// control signals
	Vector2d alfa;
	Vector2d dAlfa;
	Vector2d gCord;
	Vector2d Cord;

	Vector4d eta;					// reference signal
	Vector4d g;
	Vector4d dg;
	Vector4d f;
	Vector4d vHat;
	Vector4d gInv;
	Vector4d z;
	Vector4d dz;

	Matrix2d Rot;
	Matrix2d RotInv;

	Matrix4d X;						// controls
	Matrix4d cHat;
	Matrix4d cHatInv;
	Matrix4d adX;
	Matrix4d adXInv;
	Matrix4d T;

	Matrix<double, 4, 2> C;
	Matrix<double, 4, 2> G;

	double gfi = 0;
	double R = 0.5;

	double r_loaded, b_loaded, theta0_loaded, omega_loaded;

public:
    Calculations();
    ~Calculations();
	void set_initial_matrices();
    void calculate_integrals(const unsigned int& ui_num_of_samples,
								const double& d_step_size, 
								std::ofstream* p_oResultsFile);
};

