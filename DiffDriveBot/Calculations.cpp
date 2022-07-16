#pragma once
#include <iostream>
#include "Calculations.h"

Calculations::Calculations()
	:time(0), count_samples(0), r_loaded(0.0), b_loaded(0.0), theta0_loaded(0.0), omega_loaded(0.0)
{
    std::cout << "Initialization of integration calculations" << std::endl;
}

Calculations::~Calculations()
{
    std::cout << "\rIntegration calculations has ended!\n" << std::endl;
}

void Calculations::set_initial_matrices()
{
	Rot << cos(m_oMobileRobot.get_theta0()), -sin(m_oMobileRobot.get_theta0()), sin(m_oMobileRobot.get_theta0()), cos(m_oMobileRobot.get_theta0());

	C << 1, 0,
		0, 1,
		0, 0,
		0, 0;

	alfa << 1, 1;

	g << 0, 0, 0, 0;
	z << 0, 0, 0, 0;

	// robots parameters
	r_loaded = m_oMobileRobot.get_r();
	b_loaded = m_oMobileRobot.get_b();
	theta0_loaded = m_oMobileRobot.get_theta0();
	omega_loaded = m_oMobileRobot.get_omega();

	alfa << 1, 1;

}

void Calculations::calculate_integrals(const unsigned int& ui_num_of_samples,
										const double& d_step_size,
										std::ofstream* p_oResultsFile)
{
	assert(p_oResultsFile != nullptr);

	std::cout << "Calculating... Please wait..." << std::endl;
	if (p_oResultsFile->is_open()) {
		for (unsigned int i = 0; i < ui_num_of_samples; i++)									// 1000 samples - 1s (if step is 0.001)
		{
			time = d_step_size * i;

			eta << 0, 0, R* omega_loaded* cos(omega_loaded * time), -R * omega_loaded * sin(omega_loaded * time);

			f = m_oMobileRobot.calcFAlfa(alfa);
			adX = m_oMobileRobot.calcOperatorAdX(f);
			adXInv = adX.inverse();

			cHat = m_oMobileRobot.calcC(alfa);
			cHatInv = cHat.inverse();

			gfi = g(2) - g(3);
			T = m_oMobileRobot.calcT(gfi);
			gInv = -T.transpose() * g;

			vHat = cHatInv * adXInv * eta;

			v << vHat(0), vHat(1);
			dAlfa << vHat(2), vHat(3);
			alfa = alfa + d_step_size * dAlfa;

			X = m_oMobileRobot.calcX(g);
			dg = X * C * v;				// (10)
			g = g + d_step_size * dg;

			dz = X * adX * cHat * vHat;
			z = z + d_step_size * dz;

			if (i % 1000 == 0) {
				std::cout << "Done: " << i * 100 / ui_num_of_samples << "% \r" << std::flush;
			}
			
			*p_oResultsFile << time << " " << g(0) << " " << g(1) << " " << eta(2) << " " << eta(3) << " " << v(0) << " " << v(1) << " " << dAlfa(0) << " " << dAlfa(1) << " " << z(0) << " " << z(1) << "\n";
		
			count_samples += 1;

		}
		p_oResultsFile->close();
	}
	else std::cout << "Unable to open file";


}

