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

void Calculations::set_initial_matrices(MobileRobot& mobileRobot)
{
	Rot << cos(mobileRobot.get_theta0()), -sin(mobileRobot.get_theta0()), sin(mobileRobot.get_theta0()), cos(mobileRobot.get_theta0());

	C << 1, 0,
		0, 1,
		0, 0,
		0, 0;

	alfa << 1, 1;

	g << 0, 0, 0, 0;
	z << 0, 0, 0, 0;

	// robots parameters
	r_loaded = mobileRobot.get_r();
	b_loaded = mobileRobot.get_b();
	theta0_loaded = mobileRobot.get_theta0();
	omega_loaded = mobileRobot.get_omega();

	alfa << 1, 1;

}

void Calculations::calculate_integrals(MobileRobot& mobileRobot, Simulation& simulation, std::ofstream& file)
{
	std::cout << "Calculating... Please wait..." << std::endl;
	if (file.is_open()) {
		for (int i = 0; i < simulation.get_num_of_samples(); i++)									// 1000 samples - 1s (if step is 0.001)
		{
			time = simulation.get_step_size() * i;

			eta << 0, 0, R* omega_loaded* cos(omega_loaded * time), -R * omega_loaded * sin(omega_loaded * time);

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
			alfa = alfa + simulation.get_step_size() * dAlfa;

			X = mobileRobot.calcX(g);
			dg = X * C * v;				// (10)
			g = g + simulation.get_step_size() * dg;

			dz = X * adX * cHat * vHat;
			z = z + simulation.get_step_size() * dz;

			if (i % 1000 == 0) {
				std::cout << "Done: " << i * 100 / simulation.get_num_of_samples() << "% \r" << std::flush;
			}
			
			file << time << " " << g(0) << " " << g(1) << " " << eta(2) << " " << eta(3) << " " << v(0) << " " << v(1) << " " << dAlfa(0) << " " << dAlfa(1) << " " << z(0) << " " << z(1) << "\n";
		
			count_samples += 1;

		}
		file.close();
	}
	else std::cout << "Unable to open file";


}

