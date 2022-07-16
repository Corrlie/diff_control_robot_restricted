#include "Simulation.h"

Simulation::Simulation() 
	: num_of_samples(0), step_size(0)
{
	InitializeResultrsFile();
	std::cout << "Simulation process started!" << std::endl;
	std::cout << "Default simulation parameters!" << std::endl;
}

Simulation::Simulation(const unsigned int& init_num_of_samples, const double& init_step_size)
	: num_of_samples(init_num_of_samples), step_size(init_step_size)
{
	InitializeResultrsFile();
	std::cout << "Simulation process started!" << std::endl;
}

Simulation::~Simulation()
{
	std::cout << "\nThe simulation process has ended!\n" << std::endl;

	std::cout << "To get the graphs: \n";
	std::cout << "1) Move the results.txt file to the \"plot_results_python\" directory \n2) Run the \"mobile_robot_plots.py\" file located there" << std::endl;
	std::cout << "Thanks for using the program, goodbye\n" << std::endl;
}

unsigned int Simulation::get_num_of_samples() const
{
	return num_of_samples;
}

void Simulation::set_num_of_samples(unsigned int new_num_of_samples)
{
	num_of_samples = new_num_of_samples;
}

double Simulation::get_step_size() const
{
	return step_size;
}

void Simulation::set_step_size(double new_step_size)
{
	step_size = new_step_size;
}

void Simulation::InitializeResultrsFile()
{
	std::ofstream m_oResultsFile("results.txt");
}

void Simulation::RunSimulation()
{
	m_oCalculations.set_initial_matrices();
	m_oCalculations.calculate_integrals(get_num_of_samples(), 
										get_step_size(), 
										&m_oResultsFile);

}
