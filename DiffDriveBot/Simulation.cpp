#include "Simulation.h"

Simulation::Simulation() 
	: num_of_samples(0), step_size(0)
{
	std::cout << "Simulation process started!" << std::endl;
	std::cout << "Default initial values!" << std::endl;
}

Simulation::Simulation(const unsigned int& init_num_of_samples, const double& init_step_size)
	: num_of_samples(init_num_of_samples), step_size(init_step_size)
{
	std::cout << "Simulation process started!" << std::endl;
}

Simulation::~Simulation()
{
	std::cout << "Simulation process ended!" << std::endl;
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