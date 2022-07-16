#include <iostream>
#include "Simulation.h"

int main()
{
	unsigned int num_of_samples;
	double step_size;
	std::cout << "What is the total number of samples?" << std::endl;
	std::cin >> num_of_samples;
	std::cout << "What is the step size?" << std::endl;
	std::cin >> step_size;

	Simulation simulation1(num_of_samples, step_size);
	simulation1.RunSimulation();
}
