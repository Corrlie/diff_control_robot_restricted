#include <iostream>
#include "MobileRobot.h"
#include "Simulation.h"
#include "Simulation_integration.h"

int main()
{
	
	MobileRobot mobileRobot;	// custom object of mobile robot with restricted wheels movement

	unsigned int num_of_samples;
	double step_size;

	std::ofstream myfile("results.txt");	// write results to txt file

	std::cout << "What is the total number of samples?" << std::endl;
	std::cin >> num_of_samples;
	std::cout << "What is the step size?" << std::endl;
	std::cin >> step_size;

	Simulation simulation1(num_of_samples, step_size);
	Simulation_integration simulation_integration1;
	simulation_integration1.set_initial_matrices(mobileRobot);
	simulation_integration1.calculate_integrals(mobileRobot, simulation1, myfile);



}
