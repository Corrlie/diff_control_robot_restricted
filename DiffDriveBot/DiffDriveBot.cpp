#include <iostream>
#include "MobileRobot.h"
#include "Simulation.h"
#include "Calculations.h"

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
	Calculations calculations1;
	calculations1.set_initial_matrices(mobileRobot);
	calculations1.calculate_integrals(mobileRobot, simulation1, myfile);



}
