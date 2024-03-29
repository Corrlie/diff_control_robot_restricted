#pragma once
#include <iostream>
#include "Calculations.h"
class Simulation
{
private:
	unsigned int num_of_samples;
	double step_size;
	std::ofstream m_oResultsFile;
	void InitializeResultrsFile();
	Calculations m_oCalculations;

public:
	Simulation();
	Simulation(const unsigned int& init_num_of_samples, const double& init_step_size);
	~Simulation();
	unsigned int get_num_of_samples() const;
	void set_num_of_samples(unsigned int new_num_of_samples);
	double get_step_size() const;
	void set_step_size(double new_step_size);
	void RunSimulation();
};
