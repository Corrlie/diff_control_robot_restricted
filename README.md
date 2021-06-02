# diff_control_robot_restricted


C++ programming language approach for "Control of a differentially driven nonholomic robot subject to a restricted wheels rotation" project published by Dariusz Pazderski and Krzysztof Kozłowski.

The name of the repo stands for **diff**(erentially)_**control**(ed)_robot _ **restricted**(wheels rotation)

**The names of the main variables correspond to the names of the variables available in the article.**

## Languages:
- C++
- Python

## Libraries used:
- C++ part: 
   - Eigen, ofstream, math
- Python part:
    - matplotlib

## Description

The project is the C++ aproach of running the simulation of a differentialy controlled mobile robot (of 2.0 class), whose wheels cannot fully rotate. So the wheels cannot rotate continuously 360 degrees and the **rotation of them is restricted**.

C++ project (**"DiffDriveBot"** directory) uses **Eigen** C++ library, mainly for matrices and vectors calculations purposes.
Calculations results are being saved while running the program, to .txt file using **ofstream** (C++). 

Necessary plots are generated using Python project and **matplotlib** library (**"plots_results_python"** directory). They are also being saved in .svg (vector) format.

## Run the project

Running the projects consists of two stages. 
- First stage: run the project available in "DiffDriveBot" directory. After running the C++ project, the text file with the results (**results.txt**) was saved in the C++ project directory.

- Second stage: move "**results.txt**" file to the Python project directory. Then run the Python program. It automatically creates and shows the plots of saved data. It also saves these plots in Python project directory.


***Addition to the first stage:**

After running the  C++ project user has to set the number of samples and the step size. 
Saved txt file and plots were made for: number_of_samples = 200 000; step_size = 0.01.

The screen after running the program and setting two mentioned values, is shown below:

![1](https://user-images.githubusercontent.com/63510085/120474058-00c89700-c3a8-11eb-8087-38f83fffbcf3.png)

As one can see on the picture, there is an information about Starting the simulation, then initialization of integrations calculations. There is also the percentage value of the calculations progress (for example 8% on the screen).

When the calculations are done, user's screen is shown below:

![2](https://user-images.githubusercontent.com/63510085/120474609-b267c800-c3a8-11eb-9d37-57bf78f36e59.png)

so there is the information, which tells that the calculations have ended and also that the simulations process has ended.
There is also an instruction that tells the user what to do next to visualize the results in the form of graphs.

## Summary

The calculation results and plots correspond to the results and plots available in the article, thus the calculations were made correctly.
Saved txt file and plots were made for: number_of_samples = 200 000; step_size = 0.01.

