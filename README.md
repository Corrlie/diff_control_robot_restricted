C++ programming language approach for "Control of a differentially driven nonholomic robot subject to a restricted wheels rotation" project published by Dariusz Pazderski and Krzysztof Kozłowski.

C++ project ("DiffDriveBot" directory) uses Eigen C++ library, mainly for matrices and vectors calculations purposes. 

Calculations results are being saved while running the program, to .txt file  using ofstream (C++). 

Necessary plots are generated using Python project and matplotlib library ("plots_results_python" directory). They are also being saved in .svg (vector) format.

In C++ project considered nonholomic robot with restricted wheels rotation is represented as one class - custom "MobileRobot" class.

