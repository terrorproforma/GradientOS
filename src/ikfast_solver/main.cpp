#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <vector>
#define IKFAST_HAS_LIBRARY
#include "ikfast.h"

// The IKFast functions are declared in ikfast.h and implemented in ikfast_solver.cpp

int main(int argc, char** argv) {
    // Example target pose: a simple translation forward and up
    const double eetrans[3] = {0.3, 0.0, 0.4};
    // Identity rotation matrix (no rotation)
    const double eerot[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; 

    ikfast::IkSolutionList<double> solutions;

    // Call the IK solver
    bool found_solution = ComputeIk(eetrans, eerot, NULL, solutions);

    if (!found_solution) {
        std::cerr << "Failed to find an IK solution." << std::endl;
        return 1;
    }

    std::cout << "Found " << solutions.GetNumSolutions() << " solutions:" << std::endl;

    // Get and print the solutions
    const int num_joints = GetNumJoints();
    std::vector<double> solvalues(num_joints);
    for(std::size_t i = 0; i < solutions.GetNumSolutions(); ++i) {
        const ikfast::IkSolutionBase<double>& sol = solutions.GetSolution(i);
        
        sol.GetSolution(&solvalues[0], NULL); // Get the solution into our vector

        std::cout << "  Solution " << i + 1 << " (in degrees):" << std::endl;
        for (int j = 0; j < num_joints; ++j) {
            std::cout << "    Joint " << j + 1 << ": " << solvalues[j] * 180.0 / M_PI << std::endl;
        }
    }

    return 0;
} 