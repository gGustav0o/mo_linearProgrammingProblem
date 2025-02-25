#include <iostream>

#include <Eigen/Dense>

#include "../include/input/HardcodedInput.h"
#include "../include/interfaces/ILpOutput.h"
#include "../include/SimplexSolver.h"
#include "../include/output/ConsoleOutput.h"

int main(void)
{
    std::cout << "Linear Programming Solver\n";

    std::unique_ptr<ILpInput> input = std::make_unique<HardcodedInput>();
    std::unique_ptr<ILpOutput> output = std::make_unique<ConsoleOutput>();

    SimplexSolver solver;
    LpProblem problem;

    if (!input->loadProblem(problem)) {
        std::cerr << "Error: Failed to load linear programming problem.\n";
        return EXIT_FAILURE;
    }

    problem.convertToCanonical();


    std::vector<double> solution;
    double optimalValue = 0.0;
    bool hasSolution = solver.solve(problem, solution, optimalValue);

    if (hasSolution) {
        output->outputSolution(solution, optimalValue);
    }
    else {
        output->outputNoSolution();
    }

    return EXIT_SUCCESS;
}