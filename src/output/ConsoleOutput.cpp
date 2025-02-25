#include "../../include/output/ConsoleOutput.h"

void ConsoleOutput::outputSolution(const std::vector<double>& solution, double optimalValue)
{
    std::cout << "Optimal solution found:\n";
    for (size_t i = 0; i < solution.size(); ++i) {
        std::cout << "x" << i + 1 << " = " << solution[i] << "\n";
    }
    std::cout << "Optimal objective function value: " << optimalValue << "\n";
}

void ConsoleOutput::outputNoSolution()
{
    std::cout << "The linear programming problem has no feasible solution or the objective function is unbounded.\n";
}
