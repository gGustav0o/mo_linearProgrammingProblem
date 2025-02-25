#pragma once

#include "LpProblem.h"

class SimplexSolver
{
public:
	explicit SimplexSolver() = default;
    _Success_(return) bool solve(_In_ const LpProblem & problem, _Out_ std::vector<double>&solution, _Out_ double& optimalValue);
};

