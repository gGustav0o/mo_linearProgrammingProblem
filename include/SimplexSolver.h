#pragma once

#include <numeric>

#include "../mo_linearProgrammingProblem/global.h"
#include "LpProblem.h"

class SimplexSolver
{
public:
	explicit SimplexSolver() = default;
    _Success_(return) bool solve(_In_ const LpProblem & problem, _Out_ std::vector<double>&solution, _Out_ double& optimalValue);
private:
	_Success_(return) bool ñomputeInitialVector
	(
		_In_ const Eigen::MatrixXd & constraintMatrix
		, _In_ const Eigen::VectorXd & rhsVector
		, _Out_ std::vector<double>&basicSolutionVector
	);

	_pure_(double getEpsilon(_In_ const Eigen::MatrixXd& A_B));

};

