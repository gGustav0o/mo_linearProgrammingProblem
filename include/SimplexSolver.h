#pragma once

#include <iostream>
#include <numeric>

#include "../mo_linearProgrammingProblem/global.h"
#include "LpProblem.h"

class SimplexSolver
{
public:
	explicit SimplexSolver() = default;
    _Success_(return) bool solve(_In_ const LpProblem & problem, _Out_ std::vector<double>&solution, _Out_ double& optimalValue);
private:
	_Success_(return) bool ñomputeInitialVector(
		_In_ const Eigen::MatrixXd & constraintMatrix
		, _In_ const Eigen::VectorXd & rhsVector
		, _Out_ std::vector<double>&basicSolutionVector
	);
	Eigen::VectorXd computeReducedCostVector(
		_In_ const Eigen::MatrixXd& basisMatrix
		, _In_ const Eigen::MatrixXd& fullConstraintMatrix
		, _In_ const Eigen::VectorXd& costVector
	);
	void splitReducedCostVector(
		_In_ const Eigen::VectorXd& reducedCostVector
		, _In_ const std::vector<int>& basisIndices
		, _In_ const std::vector<int>& nonBasisIndices
		, _Out_ Eigen::VectorXd& reducedCostVectorBasis
		, _Out_ Eigen::VectorXd& reducedCostVectorNonBasis
	);
	Eigen::VectorXd computeUk(
		_In_ const Eigen::MatrixXd& inverseBasisMatrix
		, _In_ const Eigen::VectorXd& enteringColumn
	);
	Eigen::VectorXd computeNewBasicSolution(
		_In_ const Eigen::VectorXd& currentSolution
		, _In_ const Eigen::VectorXd& directionVector
		, _In_ double stepSize
	);
	double computeTheta(
		_In_ const Eigen::VectorXd& currentSolution
		, _In_ const Eigen::VectorXd& directionVector
	);
	_Success_(return) bool updateBasis(
		_In_ const Eigen::MatrixXd & basisMatrix
		, _In_ const Eigen::MatrixXd & fullConstraintMatrix
		, _In_ int enteringIndex
		, _Out_ Eigen::MatrixXd & updatedBasisMatrix
	);
	Eigen::MatrixXd computeNextBasisMatrix(
		_In_ const Eigen::MatrixXd& currentBasisMatrix
		, _In_ int enteringColumnIndex
		, _In_ int leavingColumnIndex
		, _In_ const Eigen::VectorXd& directionVector
	);

	static double getEpsilon(_In_ const Eigen::MatrixXd& A_B);
};

