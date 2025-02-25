#include "../include/SimplexSolver.h"

bool SimplexSolver::solve(const LpProblem& problem, std::vector<double>& solution, double& optimalValue)
{
	return false;
}

bool SimplexSolver::ñomputeInitialVector(
    _In_ const Eigen::MatrixXd& constraintMatrix
    , _In_ const Eigen::VectorXd& rhsVector
    , _Out_ std::vector<double>& basicSolutionVector
) {
    int numVariables = constraintMatrix.cols();
    int numConstraints = constraintMatrix.rows();
    basicSolutionVector.assign(numVariables, 0.0);

    std::vector<int> columnIndices(numVariables);
    std::iota(columnIndices.begin(), columnIndices.end(), 0);

    do {
        Eigen::MatrixXd basisMatrix(numConstraints, numConstraints);
        for (int i = 0; i < numConstraints; ++i) {
            basisMatrix.col(i) = constraintMatrix.col(columnIndices[i]);
        }

        if (std::abs(basisMatrix.determinant()) > getEpsilon(basisMatrix)) {
            Eigen::VectorXd basicVariablesVector = basisMatrix.fullPivLu().solve(rhsVector);

            if ((basicVariablesVector.array() >= 0).all()) {
                basicSolutionVector.assign(numVariables, 0.0);
                for (int i = 0; i < numConstraints; ++i) {
                    basicSolutionVector[columnIndices[i]] = basicVariablesVector(i);
                }
                return true;
            }
        }
    } while (std::next_permutation(columnIndices.begin(), columnIndices.end()));

    return false;
}

double SimplexSolver::getEpsilon(const Eigen::MatrixXd& A_B)
{
    return std::numeric_limits<double>::epsilon() * A_B.lpNorm<1>();
}
