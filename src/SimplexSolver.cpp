#include "../include/SimplexSolver.h"

bool SimplexSolver::solve(const LpProblem& problem, std::vector<double>& solution, double& optimalValue)
{
    Eigen::MatrixXd constraintMatrix = problem.getConstraints();
    Eigen::VectorXd rhsVector = problem.getRhs();
    Eigen::VectorXd costVector = problem.getObjectiveFunction();

    int numVariables = constraintMatrix.cols();
    int numConstraints = constraintMatrix.rows();

    std::vector<double> basicSolutionVector;
    if (!сomputeInitialVector(constraintMatrix, rhsVector, basicSolutionVector)) {
        std::cerr << "Failed to compute initial basic feasible solution.\n";
        return false;
    }

    Eigen::MatrixXd basisMatrix(numConstraints, numConstraints);
    std::vector<int> basisIndices(numConstraints), nonBasisIndices;

    for (int i = 0; i < numConstraints; ++i) {
        basisIndices[i] = i;
    }
    for (int i = numConstraints; i < numVariables; ++i) {
        nonBasisIndices.push_back(i);
    }

    while (true) {
        Eigen::VectorXd reducedCostVector = computeReducedCostVector(basisMatrix, constraintMatrix, costVector);

        if ((reducedCostVector.array() >= 0).all()) {
            solution = basicSolutionVector;
            optimalValue = costVector.dot(Eigen::VectorXd::Map(solution.data(), solution.size()));
            return true;
        }

        int enteringIndex = -1;
        double minReducedCost = std::numeric_limits<double>::max();
        for (int i = 0; i < reducedCostVector.size(); ++i) {
            if (reducedCostVector(i) < minReducedCost) {
                minReducedCost = reducedCostVector(i);
                enteringIndex = nonBasisIndices[i];
            }
        }

        Eigen::VectorXd enteringColumn = constraintMatrix.col(enteringIndex);
        Eigen::MatrixXd inverseBasisMatrix = basisMatrix.inverse();
        Eigen::VectorXd directionVector = computeUk(inverseBasisMatrix, enteringColumn);

        if ((directionVector.array() <= 0).all()) {
            std::cerr << "The linear program is unbounded.\n";
            return false;
        }

        if ((Eigen::VectorXd::Map(basicSolutionVector.data(), basicSolutionVector.size()).array() <= 0).any()) {
            if ((directionVector.array() > 0).any()) {
                Eigen::MatrixXd updatedBasisMatrix;
                if (!updateBasis(basisMatrix, constraintMatrix, enteringIndex, updatedBasisMatrix)) {
                    std::cerr << "Basis update failed.\n";
                    return false;
                }
                basisMatrix = updatedBasisMatrix;
                continue;
            }
        }

        double theta = computeTheta(Eigen::VectorXd::Map(basicSolutionVector.data(), basicSolutionVector.size()), directionVector);

        if (theta == std::numeric_limits<double>::infinity()) {
            std::cerr << "The linear program is unbounded.\n";
            return false;
        }

        Eigen::VectorXd currentSolution = Eigen::VectorXd::Map(basicSolutionVector.data(), basicSolutionVector.size());
        Eigen::VectorXd newSolution = computeNewBasicSolution(currentSolution, directionVector, theta);

        int leavingIndex = -1;
        for (int i = 0; i < directionVector.size(); ++i) {
            if (directionVector(i) > 0 && currentSolution(i) / directionVector(i) == theta) {
                leavingIndex = basisIndices[i];
                break;
            }
        }

        basisMatrix = computeNextBasisMatrix(basisMatrix, enteringIndex, leavingIndex, directionVector);

        for (int i = 0; i < basisIndices.size(); ++i) {
            if (basisIndices[i] == leavingIndex) {
                basisIndices[i] = enteringIndex;
                break;
            }
        }
        for (int i = 0; i < nonBasisIndices.size(); ++i) {
            if (nonBasisIndices[i] == enteringIndex) {
                nonBasisIndices[i] = leavingIndex;
                break;
            }
        }

        basicSolutionVector.assign(newSolution.data(), newSolution.data() + newSolution.size());
    }

    return false;
}

bool SimplexSolver::сomputeInitialVector(
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

Eigen::VectorXd SimplexSolver::computeReducedCostVector(
    const Eigen::MatrixXd& basisMatrix
    , const Eigen::MatrixXd& fullConstraintMatrix
    , const Eigen::VectorXd& costVector
) {
    Eigen::MatrixXd inverseBasisMatrix = basisMatrix.inverse();
    Eigen::VectorXd y_k_transpose = inverseBasisMatrix.transpose() * costVector;
    Eigen::VectorXd reducedCostVector = costVector - (y_k_transpose.transpose() * fullConstraintMatrix);

    return reducedCostVector;
}

void SimplexSolver::splitReducedCostVector(
    const Eigen::VectorXd& reducedCostVector
    , const std::vector<int>& basisIndices
    , const std::vector<int>& nonBasisIndices
    , Eigen::VectorXd& reducedCostVectorBasis
    , Eigen::VectorXd& reducedCostVectorNonBasis
) {
    int basisSize = basisIndices.size();
    int nonBasisSize = nonBasisIndices.size();

    reducedCostVectorBasis.resize(basisSize);
    reducedCostVectorNonBasis.resize(nonBasisSize);

    for (int i = 0; i < basisSize; ++i) {
        reducedCostVectorBasis(i) = reducedCostVector(basisIndices[i]);
    }

    for (int i = 0; i < nonBasisSize; ++i) {
        reducedCostVectorNonBasis(i) = reducedCostVector(nonBasisIndices[i]);
    }
}

Eigen::VectorXd SimplexSolver::computeUk(
    const Eigen::MatrixXd& inverseBasisMatrix
    , const Eigen::VectorXd& enteringColumn
) {
    Eigen::VectorXd ukBasis = inverseBasisMatrix * enteringColumn;
    return ukBasis;
}

double SimplexSolver::computeTheta(
    _In_ const Eigen::VectorXd& currentSolution
    , _In_ const Eigen::VectorXd& directionVector
) {
    double theta = std::numeric_limits<double>::infinity();

    for (int i = 0; i < directionVector.size(); ++i) {
        if (directionVector(i) > 0) {
            double candidateTheta = currentSolution(i) / directionVector(i);
            if (candidateTheta < theta) {
                theta = candidateTheta;
            }
        }
    }

    return theta;
}

bool SimplexSolver::updateBasis(
    const Eigen::MatrixXd& basisMatrix
    , const Eigen::MatrixXd& fullConstraintMatrix
    , int enteringIndex
    , Eigen::MatrixXd& updatedBasisMatrix
) {
    int numConstraints = basisMatrix.rows();
    int numVariables = fullConstraintMatrix.cols();

    for (int i = 0; i < numVariables; ++i) {
        if (i == enteringIndex) continue;

        Eigen::MatrixXd candidateBasisMatrix = basisMatrix;
        candidateBasisMatrix.col(i % numConstraints) = fullConstraintMatrix.col(i);

        if (std::abs(candidateBasisMatrix.determinant()) > std::numeric_limits<double>::epsilon() * candidateBasisMatrix.lpNorm<1>()) {
            updatedBasisMatrix = candidateBasisMatrix;
            return true;
        }
    }

    return false;
}

Eigen::VectorXd SimplexSolver::computeNewBasicSolution(
    _In_ const Eigen::VectorXd& currentSolution
    , _In_ const Eigen::VectorXd& directionVector
    , _In_ double stepSize
) {
    Eigen::VectorXd newSolution = currentSolution - stepSize * directionVector;
    return newSolution;
}

Eigen::MatrixXd SimplexSolver::computeNextBasisMatrix(
    _In_ const Eigen::MatrixXd& currentBasisMatrix
    , _In_ int enteringColumnIndex
    , _In_ int leavingColumnIndex
    , _In_ const Eigen::VectorXd& directionVector
) {
    int numConstraints = currentBasisMatrix.rows();

    Eigen::MatrixXd identityMatrix = Eigen::MatrixXd::Identity(numConstraints, numConstraints);
    for (int i = 0; i < numConstraints; ++i) {
        if (i == leavingColumnIndex) {
            identityMatrix(i, i) = 1.0 / directionVector(leavingColumnIndex);
        }
        else {
            identityMatrix(i, leavingColumnIndex) = -directionVector(i) / directionVector(leavingColumnIndex);
        }
    }

    Eigen::MatrixXd nextBasisMatrix = identityMatrix * currentBasisMatrix;
    return nextBasisMatrix;
}

double SimplexSolver::getEpsilon(const Eigen::MatrixXd& A_B)
{
    return std::numeric_limits<double>::epsilon() * A_B.lpNorm<1>();
}
