#pragma once

#include <vector>
#include <functional>

#include <Eigen/Dense>

class LpProblem
{
public:
    enum class ConstraintType {
        LESS_OR_EQUAL
        , GREATER_OR_EQUAL
        , EQUAL
    };
    enum class LpType {
        CANONICAL
        , SYMMETRIC
        , GENERAL
    };

    LpProblem() = default;

    explicit LpProblem(
        _In_ const Eigen::VectorXd& objectiveFunction
        , _In_ const Eigen::MatrixXd& constraints
        , _In_ const Eigen::VectorXd& rhs
        , _In_ const std::vector<ConstraintType>& constraintTypes
    );

    void init(
        _In_ const Eigen::VectorXd& objectiveFunction
        , _In_ const Eigen::MatrixXd& constraints
        , _In_ const Eigen::VectorXd& rhs
        , _In_ const std::vector<ConstraintType>& constraintTypes
    );
    Eigen::VectorXd getObjectiveFunction() const { return objectiveFunction_; };
    Eigen::MatrixXd getConstraints() const { return constraints_; };
    Eigen::VectorXd getRhs() const { return rhs_; };
    std::vector<ConstraintType> getConstraintTypes() const { return constraintTypes_; };
    LpType getProblemType() const { return problemType_; };
    LpProblem& convertToCanonical();
private:
    Eigen::VectorXd objectiveFunction_{};
    Eigen::MatrixXd constraints_{};
    Eigen::VectorXd rhs_{};
    std::vector<ConstraintType> constraintTypes_{};
    LpType problemType_{LpType::GENERAL};
    bool isInitialized_ = false;

    void recalculateType();
};

