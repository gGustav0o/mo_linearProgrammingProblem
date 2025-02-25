#include "../include/LpProblem.h"

LpProblem::LpProblem
(
	const Eigen::VectorXd& objectiveFunction
	, const Eigen::MatrixXd& constraints
	, const Eigen::VectorXd& rhs
	, const std::vector<ConstraintType>& constraintTypes
)
	: objectiveFunction_(objectiveFunction)
	, constraints_(constraints)
	, rhs_(rhs)
	, constraintTypes_(constraintTypes)
	, isInitialized_(true)
{
}

void LpProblem::init
(
	const Eigen::VectorXd& objectiveFunction
	, const Eigen::MatrixXd& constraints
	, const Eigen::VectorXd& rhs
	, const std::vector<ConstraintType>& constraintTypes
)
{
	if (isInitialized_) {
		throw std::logic_error("LpProblem has already been initialized.");
	}

	objectiveFunction_ = objectiveFunction;
	constraints_ = constraints;
	rhs_ = rhs;
	constraintTypes_ = constraintTypes;
	isInitialized_ = true;
}

LpProblem& LpProblem::convertToCanonical()
{
	return *this;
}

void LpProblem::recalculateType()
{

}
