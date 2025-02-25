#include "../include/LpProblem.h"

LpProblem::LpProblem
(
	std::function<double(const Eigen::VectorXd&)> objectiveFunction
	, const Eigen::MatrixXd& constraints
	, const Eigen::VectorXd& rhs
	, const std::vector<ConstraintType>& constraintTypes
)
	: objectiveFunction_(std::move(objectiveFunction))
	, constraints_(constraints)
	, rhs_(rhs)
	, constraintTypes_(constraintTypes)
	, isInitialized_(true)
{
}

void LpProblem::init
(
	std::function<double(const Eigen::VectorXd&)> objectiveFunction
	, const Eigen::MatrixXd& constraints
	, const Eigen::VectorXd& rhs
	, const std::vector<ConstraintType>& constraintTypes
)
{
	if (isInitialized_) {
		throw std::logic_error("LpProblem has already been initialized.");
	}

	objectiveFunction_ = std::move(objectiveFunction);
	constraints_ = constraints;
	rhs_ = rhs;
	constraintTypes_ = constraintTypes;
	isInitialized_ = true;
}

std::function<double(double)> LpProblem::getObjectiveFunction() const
{
	return std::function<double(double)>();
}

LpProblem& LpProblem::convertToCanonical()
{
	return *this;
}

void LpProblem::recalculateType()
{

}
