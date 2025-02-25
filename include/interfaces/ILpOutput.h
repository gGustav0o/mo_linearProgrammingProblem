#pragma once

#include <vector>

class ILpOutput
{
public:
    virtual ~ILpOutput() = default;
    virtual void outputSolution(_In_ const std::vector<double>& solution, _In_ double optimalValue) = 0;
    virtual void outputNoSolution() = 0;
};