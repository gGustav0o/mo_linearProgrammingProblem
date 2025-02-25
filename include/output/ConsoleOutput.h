#pragma once

#include <iostream>
#include <vector>

#include "../interfaces/ILpOutput.h"

class ConsoleOutput : public ILpOutput
{
public:
    explicit ConsoleOutput() = default;
    ~ConsoleOutput() override = default;
    void outputSolution(_In_ const std::vector<double>& solution, _In_ double optimalValue) override;
    void outputNoSolution() override;
};