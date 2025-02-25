#pragma once

#include "../LpProblem.h"

class ILpInput
{
public:
    virtual ~ILpInput() = default;
    _Success_(return) virtual bool loadProblem(_Out_ LpProblem& problem) = 0;
};