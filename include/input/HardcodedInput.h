#pragma once

#include "../interfaces/ILpInput.h"

class HardcodedInput : public ILpInput {
public:
    HardcodedInput() = default;
    ~HardcodedInput() override = default;
    _Success_(return) bool loadProblem(_Out_ LpProblem& problem) override;
};