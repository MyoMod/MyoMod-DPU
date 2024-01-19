#pragma once
#include "AnalysisAlgorithm.h"

namespace freesthetics {

class PassAlgorithm : public AnalysisAlgorithm {
public:
    PassAlgorithm(std::string_view name);
    virtual ~PassAlgorithm();
    Status run() override;
};

} // namespace freesthetics