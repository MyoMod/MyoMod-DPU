#pragma once
#include "AnalysisAlgorithm.h"

namespace freesthetics {

class NopAlgorithm : public AnalysisAlgorithm {
public:
    NopAlgorithm(std::string_view name);
    virtual ~NopAlgorithm();
    Status run() override;
};

} // namespace freesthetics