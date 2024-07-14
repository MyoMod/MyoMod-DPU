#pragma once
#include "AnalysisAlgorithm.h"



class NopAlgorithm : public AnalysisAlgorithm {
public:
    NopAlgorithm(std::string_view name);
    virtual ~NopAlgorithm();
    Status run() override;
};

