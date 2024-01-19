#include "NopAlgorithm.h"

#include <span>
#include <cassert>

namespace freesthetics {

NopAlgorithm::NopAlgorithm(std::string_view name) :
    AnalysisAlgorithm(name)
{
}

NopAlgorithm::~NopAlgorithm() {
}

Status NopAlgorithm::run() {
    return Status::Ok;
}

} // namespace freesthetics
