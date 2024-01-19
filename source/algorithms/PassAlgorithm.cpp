#include "PassAlgorithm.h"

#include <span>
#include <cassert>

namespace freesthetics {

PassAlgorithm::PassAlgorithm(std::string_view name) :
    AnalysisAlgorithm(name)
{
}

PassAlgorithm::~PassAlgorithm() {
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Status PassAlgorithm::run() {
    const float min = -0x0fffff00;
    const float max = 0x0fffff00;
    const float maxOut = 100;

    for (size_t channel = 0; channel < 6; channel++)
    {
        //get input values
        std::span<uint32_t> pdsData;
        Status status = pdsIn->at(0)->getChannelData(channel, pdsData);
        assert(status == Status::Ok);

        float inValue = pdsData[0];
        inValue -= min;
        float result = inValue * maxOut / (max - min);

        //bound result to minOut and maxOut
        result = result < 0 ? 0 : result;
        result = result > maxOut ? maxOut : result;

        //write result to output
        std::span<uint8_t> pdsOutData;
        status = pdsOut->at(0)->getChannelData(channel, pdsOutData);
        assert(status == Status::Ok);
        pdsOutData[0] = result;
    }

    return Status::Ok;
}

} // namespace freesthetics
