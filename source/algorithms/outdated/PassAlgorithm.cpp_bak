#include "PassAlgorithm.h"

#include <span>
#include <cassert>



PassAlgorithm::PassAlgorithm(std::string_view name) :
    AnalysisAlgorithm(name)
{
}

PassAlgorithm::~PassAlgorithm() {
}


int32_t debugData[6];
uint8_t buttonState;
uint8_t debugDataOut[6];
float min = 0;
float max = 0;

Status PassAlgorithm::run() {
    const float maxOut = 100;

    for (size_t channel = 0; channel < 6; channel++)
    {
        //get input values
        std::span<int32_t> pdsData;
        Status status = pdsIn->at(1)->getChannelData(channel, pdsData);
        assert(status == Status::Ok);

        float inValue = pdsData[0];
        min = min > inValue ? inValue : min;
        max = max < inValue ? inValue : max;
        debugData[channel] = pdsData[0];
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
        debugDataOut[channel] = result;
    }

    return Status::Ok;
}


