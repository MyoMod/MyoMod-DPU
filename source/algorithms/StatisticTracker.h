#pragma once
#include <span>
#include <vector>
#include <cassert>
#include "arm_math.h"

namespace freesthetics {

class StatisticTracker {
public:
    StatisticTracker(float timeResolution, float memoryLength, float32_t resetValue, float32_t startValue, float fs, uint32_t samplesPerCycle, uint32_t outlierCounter = 2);
    virtual ~StatisticTracker() = default;

    virtual float32_t statistic(std::span<const float32_t> data) = 0;

    float32_t update(std::span<const float32_t> data);
private:
    float32_t resetValue;
    uint32_t binDurationC; // bin duration in cycles
    uint32_t nBins;
    uint32_t cycleCounter;
    arm_sort_instance_f32 sortInstance;
    const uint32_t outlierCounter;
    
    std::vector<float32_t> bins;
    std::vector<float32_t> sortedBins;
};

class MaxTracker: public StatisticTracker {
private:
    /* data */
public:
    MaxTracker(float timeResolution, float memoryLength, float32_t resetValue, float32_t startValue, float fs, uint32_t samplesPerCycle, uint32_t outlierCounter = 2)
        : StatisticTracker{ 
            timeResolution,
            memoryLength,
            resetValue,
            startValue,
            fs,
            samplesPerCycle,
            outlierCounter}
    {};
    ~MaxTracker(){};

    float32_t statistic(std::span<const float32_t> data)
    {
        float32_t result;
        uint32_t pos;
        arm_max_f32(data.data(), data.size(), &result, &pos);
        return result;
    }
};

class MinTracker : public StatisticTracker
{
private:
    /* data */
public:
    MinTracker(float timeResolution, float memoryLength, float32_t resetValue, float32_t startValue, float fs, uint32_t samplesPerCycle, uint32_t outlierCounter = 2)
        : StatisticTracker{ 
            timeResolution,
            memoryLength,
            resetValue,
            startValue,
            fs,
            samplesPerCycle,
            outlierCounter}
    {};
    ~MinTracker(){};

    float32_t statistic(std::span<const float32_t> data)
    {
        float32_t result;
        uint32_t pos;
        arm_min_f32(data.data(), data.size(), &result, &pos);
        return result;
    }
};



} // namespace freesthetics
