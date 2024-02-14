#pragma once
#include "AnalysisAlgorithm.h"
#include <span>
#include <cassert>
#include "arm_math.h"

#include "StatisticTracker.h"

namespace freesthetics {

using DspType = float32_t;

class FFTAlgorithm : public AnalysisAlgorithm {
public:

    // Configuration
    static constexpr uint32_t samplesPerFFT = 256;
    static constexpr uint32_t fftSize = 256;
    static constexpr uint32_t numChannels = 6;
    static constexpr uint32_t samplesPerCycle = 15;
    static constexpr uint32_t fCycle = 100;

    static constexpr float fs = fCycle * samplesPerCycle;
    static constexpr float fMin = 50;
    static constexpr float fMax = 300;
    static constexpr bool clipToZero = true;

    static constexpr DspType maxOut = 100;
    static constexpr float normalisationStart = 3.0;
    static constexpr float normalisationEnd = 8.0;

    static constexpr float32_t maxTrackerResolution = 0.2;
    static constexpr float32_t maxTrackerMemoryLength = 15;
    static constexpr float32_t maxTrackerStartValue = 2;
    static constexpr float32_t maxTrackerOutlierCounter = 2;
    static constexpr float32_t minTrackerResolution = 0.2;
    static constexpr float32_t minTrackerMemoryLength = 15;
    static constexpr float32_t minTrackerStartValue = 2;
    static constexpr float32_t minTrackerOutlierCounter = 2;

    // Sub-FFT slice claculation
    static constexpr float fBinSize = fs / (fftSize); // (fs / 2) / (fftSize / 2)
    static constexpr uint32_t subFftStart = fMin / fBinSize;
    static constexpr uint32_t subFftEnd = fMax / fBinSize;
    static constexpr uint32_t subFftSize = subFftEnd - subFftStart;

    // FFT Buffers
    DspType fftWindow[samplesPerFFT];
    DspType windowedData[fftSize] = {0};
    DspType complexFft[fftSize] = {0};
    DspType subFft[subFftSize] = {0}; // this is real
    arm_rfft_fast_instance_f32 fftInstance;

    // Normalisation
    float64_t normAcc[numChannels][subFftSize] = {0};
    DspType normFFT[numChannels][subFftSize] = {1};
    uint32_t normAccCounter[numChannels] = {0};
    float64_t normTemp[subFftSize];

    // Data Memory
    DspType dataMemory[numChannels][fftSize] = {0};
    uint32_t elementsInMemory = 0;
    bool bufferFilled = false;

    // Misc variables
    uint32_t nCycle = 0;
    DspType inputBuffer[samplesPerCycle] = {0};

    // Auto Scale
    std::vector<MaxTracker> maxTracker;
    std::vector<MinTracker> minTracker;

    // methods
    Status processFftFilter(uint32_t channel, std::span<const DspType> pdsIn, DspType& pdsOut);
public:
    FFTAlgorithm(std::string_view name);
    virtual ~FFTAlgorithm();
    Status run() override;
};

} // namespace freesthetics
