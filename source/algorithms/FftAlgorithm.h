#pragma once
#include "AnalysisAlgorithm.h"
#include <span>
#include <cassert>
#include "arm_math.h"

#include "StatisticTracker.h"
#include "GestureEstimator.h"

namespace freesthetics {

using DspType = float32_t;

class FFTAlgorithm : public AnalysisAlgorithm {
public:

    // Configuration
    static constexpr uint32_t samplesPerFFT = 512;
    static constexpr uint32_t fftSize = 512;
    static constexpr uint32_t numChannels = 6;
    static constexpr uint32_t samplesPerCycle = 15;
    static constexpr uint32_t fCycle = 100;

    static constexpr float fs = fCycle * samplesPerCycle;
    static constexpr float fMin = 60;
    static constexpr float fMax = 300;
    static constexpr bool clipToZero = true;

    static constexpr DspType maxOut = 100;
    static constexpr float normalisationStart = 3.0;
    static constexpr float normalisationEnd = 5.0;

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

    // Gesture Estimation
    GestureEstimator *gestureEstimator;

    // methods
    Status processFftFilter(uint32_t channel, std::span<const DspType> pdsIn, DspType& pdsOut);
    Status recalculateNormFft(uint32_t channel, bool rescale);
public:
    FFTAlgorithm(std::string_view name);
    virtual ~FFTAlgorithm();
    Status run() override;
};

} // namespace freesthetics
