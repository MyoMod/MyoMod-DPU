#pragma once
#include "AnalysisAlgorithm.h"
#include <span>
#include <cassert>
#include "arm_math.h"

namespace freesthetics {

using DspType = float32_t;

class FFTAlgorithm : public AnalysisAlgorithm {
public:
    static constexpr uint32_t numChannels = 6;
    static constexpr DspType maxOut = 100;

    static constexpr float normalisationStart = 0.5;
    static constexpr float normalisationEnd = 2.5;
    static constexpr float normalisationAlpha = 0.98;

    static constexpr uint32_t samplesPerCycle = 15;
    static constexpr uint32_t samplesPerFFT = 256;
    static constexpr uint32_t fftSize = 256;

    static constexpr float fs = 100 * samplesPerCycle;
    static constexpr float fMin = 50;
    static constexpr float fMax = 300;

    DspType fftWindow[samplesPerFFT];

    static constexpr bool clipToZero = true;
    
    static constexpr float fBinSize = fs / (fftSize); // (fs / 2) / (fftSize / 2)

    // The sub-FFT is the FFT of the frequency range we are interested in (bandpassed) 
    static constexpr uint32_t subFftStart = fMin / fBinSize;
    static constexpr uint32_t subFftEnd = fMax / fBinSize;
    static constexpr uint32_t subFftSize = subFftEnd - subFftStart;

    DspType normalisationFFT[subFftSize] = {1};
    bool normalisationFFTInitialised = false;
    DspType dataMemory[numChannels][fftSize] = {0};
    uint32_t elementsInMemory = 0;
    bool bufferFilled = false;
    DspType windowedData[fftSize] = {0};
    DspType fft[fftSize+10] = {0};
    DspType realFft[fftSize/2+10] = {0};
    DspType *subFft = &realFft[subFftStart];

    DspType filteredFft[subFftSize] = {0};

    // dsp instances
    arm_rfft_fast_instance_f32 fftInstance;

    uint32_t startTime = 0;
    DspType maxResult[numChannels] = {1};

    uint32_t nCycle = 0;
public:
    FFTAlgorithm(std::string_view name);
    virtual ~FFTAlgorithm();
    Status run() override;
};

} // namespace freesthetics
