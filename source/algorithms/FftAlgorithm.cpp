#include "FftAlgorithm.h"


#include <span>
#include <cassert>
#include "arm_math.h"
#include "fsl_gpt.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

namespace freesthetics {

FFTAlgorithm::FFTAlgorithm(std::string_view name) :
    AnalysisAlgorithm(name)
{
    arm_hanning_f32(fftWindow, samplesPerFFT);
    arm_rfft_fast_init_f32(&fftInstance, fftSize);

    startTime = GPT_GetCurrentTimerCount(GPT1);
}

FFTAlgorithm::~FFTAlgorithm() {
}

template <typename T>
T map(T x, T in_min, T in_max, T out_min, T out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

DspType inputValues[6];
DspType outputValues[6];
DspType maxDebug;

Status FFTAlgorithm::run() {
    GPIO_PinWrite(BOARD_INITPINS_USR_LED_GPIO, BOARD_INITPINS_USR_LED_PIN, 1);

    // iterate over all channels
    for (size_t channel = 0; channel < numChannels; channel++)
    {
        DspType result = 0;

        //get input values
        std::span<int32_t> pdsData;
        Status status = pdsIn->at(0)->getChannelData(channel, pdsData);
        assert(status == Status::Ok);

        // shift dataMemory
        for (size_t i = 0; i < (samplesPerFFT - samplesPerCycle); i++)
        {
            dataMemory[channel][i] = dataMemory[channel][i + samplesPerCycle];
        }

        // fill dataMemory with new data
        for (size_t i = 0; i < samplesPerCycle; i++)
        {
            float transformedValue = ((DspType)pdsData[i] / (1 << 23)) * 3.0;
            //transformedValue = pdsData[i];
            dataMemory[channel][i + (samplesPerFFT - samplesPerCycle)] = transformedValue;
            inputValues[channel] = transformedValue;
        }
        if(channel == (numChannels - 1) && !bufferFilled)
        {
            elementsInMemory += samplesPerCycle;
            bufferFilled = elementsInMemory >= samplesPerFFT;
        }

        // proceed only if buffer is filled, as otherwise the FFT would contain zeros
        if(bufferFilled)
        {
            // apply window to data
            arm_mult_f32(dataMemory[channel], fftWindow, windowedData, samplesPerFFT);

            // calculate FFT
            arm_rfft_fast_f32(&fftInstance, windowedData, fft, 0);
            // convert complex to real (fft now has length fftSize/2)
            arm_cmplx_mag_f32(fft, realFft, fftSize/2);

            float time = ((float) nCycle) / (fs / samplesPerCycle);
            if( normalisationStart <= time && time <= normalisationEnd)
            {
                if(!normalisationFFTInitialised)
                {
                    arm_copy_f32(subFft, normalisationFFT, subFftSize);
                    normalisationFFTInitialised = true;
                }

                // normalise FFT
                for (size_t i = 0; i < subFftSize; i++)
                {
                    normalisationFFT[i] = normalisationFFT[i] * normalisationAlpha + subFft[i] * (1 - normalisationAlpha);
                }
                arm_clip_f32(normalisationFFT, normalisationFFT, 1e-8, 10000, subFftSize);
            }

            // apply normalisation
            for (size_t i = 0; i < subFftSize; i++)
            {
                subFft[i] = (subFft[i] / normalisationFFT[i]) - 1;
            }

            arm_mean_f32(subFft, subFftSize, &result);
        }

        outputValues[channel] = result;

        //auto scale
        DspType diff = result - maxResult[channel];
        maxResult[channel] *= (diff > 0) ? 1.05 : 0.999;
        maxDebug = maxResult[0];
        result = map(result, 0.0f, 35.0f, 0.0f, maxOut);

        //bound result to minOut and maxOut
        result = result < 0 ? 0 : result;
        result = result > maxOut ? maxOut : result;

        //write result to output
        std::span<uint8_t> pdsOutData;
        status = pdsOut->at(0)->getChannelData(channel, pdsOutData);
        assert(status == Status::Ok);
        pdsOutData[channel] = result;
        //outputValues[channel] = result;
    }

    nCycle++;
    GPIO_PinWrite(BOARD_INITPINS_USR_LED_GPIO, BOARD_INITPINS_USR_LED_PIN, 0);
    return Status::Ok;
}

} // namespace freesthetics
