#include "FftAlgorithm.h"


#include <span>
#include <cassert>
#include "arm_math.h"
#include "fsl_gpt.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

namespace freesthetics {

static DspType inputDebug[6];
static DspType outputDebug[6];
static DspType maxInDebug[6];
static DspType minInDebug[6];
static DspType normAccLengthDebug[6];

FFTAlgorithm::FFTAlgorithm(std::string_view name) :
    AnalysisAlgorithm(name),
    maxTracker(numChannels, MaxTracker(maxTrackerResolution, maxTrackerMemoryLength, F32_MIN, maxTrackerStartValue, fs, samplesPerCycle, maxTrackerOutlierCounter)),
    minTracker(numChannels, MinTracker(minTrackerResolution, minTrackerMemoryLength, F32_MAX, minTrackerStartValue, fs, samplesPerCycle, minTrackerOutlierCounter))
{
    arm_hanning_f32(fftWindow, samplesPerFFT);
    arm_rfft_fast_init_f32(&fftInstance, fftSize);
}

FFTAlgorithm::~FFTAlgorithm() {
}

template <typename T>
inline T map(T x, T in_min, T in_max, T out_min, T out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template <typename T>
inline void vectorDivide(T* pSrcA, T* pSrcB, T* pDst, size_t blkCnt)
{
    while (blkCnt > 0U)
    {
        /* C = A * B */

        /* Multiply input and store result in destination buffer. */
        *pDst++ = (*pSrcA++) * (*pSrcB++);

        /* Decrement loop counter */
        blkCnt--;
    }
}


Status FFTAlgorithm::processFftFilter(uint32_t channel, std::span<const DspType> pdsIn, DspType& result) {
    // shift dataMemory
    for (size_t i = 0; i < (samplesPerFFT - samplesPerCycle); i++)
    {
        dataMemory[channel][i] = dataMemory[channel][i + samplesPerCycle];
    }

    // fill dataMemory with new data
    for (size_t i = 0; i < samplesPerCycle; i++)
    {
        dataMemory[channel][i + (samplesPerFFT - samplesPerCycle)] = pdsIn[i];
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
        arm_rfft_fast_f32(&fftInstance, windowedData, complexFft, 0);
        // calculate magnitude only for needed frequencies 
        // (this is a bandpass filter and saves computation time)
        // complexFft is packed as [real0, imag0, real1, imag1, ...]
        DspType* complexFftPtr = &complexFft[subFftStart * 2];
        arm_cmplx_mag_f32(complexFftPtr, subFft, subFftSize);

        float tNow = ((float) nCycle) / ((float) fCycle);
        bool modPressed = !GPIO_PinRead(BOARD_INITPINS_SW_MOD_GPIO, BOARD_INITPINS_SW_MOD_PIN);
        if( (normalisationStart <= tNow && tNow <= normalisationEnd) || modPressed)
        {
            arm_float_to_f64(subFft, normTemp, subFftSize);
            arm_add_f64(normAcc[channel], normTemp, normAcc[channel], subFftSize);
            normAccCounter[channel]++;
            
            // update norm FFT
            float64_t normCoeff = 1.0 / normAccCounter[channel];
            arm_scale_f64(normAcc[channel], normCoeff, normTemp, subFftSize);
            arm_f64_to_float(normTemp, normFFT[channel], subFftSize);
            arm_clip_f32(normFFT[channel], normFFT[channel], 1e-15f, 10000.0f, subFftSize);

            // get reciprocal of normFFT so that it can be applied by multiplication
            uint32_t blockSize = subFftSize;
            float32_t* pSrcA = normFFT[channel];
            float32_t* pDst = normFFT[channel];
            while (blockSize > 0U)
            {
                *pDst++ = 1.0f / (*pSrcA++);
                blockSize--;
            }
        }

        // apply normalisation
        if(normAccCounter[channel] > 0)
        {
            // apply normalisation
            arm_mult_f32(subFft, normFFT[channel], subFft, subFftSize);
            arm_offset_f32(subFft, -1.0f, subFft, subFftSize);
        }

        arm_mean_f32(subFft, subFftSize, &result);
    }
    else
    {   
        // if buffer is not filled, return 0
        result = 0;
    }

    return Status::Ok;
}

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

        //convert input values to float
        for (size_t i = 0; i < samplesPerCycle; i++)
        {
            inputBuffer[i] = (((float32_t) pdsData[i]) / (1 << 23)) * 3.0f;
        }
        inputDebug[channel] = inputBuffer[0];

        //process FFT
        processFftFilter(channel, inputBuffer, result);
        
        // scale result
        outputDebug[channel] = result;

        //auto scale
        std::array<DspType, 1> values = {result};
        DspType maxIn = maxTracker[channel].update(values);
        DspType minIn = minTracker[channel].update(values);

        //debug
        maxInDebug[channel] = maxIn;
        minInDebug[channel] = minIn;
        normAccLengthDebug[channel] = normAccCounter[channel];

        result = map(result, minIn, maxIn, 0.0f, maxOut);
        result = MIN(MAX(result, 0), maxOut);

        //write result to output
        std::span<uint8_t> pdsOutData;
        status = pdsOut->at(0)->getChannelData(channel, pdsOutData);
        assert(status == Status::Ok);
        pdsOutData[channel] = result;
    }

    nCycle++;
    GPIO_PinWrite(BOARD_INITPINS_USR_LED_GPIO, BOARD_INITPINS_USR_LED_PIN, 0);
    return Status::Ok;
}

} // namespace freesthetics
