#include "FftAlgorithm.h"


#include <span>
#include <cassert>
#include "arm_math.h"
#include "fsl_gpt.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

#include "config.h"
#include "DirectControl.h"
#include "LinearRegression.h"

namespace freesthetics {

#define DEBUG_PIN_1(x) GPIO_PinWrite(BOARD_INITPINS_DEBUG1_GPIO, BOARD_INITPINS_DEBUG1_PIN, x)
#define DEBUG_PIN_2(x) GPIO_PinWrite(BOARD_INITPINS_DEBUG2_GPIO, BOARD_INITPINS_DEBUG2_PIN, x)
#define DEBUG_PIN_3(x) GPIO_PinWrite(BOARD_INITPINS_DEBUG3_GPIO, BOARD_INITPINS_DEBUG3_PIN, x)
#define DEBUG_PIN_4(x) GPIO_PinWrite(BOARD_INITPINS_DEBUG4_GPIO, BOARD_INITPINS_DEBUG4_PIN, x)
#define DEBUG_PINS(x) DEBUG_PIN_1(x&1); DEBUG_PIN_2((x>>1) & 1); DEBUG_PIN_3((x>>2) & 1); DEBUG_PIN_4((x>>3) & 1)

static DspType inputDebug[6];
static DspType outputDebug[6];
static DspType scaledOutputDebug[6];
static DspType preNormDebug[6];
static DspType maxInDebug[6];
static DspType minInDebug[6];
static DspType normAccLengthDebug[6];

FFTAlgorithm::FFTAlgorithm(std::string_view name) :
    AnalysisAlgorithm(name)
{
    gestureEstimator = new LinearRegression();
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
        /* C = A / B */

        /* Divide input and store result in destination buffer. */
        *pDst++ = (*pSrcA++) / (*pSrcB++);

        /* Decrement loop counter */
        blkCnt--;
    }
}

template <typename T>
inline void vectorReciprocal(T* pSrcA, T* pDst, size_t blkCnt)
{
    while (blkCnt > 0U)
    {
        /* C = 1 / A */

        /* Store result in destination buffer. */
        *pDst++ = 1.0f / (*pSrcA++);

        /* Decrement loop counter */
        blkCnt--;
    }
}

inline void vectorWeightedAddition(DspType* pSrcA, DspType* pSrcB, DspType* pDst, DspType weightA, DspType weightB, size_t blkCnt)
{
    while (blkCnt > 0U)
    {
        /* C = A * weightA + B * weightB */

        /* Add input and store result in destination buffer. */
        *pDst++ = (*pSrcA++) * weightA + (*pSrcB++) * weightB;

        /* Decrement loop counter */
        blkCnt--;
    }
}

Status FFTAlgorithm::recalculateNormFft(uint32_t channel, bool rescale) {
    if(normAccCounter[channel] > 0)
    {
        // update norm FFT
        float64_t normCoeff = 1.0 / normAccCounter[channel];
        arm_scale_f64(normAcc[channel], normCoeff, normTemp, subFftSize);
        arm_f64_to_float(normTemp, normFFT[channel], subFftSize);
        arm_clip_f32(normFFT[channel], normFFT[channel], 1e-15f, 10000.0f, subFftSize);

        // get reciprocal of normFFT so that it can be applied by multiplication
        vectorReciprocal(normFFT[channel], normFFT[channel], subFftSize);

        // Rescale normFFT, so that it is 1.0 at mvc input
        if(rescale)
        {
            DspType mvcResponse;
            arm_dot_prod_f32(CONFIG_MVC_FFT_PERSON1[channel].data(), normFFT[channel], subFftSize, &mvcResponse);
            // normFFT = normFFT / mvcResponse
            mvcResponse = 1.0 / mvcResponse;
            arm_scale_f32(normFFT[channel], mvcResponse, normFFT[channel], subFftSize);
        }
    }
    return Status::Ok;
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

    DEBUG_PINS(3);

    // proceed only if buffer is filled, as otherwise the FFT would contain zeros
    if(bufferFilled)
    {
        // apply window to data
        arm_mult_f32(dataMemory[channel], fftWindow, windowedData, samplesPerFFT);

        DEBUG_PINS(4);

        // calculate FFT
        arm_rfft_fast_f32(&fftInstance, windowedData, complexFft, 0);

        DEBUG_PINS(5);
        // calculate magnitude only for needed frequencies 
        // (this is a bandpass filter and saves computation time)
        // complexFft is packed as [real0, imag0, real1, imag1, ...]
        DspType* complexFftPtr = &complexFft[subFftStart * 2];
        arm_cmplx_mag_f32(complexFftPtr, subFft, subFftSize);

        DEBUG_PINS(6);

        float tNow = ((float) nCycle) / ((float) fCycle);
        bool modPressed = !GPIO_PinRead(BOARD_INITPINS_SW_MOD_GPIO, BOARD_INITPINS_SW_MOD_PIN);
        if( (normalisationStart <= tNow && tNow <= normalisationEnd) || modPressed)
        {
            arm_float_to_f64(subFft, normTemp, subFftSize);
            arm_add_f64(normAcc[channel], normTemp, normAcc[channel], subFftSize);
            normAccCounter[channel]++;
            
            // update norm FFT
            recalculateNormFft(channel, true);
        }
        DEBUG_PINS(7);

        // apply normalisation when normalisation and scale periods are over
        if(tNow > normalisationEnd)
        {
            // apply normalisation
            arm_dot_prod_f32(subFft, normFFT[channel], subFftSize, &result);
        }
        else
        {
            result = 0;
        }

        DEBUG_PINS(9);
    }
    else
    {   
        // if buffer is not filled, return 0
        result = 0;
    }

    return Status::Ok;
}

Status FFTAlgorithm::run() {
    const uint32_t activeChannels = 0b111111;
    std::array<DspType, numChannels> emgData;
    HandAxes axes;

    // iterate over all channels
    for (size_t channel = 0; channel < numChannels; channel++)
    {
        DEBUG_PINS(1);
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
        DEBUG_PINS(2);

        //process FFT
        processFftFilter(channel, inputBuffer, result);
        DEBUG_PINS(14);

        //clip result
        result = MIN(MAX(result, 0.0f), 1.0f);


        //write result to output
        std::span<uint8_t> pdsOutDisplayData;
        status = pdsOut->at(0)->getChannelData(channel, pdsOutDisplayData);
        assert(status == Status::Ok);
        pdsOutDisplayData[0] = ((activeChannels>>channel)&1) ? (uint8_t) (result * 100) : 0;

        emgData[channel] = result;
    }

    // update gesture estimator
    gestureEstimator->update(emgData);

    // update output
    gestureEstimator->getAxes(axes);


    // Write axes to output
    for (size_t channel = 0; channel < 6; channel++)
    {
        Status status;
        std::span<uint8_t> pdsOutBTData;
        status = pdsOut->at(1)->getChannelData(channel, pdsOutBTData);
        assert(status == Status::Ok);
        pdsOutBTData[0] = axes[channel];
    }

    // Write active gesture to output
    std::span<uint8_t> pdsOutDisplayData;
    Status status = pdsOut->at(0)->getChannelData(6, pdsOutDisplayData);
    assert(status == Status::Ok);
    pdsOutDisplayData[0] = 0;

    
    DEBUG_PINS(0);

    float tNow = ((float) nCycle) / ((float) fCycle);
    GPIO_PinWrite(BOARD_INITPINS_USR_LED_GPIO, BOARD_INITPINS_USR_LED_PIN, tNow < normalisationEnd);

    nCycle++;
    return Status::Ok;
}

} // namespace freesthetics
