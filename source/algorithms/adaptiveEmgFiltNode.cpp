/**
 * @file adaptiveEmgFiltNode.cpp
 * @author Leon Farchau (leon2225)
 * @brief 
 * @version 0.1
 * @date 25.02.2025
 * 
 * @copyright Copyright (c) 2025
 * 
 */
/* -------------------------------- Includes -------------------------------- */
#include "adaptiveEmgFiltNode.h"
#include "emgFilt_config.h"
#include "dpu_gpio.h"

/* ---------------------------- Static Variables ---------------------------- */ 
[[maybe_unused]] static DspType inputDebug[6];
[[maybe_unused]] static DspType outputDebug[6];
[[maybe_unused]] static DspType scaledOutputDebug[6];
[[maybe_unused]] static DspType preNormDebug[6];
[[maybe_unused]] static DspType maxInDebug[6];
[[maybe_unused]] static DspType minInDebug[6];
[[maybe_unused]] static DspType normAccLengthDebug[6];

/* ---------------------- Private Function Declarations --------------------- */
template <typename T>
inline T map(T x, T in_min, T in_max, T out_min, T out_max);
template <typename T>
inline void vectorDivide(T* pSrcA, T* pSrcB, T* pDst, size_t blkCnt);
template <typename T>
inline void vectorReciprocal(T* pSrcA, T* pDst, size_t blkCnt);
inline void vectorWeightedAddition(DspType* pSrcA, DspType* pSrcB, DspType* pDst, 
                                    DspType weightA, DspType weightB, size_t blkCnt);

/* ----------------------------- Implementation ----------------------------- */
Status AdaptiveEmgFiltNode::recalculateNormFft(uint32_t channel, bool rescale) {
    assert(channel < m_NUM_CHANNELS);
    if(m_normAccCounter[channel] > 0)
    {
        // update norm FFT
        float64_t normCoeff = 1.0 / m_normAccCounter[channel];
        arm_scale_f64(m_normAcc[channel], normCoeff, m_normTemp, m_SUB_FFT_SIZE);
        arm_f64_to_float(m_normTemp, m_normFFT[channel], m_SUB_FFT_SIZE);
        arm_clip_f32(m_normFFT[channel], m_normFFT[channel], 1e-15f, 10000.0f, m_SUB_FFT_SIZE);

        // get reciprocal of m_normFFT so that it can be applied by multiplication
        vectorReciprocal(m_normFFT[channel], m_normFFT[channel], m_SUB_FFT_SIZE);

        // Rescale m_normFFT, so that it is 1.0 at mvc input
        if(rescale)
        {
            DspType mvcResponse;
            arm_dot_prod_f32(CONFIG_MVC_FFT_PERSON1[channel].data(), m_normFFT[channel], m_SUB_FFT_SIZE, &mvcResponse);
            // m_normFFT = m_normFFT / mvcResponse
            mvcResponse = 1.0f / mvcResponse;
            arm_scale_f32(m_normFFT[channel], mvcResponse, m_normFFT[channel], m_SUB_FFT_SIZE);
        }
    }
    return Status::Ok;
}

Status AdaptiveEmgFiltNode::processFftFilter(uint32_t channel, std::span<const DspType> pdsIn, DspType& result) {
    assert(channel < m_NUM_CHANNELS);
    assert(pdsIn.size() == m_SAMPLES_PER_CYCLE);

    // shift m_dataMemory
    for (size_t i = 0; i < (m_SAMPLES_PER_FFT - m_SAMPLES_PER_CYCLE); i++)
    {
        m_dataMemory[channel][i] = m_dataMemory[channel][i + m_SAMPLES_PER_CYCLE];
    }

    inputDebug[channel] = m_dataMemory[channel][0];

    // fill m_dataMemory with new data
    for (size_t i = 0; i < m_SAMPLES_PER_CYCLE; i++)
    {
        m_dataMemory[channel][i + (m_SAMPLES_PER_FFT - m_SAMPLES_PER_CYCLE)] = pdsIn[i];
    }
    if(channel == (m_NUM_CHANNELS - 1) && !m_bufferFilled)
    {
        m_elementsInMemory += m_SAMPLES_PER_CYCLE;
        m_bufferFilled = m_elementsInMemory >= m_SAMPLES_PER_FFT;
    }

    debugSetValue(2);

    // proceed only if buffer is filled, as otherwise the FFT would contain zeros
    if(m_bufferFilled)
    {
        // apply window to data
        arm_mult_f32(m_dataMemory[channel], m_fftWindow, m_windowedData, m_SAMPLES_PER_FFT);

        debugSetValue(3);

        // calculate FFT
        arm_rfft_fast_f32(&m_fftInstance, m_windowedData, m_complexFft, 0);

        debugSetValue(4);
        // calculate magnitude only for needed frequencies 
        // (this is a bandpass filter and saves computation time)
        // complexFft is packed as [real0, imag0, real1, imag1, ...]
        DspType* complexFftPtr = &m_complexFft[m_SUB_FFT_START * 2];
        arm_cmplx_mag_f32(complexFftPtr, m_subFft, m_SUB_FFT_SIZE);

        debugSetValue(5);

        float tNow = ((float) m_nCycle) / ((float) m_F_CYCLE);
        //bool adaptFilter = !GPIO_PinRead(BOARD_INITPINS_SW_MOD_GPIO, BOARD_INITPINS_SW_MOD_PIN);
        bool adaptFilter = false;
        if( (m_NORMILISATION_START <= tNow && tNow <= m_NORMILISATION_END) || adaptFilter)
        {
            arm_float_to_f64(m_subFft, m_normTemp, m_SUB_FFT_SIZE);
            arm_add_f64(m_normAcc[channel], m_normTemp, m_normAcc[channel], m_SUB_FFT_SIZE);
            m_normAccCounter[channel]++;
            
            // update norm FFT
            recalculateNormFft(channel, true);
        }
        debugSetValue(6);

        // apply normalisation when normalisation and scale periods are over
        if(tNow > m_NORMILISATION_END)
        {
            // apply normalisation
            arm_dot_prod_f32(m_subFft, m_normFFT[channel], m_SUB_FFT_SIZE, &result);
        }
        else
        {
            result = 0;
        }

        debugSetValue(7);
    }
    else
    {   
        // if buffer is not filled, return 0
        result = 0;
    }

    return Status::Ok;
}

AdaptiveEmgFiltNode::AdaptiveEmgFiltNode()
: AlgorithmicNode()
{
    // Initialize FFT window
    arm_rfft_fast_init_f32(&m_fftInstance, m_FFT_SIZE);
    arm_hanning_f32(m_fftWindow, m_SAMPLES_PER_FFT);

    // Initialize input and output ports
    for (size_t i = 0; i < m_NUM_CHANNELS; i++)
    {
        m_rawEmgInputPorts[i] = std::make_shared<InputPort<std::array<float, 15>>>();
        m_filteredEmgOutputPorts[i] = std::make_shared<OutputPort<float>>();

        m_inputPorts.push_back(m_rawEmgInputPorts[i]);
        m_outputPorts.push_back(m_filteredEmgOutputPorts[i]);
    }
    m_filterProgressOutputPort = std::make_shared<OutputPort<float>>();
    m_outputPorts.push_back(m_filterProgressOutputPort);
}

void AdaptiveEmgFiltNode::process()
{
    //check if all inputs are valid
    bool allValid = true;
    for (const auto& emgInput : m_rawEmgInputPorts)
    {
        allValid &= emgInput->isValid();
    }

    if(!allValid)
    {
        // As the inputs are invalid, the output is invalid as well
        for (const auto& emgOutput : m_filteredEmgOutputPorts)
        {
            emgOutput->setValid(false);
        }
        m_filterProgressOutputPort->setValid(false);
        return;
    }

    // Process data
    for (size_t i = 0; i < m_NUM_CHANNELS; i++)
    {
        // Convert input to DspType
        std::array<DspType, 15> rawEMG = m_rawEmgInputPorts[i]->getValue();

        // Process FFT filter
        DspType filteredEMG;
        processFftFilter(i, rawEMG, filteredEMG);

        // Store result
        outputDebug[i] = filteredEMG;
        m_filteredEmgOutputPorts[i]->setValid(true);
        m_filteredEmgOutputPorts[i]->setValue(filteredEMG);
    }

    // Set progress output
    m_filterProgressOutputPort->setValid(true);
    m_filterProgressOutputPort->setValue(0.0f);
    
    m_nCycle++;
}

/* ---------------------------- Private functions --------------------------- */
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