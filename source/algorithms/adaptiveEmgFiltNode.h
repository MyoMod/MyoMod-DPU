/**
 * @file adaptiveEmgFiltNode.h
 * @author Leon Farchau (leon2225)
 * @brief This Node implements an adaptive filter for EMG signals
 *          based on a automatically generated filterfunction
 * @version 0.1
 * @date 25.02.2025
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

/* -------------------------------- Includes -------------------------------- */
// System includes
#include <array>
#include "math.h"
#include "arm_math.h"

// Project includes
#include "Status.h"
#include "algorithmicNode.h"

/* ---------------------------- Type Definitions ---------------------------- */
using DspType = float32_t;

/* ---------------------------- Class Definition ---------------------------- */
/**
 * @brief This Node implements an adaptive filter for EMG signals
 *          based on a automatically generated filterfunction
 * 
 */
class AdaptiveEmgFiltNode : public AlgorithmicNode
{
protected:
    /* ------------------------------ Configuration ----------------------------- */
    static constexpr uint32_t m_SAMPLES_PER_FFT = 512;
    static constexpr uint32_t m_FFT_SIZE = 512;
    static constexpr uint32_t m_NUM_CHANNELS = 6;
    static constexpr uint32_t m_SAMPLES_PER_CYCLE = 15;
    static constexpr uint32_t m_F_CYCLE = 100;

    static constexpr float m_FS = m_F_CYCLE * m_SAMPLES_PER_CYCLE;
    static constexpr float m_F_MIN = 60;
    static constexpr float m_F_MAX = 300;
    static constexpr bool m_CLIP_TO_ZERO = true;

    static constexpr DspType m_MAX_OUT = 100;
    static constexpr float m_NORMILISATION_START = 3.0;
    static constexpr float m_NORMILISATION_END = 5.0;
    /* ---------------------------- Configuration End --------------------------- */

    // Sub-FFT slice calculation
    static constexpr float m_F_BIN_SIZE = m_FS / (m_FFT_SIZE); // (fs / 2) / (fftSize / 2)
    static constexpr uint32_t m_SUB_FFT_START = m_F_MIN / m_F_BIN_SIZE;
    static constexpr uint32_t m_SUB_FFT_END = m_F_MAX / m_F_BIN_SIZE;
    static constexpr uint32_t m_SUB_FFT_SIZE = m_SUB_FFT_END - m_SUB_FFT_START;

    // FFT Buffers
    DspType m_fftWindow[m_SAMPLES_PER_FFT];
    DspType m_windowedData[m_FFT_SIZE] = {0};
    DspType m_complexFft[m_FFT_SIZE] = {0};
    DspType m_subFft[m_SUB_FFT_SIZE] = {0}; // this is real
    arm_rfft_fast_instance_f32 m_fftInstance;

    // Normalisation
    float64_t m_normAcc[m_NUM_CHANNELS][m_SUB_FFT_SIZE] = {0};
    DspType m_normFFT[m_NUM_CHANNELS][m_SUB_FFT_SIZE] = {1};
    uint32_t m_normAccCounter[m_NUM_CHANNELS] = {0};
    float64_t m_normTemp[m_SUB_FFT_SIZE];

    // Data Memory
    DspType m_dataMemory[m_NUM_CHANNELS][m_FFT_SIZE] = {0};
    uint32_t m_elementsInMemory = 0;
    bool m_bufferFilled = false;

    // Misc variables
    uint32_t m_nCycle = 0;
    DspType m_inputBuffer[m_SAMPLES_PER_CYCLE] = {0};

    std::array<std::shared_ptr<InputPort<std::array<float, 15>>>, 6> m_rawEmgInputPorts;
    std::array<std::shared_ptr<OutputPort<float>>, 6> m_filteredEmgOutputPorts;
    std::shared_ptr<OutputPort<float>> m_filterProgressOutputPort;

    // methods
    Status processFftFilter(uint32_t channel, std::span<const DspType> pdsIn, DspType& pdsOut);
    Status recalculateNormFft(uint32_t channel, bool rescale);
public:
    AdaptiveEmgFiltNode();
    ~AdaptiveEmgFiltNode() = default;

    void process() override;
};