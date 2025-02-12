/**
 * @file embeddedIMU.h
 * @author Leon Farchau (leon2225)
 * @brief A driver for a MAX11254 6Ch diff. ADC that is used for EMG measurement interfaced directly with the DPU via SPI.
 * @version 0.1
 * @date 11.02.2025
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

/* -------------------------------- includes -------------------------------- */
// System includes
#include <array>

// Includes from project
#include "charArray.h"
#include "embeddedDeviceNode.h"

/* ------------------------------- Constants ------------------------------- */
#define EMBEDDED_EMG_OVERSAMPLING 15

/* ---------------------------- Type Definitions ---------------------------- */

/**
 * @brief Driver for a MAX11254 6Ch diff. ADC interfaced directly with the DPU via SPI.
 * 
 */
class EmbeddedEMG : public EmbeddedDeviceNode
{
public:
    EmbeddedEMG(std::array<char, 10> id, uint8_t amplification);
    ~EmbeddedEMG() = default;

    void processInData() override;
    void processOutData() override;

    void sync() override;
    void enterRealTimeMode() override;
    void exitRealTimeMode() override;

private:
    std::array<std::shared_ptr<OutputPort<std::array<float, EMBEDDED_EMG_OVERSAMPLING>>>, 6> m_emgPorts;
    uint8_t m_amplification;
};