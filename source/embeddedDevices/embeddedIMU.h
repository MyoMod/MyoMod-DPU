/**
 * @file embeddedIMU.h
 * @author Leon Farchau (leon2225)
 * @brief A driver for a ICM-42670-P 6-axis IMU interfaced directly with the DPU via SPI.
 * @version 0.1
 * @date 07.01.2025
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

/* ---------------------------- Type Definitions ---------------------------- */

/**
 * @brief Driver for a ICM-42670-P 6-axis IMU interfaced directly with the DPU via SPI.
 * 
 */
class EmbeddedIMU : public EmbeddedDeviceNode
{
public:
    EmbeddedIMU(std::array<char, 10> id);
    ~EmbeddedIMU() = default;

    void processInData() override;
    void processOutData() override;

    void sync() override;
    void enterRealTimeMode() override;
    void exitRealTimeMode() override;

private:
    std::array<std::shared_ptr<OutputPort<float>>, 3> m_accelPorts;
    std::array<std::shared_ptr<OutputPort<float>>, 3> m_gyroPorts;
};