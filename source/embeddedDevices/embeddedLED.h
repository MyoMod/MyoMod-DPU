/**
 * @file embeddedLEd.h
 * @author Leon Farchau (leon2225)
 * @brief A driver for the LED embedded in the bracelet
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
 * @brief Driver for the RGB LED on the bracelet pcb that is connected via PWM.
 * 
 */
class EmbeddedLED : public EmbeddedDeviceNode
{
public:
    EmbeddedLED(std::array<char, 10> id);
    ~EmbeddedLED() = default;

    void processInData() override;
    void processOutData() override;

    void sync() override;
    void enterRealTimeMode() override;
    void exitRealTimeMode() override;

private:
    std::array<std::shared_ptr<InputPort<float>>, 3> m_colorPorts; // RGB
};