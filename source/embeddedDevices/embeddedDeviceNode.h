/**
 * @file virutalDeviceNode.h
 * @author Leon Farchau (leon2225)
 * @brief 
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
#include "node.h"

/* ------------------------------- Constants ------------------------------- */

/* ---------------------------- Type Definitions ---------------------------- */

/**
 * @brief Base class for devices that are embedded in the DPU and
 *          are not connected via the MyoMod interface.
 * 
 */
class EmbeddedDeviceNode : public BaseNode
{
public:
    EmbeddedDeviceNode(std::array<char, 10> id, const std::array<char, 10> type) :
        m_id{id},
        m_type{type},
        m_activeBuffer{false}
    {}

    virtual void processInData() = 0;
    virtual void processOutData() = 0;
    const DeviceIdentifier getDeviceIdentifier() const;

    virtual void sync() = 0;
    virtual void enterRealTimeMode() = 0;
    virtual void exitRealTimeMode() = 0;
private:
    std::array<char, 10> m_id;
    const std::array<char, 10> m_type;

    bool m_activeBuffer; // active ping-pong buffer for output
};