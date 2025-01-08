/**
 * @file virtualDeviceNode.cpp
 * @author Leon Farchau (leon2225)
 * @brief 
 * @version 0.1
 * @date 07.01.2025
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/* -------------------------------- Includes -------------------------------- */
#include "embeddedDeviceNode.h"

/* ----------------------------- Implementation ----------------------------- */

/**
 * @brief Returns the DeviceIdentifier of the EmbeddedDeviceNode
 * 
 * @return const DeviceIdentifier 
 */
const DeviceIdentifier EmbeddedDeviceNode::getDeviceIdentifier() const
{
    DeviceIdentifier identifier;
    identifier.type = m_type;
    identifier.id = m_id;
    return identifier;
}