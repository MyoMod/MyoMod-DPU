#include "deviceNode.h"
#include <cstdint>
#include "SEGGER_RTT.h"



/**
 * @brief Returns true if m_initialized flag is set, 0 otherwise, returns RegisterNotUpdated if the register has not been updated yet
 * 
 * @param m_initialized 
 * @return Status 
 */
Status DeviceNode::getDeviceInitialized(bool* m_initialized) {
	// TODO: Implement getDeviceNodeInitialized method
	return Status::Error;
}

/**
 * @brief Shedules a write to the m_initialized flag of the DeviceNode
 * 
 * @param m_initialized 
 * @return Status 
 */
Status DeviceNode::setDeviceInitialized(bool m_initialized) {
	// TODO: Implement setDeviceNodeInitialized method
	return Status::Error;
}


/**
 * @brief Returns the rawdata of the register at the given index
 * 
 * @param registerIndex 
 * @param value 
 * @return Status 
 */
Status DeviceNode::getRegisterRawData(uint32_t registerIndex, void* value) {
	// TODO: Implement getRegisterRawData method
	return Status::Error;
}

/**
 * @brief Sets the rawdata of the register at the given index
 * 
 * @param registerIndex 
 * @param value 
 * @return Status 
 */
Status DeviceNode::setRegisterRawData(uint32_t registerIndex, void* value) {
	// TODO: Implement setRegisterRawData method
	return Status::Error;
}

/**
 * @brief Returns the DeviceIdentifier of the DeviceNode
 * 
 * @return DeviceIdentifier 
 */
DeviceIdentifier const DeviceNode::getDeviceIdentifier()
{
    DeviceIdentifier identifier;
    identifier.type = m_type;
    identifier.id = m_id;
    return identifier;
}


