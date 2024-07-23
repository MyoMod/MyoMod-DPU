#include "deviceNode.h"
#include <cstdint>
#include "SEGGER_RTT.h"



/**
 * @brief Returns the DeviceIdentifier of the DeviceNode
 * 
 * @return DeviceIdentifier 
 */
const DeviceIdentifier DeviceNode::getDeviceIdentifier() const
{
    DeviceIdentifier identifier;
    identifier.type = m_type;
    identifier.id = m_id;
    return identifier;
}

/**
 * @brief Returns the raw data of the register specified by registerType
 * 
 * @note This only returns the common registers, the device specific registers are handled by the derived classes
 * 
 * @param registerType 		The type of the register to get the raw data from
 * @param status 			The status of the operation
 * 								- Status::Ok if the operation was successful
 *                              - Status::RegisterNotSupported if this device doesn't implement the register
 * @return std::span<const std::byte> 
 */
std::span<const std::byte> 
DeviceNode::getRegisterRawData (DeviceRegisterType registerType, Status& status) const
{
	status = Status::Ok;
	switch (registerType)
	{
	case DeviceRegisterType::Status:
		return std::span<const std::byte>(
			reinterpret_cast<const std::byte*>(&m_statusByte), 
			sizeof(m_statusByte));
	case DeviceRegisterType::CommonDeviceStatus:
		return std::span<const std::byte>(
			reinterpret_cast<const std::byte*>(&m_commonDeviceStatus), 
			sizeof(m_commonDeviceStatus));
	case DeviceRegisterType::CommonDeviceInformation:
		return std::span<const std::byte>(
			reinterpret_cast<const std::byte*>(&m_commonDeviceInformation), 
			sizeof(m_commonDeviceInformation));
	case DeviceRegisterType::CommonDeviceConfiguration:
		return std::span<const std::byte>(
			reinterpret_cast<const std::byte*>(&m_commonDeviceConfiguration), 
			sizeof(m_commonDeviceConfiguration));
	default:
		status = Status::RegisterNotImplemented;
		return {};
	}
}

/**
 * @brief Sets the raw data of the register specified by registerType
 * 
 * @note This only sets the common registers, the device specific registers are handled by the derived classes
 * 
 * @param registerType 		The type of the register to set the raw data to
 * @param value 			The raw data to set
 * @return  				The status of the operation
 * 								- Status::Ok if the operation was successful
 *                              - Status::RegisterNotSupported if this device doesn't implement the register
 */
Status 
DeviceNode::setRegisterRawData(DeviceRegisterType registerType, std::span<const std::byte> value)
{
	switch (registerType)
	{
	case DeviceRegisterType::Status:
		assert(value.size() == sizeof(m_statusByte));
		m_statusByte = *reinterpret_cast<const StatusByte_t*>(value.data());
		return Status::Ok;
	case DeviceRegisterType::CommonDeviceStatus:
		assert(value.size() == sizeof(m_commonDeviceStatus));
		m_commonDeviceStatus = *reinterpret_cast<const CommonDeviceStatus_t*>(value.data());
		return Status::Ok;
	case DeviceRegisterType::CommonDeviceInformation:
		assert(value.size() == sizeof(m_commonDeviceInformation));
		m_commonDeviceInformation = *reinterpret_cast<const CommonDeviceInformation_t*>(value.data());
		return Status::Ok;
	case DeviceRegisterType::CommonDeviceConfiguration:
		assert(value.size() == sizeof(m_commonDeviceConfiguration));
		m_commonDeviceConfiguration = *reinterpret_cast<const CommonDeviceConfiguration_t*>(value.data());
		return Status::Ok;
	default:
		return Status::RegisterNotImplemented;
	}
}

