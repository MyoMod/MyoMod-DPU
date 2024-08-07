/*
 * Device.h
 *
 *  Created on: Jan 9, 2024
 *      Author: leon
 */

#ifndef DEVICE_H_
#define DEVICE_H_
#include "stdint.h"
#include <vector>
#include <string>
#include <array>
#include <string_view>
#include <span>

#include "Status.h"
#include "MemoryRegion.h"
#include "commonRegisterDef.h"
#include "node.h"

// Packed Storage for DeviceNodes that contains
//  the data itself and the status byte
template <typename T>
struct __attribute__((__packed__)) InputStorage
{
	StatusByte_t statusByte;
	T data;
};

// Forward Declaration of PeripheralHandler for friend declaration
class PeripheralHandler;

class DeviceNode : public BaseNode
{
	// PeripheralHandler may access raw data, this shouldn't be public
    friend class PeripheralHandler;

public:
	DeviceNode(std::array<char, 10> id, const std::array<char, 10> type) :
        m_id{id},
		m_type{type},
		m_activeBuffer{false},
		m_initialized{false}
    {}

	virtual void processInData() = 0;
    virtual void processOutData() = 0;
    virtual DeviceNodeStorage getNodeStorage() = 0;
    const DeviceIdentifier getDeviceIdentifier() const;

protected:
	/** User (common) Register Access **/
	// Status getDeviceVersion(uint32_t* version); TODO: implement versioning
	CommonDeviceStatus_t m_commonDeviceStatus;
	CommonDeviceInformation_t m_commonDeviceInformation;
	CommonDeviceConfiguration_t m_commonDeviceConfiguration;

	/** Raw Register Access **/
	virtual std::span<const std::byte> getRegisterRawData (DeviceRegisterType registerType, Status& status) const;
	virtual Status setRegisterRawData(DeviceRegisterType registerType, std::span<const std::byte> value);

    std::array<char, 10> m_id;
	const std::array<char, 10> m_type;

	bool m_activeBuffer; // active ping-pong buffer for output
	// Common Registers
	// uint32_t m_version; TODO: implement versioning
	bool m_initialized;

	StatusByte_t m_statusByte;

	static const uint32_t STATUS_LENGTH = 1;
};



#endif /* DEVICE_H_ */
