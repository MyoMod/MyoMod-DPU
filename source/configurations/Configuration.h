/*
 * Configuration.h
 *
 *  Created on: Jan 9, 2024
 *      Author: leon
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_
#include "stdint.h"
#include <vector>
#include <string>
#include <array>
#include <set>

#include "Status.h"
#include "AnalysisAlgorithm.h"
#include "PassAlgorithm.h"

namespace freesthetics {

class DeviceDescriptor
{
public:
	// Used to identify the device
	std::array<char, 10> deviceType;
	std::array<char, 10> deviceIdentifier;

	// May be optionally used to identify the device
	int32_t peripheralIndex;
	int32_t deviceAddress = -1;

	// Name for identification 
	std::string name;

	friend bool operator== (const DeviceDescriptor& lhs, const DeviceDescriptor& rhs)
	{
		return lhs.deviceType == rhs.deviceType &&
			   lhs.deviceIdentifier == rhs.deviceIdentifier;
	}
};

struct ChannelDescriptor
{
	DeviceDescriptor* device;
	uint32_t channelIndex;

	// This is the name the channel should get in the PDS, 
	//  it may be different from the named used in the device
	std::string name;
};


struct PDSDescriptor
{
	std::string name;
	std::vector<ChannelDescriptor> channels;
	bool isInput;
};


/*
 *
 */
class Configuration {
public:
	Configuration(std::string_view name, AnalysisAlgorithm* algorithm);
	virtual ~Configuration();
	void getNeededDevices(std::set<DeviceDescriptor*>* devices);
	bool isCompatibleWith(const std::vector<DeviceDescriptor>& devices);

	Status setAddresses(const std::vector<DeviceDescriptor>& devices);
	void resetAddresses();

	std::string name;
	std::vector<PDSDescriptor> PDSs;
	AnalysisAlgorithm* algorithm;
};

} /* namespace freesthetics */

#endif /* CONFIGURATION_H_ */
