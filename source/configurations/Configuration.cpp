/*
 * Configuration.cpp
 *
 *  Created on: Jan 9, 2024
 *      Author: leon
 */

#include "Configuration.h"
#include <algorithm>

namespace freesthetics {

Configuration::Configuration(std::string_view name, AnalysisAlgorithm* algorithm):
	name{ name },
	algorithm{ algorithm }
{
}

Configuration::~Configuration() {
}

/**
 * @brief Returns a list of all devices needed by this configuration
 * 
 * @param devices 		Pointer to a set of DeviceDescriptors
 */
void Configuration::getNeededDevices(std::set<DeviceDescriptor*>* devices)
{
	for(auto& pds : PDSs)
	{
		for(auto& channel : pds.channels)
		{
			devices->insert(channel.device);
		}
	}
}

/**
 * @brief Checks if the configuration is possible with the given devices
 * 
 * @param devices 		List of devices
 * @return true 		If the configuration is possible
 * @return false 		If the configuration is not possible
 */
bool Configuration::isCompatibleWith(const std::vector<DeviceDescriptor>& devices)
{
	std::set<DeviceDescriptor*> neededDevices;
	getNeededDevices(&neededDevices);

	for(auto& device : neededDevices)
	{
		if(std::find(devices.begin(), devices.end(), *device) == devices.end())
		{
			return false;
		}
	}

	return true;
}

/**
 * @brief Sets the addresses and peripheralIndices of the devices in the configuration 
 * 			according to the given list of devices
 * 
 * @param devices 		List of devices containing addresses and the correct type/identifier pairs
 * @return Status 		Ok if the addresses could be set, 
 * 						Error if the given list of devices does not contain all needed devices
 */
Status Configuration::setAddresses(const std::vector<DeviceDescriptor>& devices)
{
	std::set<DeviceDescriptor*> neededDevices; //Devices needed by this configuration
	getNeededDevices(&neededDevices);

	for(auto& internalDevice : neededDevices)
	{
		auto it = std::find(devices.begin(), devices.end(), *internalDevice);
		if(it == devices.end())
		{
			return Status::Error;
		}
		internalDevice->deviceAddress = it->deviceAddress;
		internalDevice->peripheralIndex = it->peripheralIndex;
	}

	return Status::Ok;
}

/**
 * @brief Resets the addresses of all the devices in the configuration
 * 			to a non-valid value (-1)
 * 
 */
void Configuration::resetAddresses()
{
	for(auto& pds : PDSs)
	{
		for(auto& channel : pds.channels)
		{
			channel.device->deviceAddress = -1;
			channel.device->peripheralIndex = -1;
		}
	}
}
} /* namespace freesthetics */
