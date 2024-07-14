/*
 * Configuration.cpp
 *
 *  Created on: Jan 9, 2024
 *      Author: leon
 */

#include "Configuration.h"
#include <algorithm>


/**
 * @brief Checks if the configuration is possible with the given devices
 * 
 * @param devices 		List of devices
 * @return true 		If the configuration is possible
 * @return false 		If the configuration is not possible
 */
bool Configuration::isCompatibleWith(const std::vector<DeviceIdentifier>& devices) const
{
	for(auto& neededDevice : deviceNodes)
	{
		if(std::find(devices.begin(), devices.end(), neededDevice) == devices.end())
		{
			return false;
		}
	}

	return true;
}
