/*
 * ConfigurationManager.h
 *
 *  Created on: Jan 9, 2024
 *      Author: leon
 */

#ifndef CONFIGURATIONMANAGER_H_
#define CONFIGURATIONMANAGER_H_
#include "stdint.h"
#include <vector>

#include "Status.h"
#include "Configuration.h"
#include "configParser.h"

struct ConfigurationHandle
{
	Configuration config;
	bool isValid = false;

	ConfigurationHandle(Configuration config, bool isValid) : config(config), isValid(isValid) {}
	ConfigurationHandle(Configuration config) : config(config), isValid{false} {}
};

/*
 *
 */
class ConfigurationManager {
public:
	ConfigurationManager();

	Status readConfigurations();

	Status updateValidConfigurations(const std::vector<DeviceIdentifier>& foundDevices);

	Status setActiveConfiguration(uint32_t index);
	Status incrementActiveConfiguration();
	Status decrementActiveConfiguration();

	NodesTuple createActiveConfiguration();

	std::vector<DeviceIdentifier> getNeededDevices();
	std::string getActiveConfigurationName();
	uint32_t getActiveConfigurationIndex();
	bool isValid();

	uint32_t getNumberOfValidConfigurations();
	
private:
	Status removeAllConfigurations();

	uint32_t activeConfigIndex = 0;
	std::vector<ConfigurationHandle> configurationHandles;
};

#endif /* CONFIGURATIONMANAGER_H_ */
