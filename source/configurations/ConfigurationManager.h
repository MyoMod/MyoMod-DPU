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

#include "Configuration.h"
#include "ComInterface.h"



struct ConfigurationHandle
{
	Configuration config;
	bool isValid;
};

/*
 *
 */
class ConfigurationManager {
public:
	ConfigurationManager(ComInterface* comInterface);
	virtual ~ConfigurationManager();

	Status addConfiguration(const Configuration& config);
	Status removeConfiguration(uint32_t index);
	Status removeConfiguration(std::string_view name);

	Status renumrateDevices();
	Status updateValidConfigurations(const std::vector<DeviceDescriptor>& foundDevices);
	
	Configuration getActiveConfiguration();
	Status setActiveConfiguration(uint32_t index);
	uint32_t getActiveConfigurationIndex();
	uint32_t getNumberOfValidConfigurations();
	Status incrementActiveConfiguration();
	Status decrementActiveConfiguration();
private:
	uint32_t activeConfigIndex = 0;
	std::vector<ConfigurationHandle> configurationHandles;
	ComInterface* comInterface;
};
} /* namespace freesthetics */

#endif /* CONFIGURATIONMANAGER_H_ */
