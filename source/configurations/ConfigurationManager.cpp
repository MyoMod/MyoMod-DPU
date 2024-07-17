/**
 * @file ConfigurationManager.cpp
 * @author Leon Farchau (leon2225)
 * @brief
 * @version 0.1
 * @date 13.07.2024
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "ConfigurationManager.h"
#include "configParser.h"
#include "SEGGER_RTT.h"

//Default json configuration file
// const char *configurationString1 = (
// #include "config.json"
// );
const char *configurationString = 
(
R"for_c++_include(
[
    {
        "name": "Config 1",
        "deviceNodes": [
            {
                "type": "BarDisp7Ch",
                "ID" : "BarDisplay"
            }
            ],
        "algorithmicNodes": [
            {
                "type": "TestAlgorithm"
            }],
        "links": {
            "a0:0": "d0:0",
            "d0:0": "a0:0",
            }
    }
])for_c++_include");

/**
 * @brief Construct a new Configuration Manager:: Configuration Manager object
 *
 * @param comInterface
 */
ConfigurationManager::ConfigurationManager()
{

	// Add a default NOP configuration
	configurationHandles.emplace_back(Configuration{"NOP",std::vector<DeviceIdentifier>()}, true);
	setActiveConfiguration(0);
}

/**
 * @brief Parses the configuration and and stores the needed devices in the configurationHandles
 *
 * @return Status 			Ok if the configurations were read successfully
 * 							Error if the configurations could not be read
 */
Status ConfigurationManager::readConfigurations()
{
	// Remove all configurations except the NOP configuration
	removeAllConfigurations();

	ConfigParser &configParser = ConfigParser::getInstance();
	auto configutaions = configParser.scanConfigurations(configurationString);

    for (auto &config : configutaions)
    {
        configurationHandles.emplace_back(config);
    }
    return Status::Ok;
}

/**
 * @brief Updates the isValid flag of all configurations
 *
 * @note Does not update the active configuration.
 * @note Always finds a valid configuration, because the NOP configuration is always valid.
 *
 * @param foundDevices 		The devices found by the ComInterface
 * @return Status 			Ok if the active configuration is valid
 *							Warning if the active configuration is not valid anymore
 * 							 and has been updated to the first valid one
 */
Status ConfigurationManager::updateValidConfigurations(const std::vector<DeviceIdentifier> &foundDevices)
{
	SEGGER_RTT_printf(0, "ConfigurationManager: Update Valid Configurations\n");
	for (auto &configHandle : configurationHandles)
	{
		bool compatible = configHandle.config.isCompatibleWith(foundDevices);

		configHandle.isValid = compatible;
	}

	bool activeConfigIsValid = configurationHandles[activeConfigIndex].isValid;
	return activeConfigIsValid ? Status::Ok : Status::Warning;
}

/**
 * @brief Sets the active configuration
 *
 * @param index 			Index of the configuration to set as active
 * @return Status 			Ok if the configuration was set as active
 * 							Error if either of the following is true:
 * 								- The index is out of bounds
 * 								- The selected configuration is not valid
 * 								- The old algorithm could not be stopped
 * 								- The ComInterface could not be built
 */
Status ConfigurationManager::setActiveConfiguration(uint32_t index)
{
	if (index >= configurationHandles.size() || !configurationHandles[index].isValid)
	{
		return Status::Error;
	}
	Status status = Status::Ok;

	if (activeConfigIndex >= 0)
	{
		// TODO: Maybe implement stop event
	}

	activeConfigIndex = index;
	return status;
}

/**
 * @brief Increments the active configuration
 *
 * @note This excludes the NOP configuration
 *
 * @return Status 			Ok if the active configuration was incremented
 * 							Warning if there is no valid configuration
 * 							Error if the new configuration could not be set as active
 */
Status ConfigurationManager::incrementActiveConfiguration()
{
	// normally the NOP configuration should be ignored, but is used as a fallback
	uint32_t nValid = getNumberOfValidConfigurations();
	if (nValid == 0)
	{
		setActiveConfiguration(0);
		SEGGER_RTT_printf(0, "No valid configuration found, reset to nop configuration\n");
		return Status::Warning;
	}

	int32_t nextIndex = (activeConfigIndex + 1) % nValid; // iterate through the valid configurations
	nextIndex++;										  // But skip the NOP configuration
	SEGGER_RTT_printf(0, "Increment active configuration to %s\n", configurationHandles[nextIndex].config.name.data());
	return setActiveConfiguration(nextIndex);
}

/**
 * @brief Decrements the active configuration
 *
 * @note This excludes the NOP configuration
 *
 * @return Status 			Ok if the active configuration was decremented
 * 							Warning if there is no valid configuration
 * 							Error if the new configuration could not be set as active
 */
Status ConfigurationManager::decrementActiveConfiguration()
{
	// normally the NOP configuration should be ignored, but is used as a fallback
	uint32_t nValid = getNumberOfValidConfigurations();
	if (nValid == 0)
	{
		setActiveConfiguration(0);
		return Status::Warning;
	}

	int32_t nextIndex = (activeConfigIndex - 1) % nValid; // iterate through the valid configurations
	nextIndex++;										  // But skip the NOP configuration
	return setActiveConfiguration(nextIndex);
}

/**
 * @brief Creates the active configuration
 *
 * @return NodesTuple 		The active configuration
 */
NodesTuple ConfigurationManager::createActiveConfiguration()
{
    if (activeConfigIndex == 0)
    {
        return NodesTuple();
    }

	ConfigParser &configParser = ConfigParser::getInstance();
	return configParser.loadConfig(configurationString, activeConfigIndex);	
}

/**
 * @brief Gets the needed devices of the active configuration
 *
 * @return std::vector<DeviceIdentifier> 		The needed devices of the active configuration
 */
std::vector<DeviceIdentifier> ConfigurationManager::getNeededDevices()
{
	return configurationHandles[activeConfigIndex].config.deviceNodes;
}

std::string ConfigurationManager::getActiveConfigurationName()
{
	return configurationHandles[activeConfigIndex].config.name;
}

/**
 * @brief Gets the index of the active configuration
 *
 * @note There is always an active configuration, because the NOP configuration is always valid,
 * 		 therefore this function always returns a valid index
 *
 * @return uint32_t 		Index of the active configuration
 */
uint32_t ConfigurationManager::getActiveConfigurationIndex()
{
	return activeConfigIndex;
}

/**
 * @brief Gets the number of valid configurations
 *
 * @note This excludes the NOP configuration
 *
 * @return uint32_t 		Number of valid configurations
 */
uint32_t ConfigurationManager::getNumberOfValidConfigurations()
{
	uint32_t numberOfValidConfigurations = 0;
	for (auto &configHandle : configurationHandles)
	{
		if (configHandle.isValid)
		{
			numberOfValidConfigurations++;
		}
	}
	return numberOfValidConfigurations - 1;
}


Status ConfigurationManager::removeAllConfigurations()
{
	auto nopConfig = configurationHandles[0];
	configurationHandles.clear();
	configurationHandles.push_back(nopConfig);
	activeConfigIndex = 0;
	return Status::Ok;
}
