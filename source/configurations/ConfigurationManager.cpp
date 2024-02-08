#include "ConfigurationManager.h"
#include "NopAlgorithm.h"

namespace freesthetics {

ConfigurationManager::ConfigurationManager(ComInterface* comInterface):
	comInterface{ comInterface }
{
	AnalysisAlgorithm* nopAlgorithm = new NopAlgorithm("Sample Algorithm");

	// Add a default NOP configuration
	Configuration nopConfiguration {
		"NOP",
		nopAlgorithm
	};

	addConfiguration(nopConfiguration);
	configurationHandles[0].isValid = true; // The NOP configuration is always valid
	setActiveConfiguration(0);
}

ConfigurationManager::~ConfigurationManager() {
	delete configurationHandles[0].config.algorithm;
}

/**
 * @brief Adds a configuration
 * 
 * @note The configuration is not valid until renumrateDevices() is called
 * 
 * @param config 			Configuration to add
 * @return Status 			Ok if the configuration was added
 */
Status ConfigurationManager::addConfiguration(const Configuration& config) {

	configurationHandles.push_back(ConfigurationHandle {
		.config = config,
		.isValid = false
	});
	return freesthetics::Status::Ok;
}

/**
 * @brief Removes the configuration at the given index
 * 
 * @note The NOP configuration and the active configuration may not be removed
 * @note The active configuration index is updated if needed
 * 
 * @param index 			Index of the configuration to remove
 * @return Status 			Ok if the configuration was removed
 * 							Error if either of the following is true:
 * 								- The index is out of bounds
 * 								- The selected configuration was the activeConfiguration
 * 								- The selected configuration was the NOP configuration
 */
Status ConfigurationManager::removeConfiguration(uint32_t index) {
	// The nopConfiguration and the active configuration may not be removed
	if(0 < index && index < configurationHandles.size() && index != activeConfigIndex)
	{
		// Remove the configuration
		configurationHandles.erase(configurationHandles.begin() + index);

		// Update the active configuration index if needed
		if(index < activeConfigIndex)
		{
			activeConfigIndex--;
		}
		return freesthetics::Status::Ok;
	}
	else
	{
		return freesthetics::Status::Error;
	}
}

/**
 * @brief Removes the configuration with the given name
 * 
 * @note The NOP configuration and the active configuration may not be removed
 * @note The active configuration index is updated if needed
 * 
 * @param name 				Name of the configuration to remove
 * @return Status 			Ok if the configuration was removed
 * 							Error if either of the following is true:
 * 								- A configuration with the given name does not exist
 * 								- The selected configuration was the activeConfiguration
 * 								- The selected configuration was the NOP configuration
 */
Status ConfigurationManager::removeConfiguration(std::string_view name) {
	// Remove the configuration
	for(uint32_t i = 0; i < configurationHandles.size(); i++)
	{
		if(configurationHandles[i].config.name == name)
		{
			return removeConfiguration(i);
		}
	}
	return freesthetics::Status::Error;
}

/**
 * @brief Enumerates the devices and updates the isValid flag of all configurations
 * 
 * @note Sets the active configuration to the first valid one if the current one is not valid anymore
 * @note Always finds a valid configuration, because the NOP configuration is always valid
 * 
 * @return Status 			Ok if the active configuration is valid
 * 							Warning if the active configuration is not valid anymore 
 * 							 and has been updated to the first valid one
 */
Status ConfigurationManager::renumrateDevices() {
	Status status;
	std::vector<freesthetics::DeviceDescriptor> foundDevices;
	status = comInterface->enumrateDevices(&foundDevices);

	SEGGER_RTT_printf(0, "ConfigurationManager: Renumrate Devices\n");
	for (auto& device : foundDevices) {
		SEGGER_RTT_printf(0, "ConfigurationManager: Found device: %s %s at peripheral %d\n", device.deviceType.data(), device.deviceIdentifier.data(), device.peripheralIndex);
	}

	assert(status == freesthetics::Status::Ok);

	/** Update which configurations are still possible and update the current configuration if needed**/
	return updateValidConfigurations(foundDevices);
}

/**
 * @brief Updates the isValid flag of all configurations
 * 
 * @note Sets the active configuration to the first valid one if the current one is not valid anymore
 * @note Always finds a valid configuration, because the NOP configuration is always valid
 * 
 * @param foundDevices 		The devices found by the ComInterface
 * @return Status 			Ok if the active configuration is valid
 *							Warning if the active configuration is not valid anymore 
 * 							 and has been updated to the first valid one
 */
Status ConfigurationManager::updateValidConfigurations(const std::vector<DeviceDescriptor>& foundDevices) {
	SEGGER_RTT_printf(0, "ConfigurationManager: Update Valid Configurations\n");
	for(auto& configHandle : configurationHandles)
	{
		bool compatible = configHandle.config.isCompatibleWith(foundDevices);

		// If the configHandle is compatible with the found devices:
		// 	- Set the isValid flag
		//  - fill in the actual device addresses
		if(compatible)
		{
			configHandle.isValid = true;
			configHandle.config.setAddresses(foundDevices);
		}

		// If the active configuration is not valid:
		// 	- Clear the isValid flag
		//  - reset the addresses
		else
		{
			configHandle.isValid = false;
			configHandle.config.resetAddresses();
		}
		
	}
	bool activeConfigIsValid = configurationHandles[activeConfigIndex].isValid;
	if(!activeConfigIsValid)
	{
		// Find the first valid configuration
		for(uint32_t i = 0; i < configurationHandles.size(); i++)
		{
			if(configurationHandles[i].isValid)
			{
				SEGGER_RTT_printf(0, "ConfigurationManager: Old configuration was no longer valid, set active configuration to %s\n", configurationHandles[i].config.name.data());

				setActiveConfiguration(i);
				break;
			}
		}
	}
	else
	{
		SEGGER_RTT_printf(0, "ConfigurationManager: Active configuration is still valid\n");
	}

	return activeConfigIsValid ? freesthetics::Status::Ok : freesthetics::Status::Warning;
}

/**
 * @brief Gets the active configuration
 * 
 * @return			Reference to which the active configuration should be copied
 */
Configuration ConfigurationManager::getActiveConfiguration() {
	return configurationHandles[activeConfigIndex].config;
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
Status ConfigurationManager::setActiveConfiguration(uint32_t index) {
	if(index >= configurationHandles.size() || !configurationHandles[index].isValid)
	{
		return freesthetics::Status::Error;
	}
	Status status;

	if(activeConfigIndex >= 0)
	{
		// Stop the current configuration
		status = configurationHandles[activeConfigIndex].config.algorithm->stop();
		if(status != freesthetics::Status::Ok)
		{
			return Status::Error;
		}
	}

	Configuration* oldConfiguration = nullptr;
	if(activeConfigIndex >= 0)
	{
		oldConfiguration = &configurationHandles[activeConfigIndex].config;
	} 
	activeConfigIndex = index;

	/** Build the ComInterface from the active configuration **/
	status = comInterface->buildFromConfiguration(&configurationHandles[activeConfigIndex].config, oldConfiguration);
	return status;
}

/**
 * @brief Gets the index of the active configuration
 * 
 * @note There is always an active configuration, because the NOP configuration is always valid,
 * 		 therefore this function always returns a valid index
 * 
 * @return uint32_t 		Index of the active configuration
 */
uint32_t ConfigurationManager::getActiveConfigurationIndex() {
	return activeConfigIndex;
}

/**
 * @brief Gets the number of valid configurations
 * 
 * @note This excludes the NOP configuration
 * 
 * @return uint32_t 		Number of valid configurations
 */
uint32_t ConfigurationManager::getNumberOfValidConfigurations() {
	uint32_t numberOfValidConfigurations = 0;
	for(auto& configHandle : configurationHandles)
	{
		if(configHandle.isValid)
		{
			numberOfValidConfigurations++;
		}
	}
	return numberOfValidConfigurations - 1;
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
Status ConfigurationManager::incrementActiveConfiguration() {
	// normally the NOP configuration should be ignored, but is used as a fallback
	uint32_t nValid = getNumberOfValidConfigurations();
	if(nValid == 0)
	{
		setActiveConfiguration(0);
		SEGGER_RTT_printf(0, "No valid configuration found, reset to nop configuration\n");
		return freesthetics::Status::Warning;
	}


	int32_t nextIndex = (activeConfigIndex + 1) % nValid; // iterate through the valid configurations
	nextIndex++; // But skip the NOP configuration
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
Status ConfigurationManager::decrementActiveConfiguration() {
	// normally the NOP configuration should be ignored, but is used as a fallback
	uint32_t nValid = getNumberOfValidConfigurations();
	if(nValid == 0)
	{
		setActiveConfiguration(0);
		return freesthetics::Status::Warning;
	}


	int32_t nextIndex = (activeConfigIndex - 1) % nValid; // iterate through the valid configurations
	nextIndex++; // But skip the NOP configuration
	return setActiveConfiguration(nextIndex);
}
} /* namespace freesthetics */
