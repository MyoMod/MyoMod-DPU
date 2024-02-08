/*
 * ComInterface.cpp
 *
 *  Created on: Jan 9, 2024
 *      Author: leon
 */

#include "ComInterface.h"
#include "string.h"
#include <array>
#include <string_view>
#include <set>
#include <vector>
#include <algorithm>
#include <functional>

#include "SEGGER_RTT.h"
#include "EmgElectrode6Chn.h"
#include "BarDisplay.h"
#include "BtSink.h"

namespace freesthetics {

ComInterface::ComInterface(void (*dataReceivedCallback)(void)):
	peripheralHandlers{
		PeripheralHandler(DMA0, 1, [this]() -> void {
			return this->newDataCallback(0);
		}, true), 
		PeripheralHandler(DMA0, 3, [this]() -> void {
			return this->newDataCallback(1);
		}, true), 
		PeripheralHandler(DMA0, 4, [this]() -> void {
			return this->newDataCallback(2);
		}, false)
	},
	dataReceivedCallback(dataReceivedCallback),
	realTimeMode(false)
{
}

ComInterface::~ComInterface() {
}

/**
 * @brief Creates a new device with the typed defined in the device descriptor
 * 
 * @param deviceDescriptor 		Descriptor of the device to create
 * @param createdDevice 		Pointer to the created device
 * @return Status 				Error if the device type is not known, Ok otherwise
 */
Status ComInterface::createDevice(DeviceDescriptor* deviceDescriptor, Device** createdDevice) {
	std::array<char, 10> deviceType = deviceDescriptor->deviceType;
	std::string_view name = deviceDescriptor->name;

	if(deviceType == std::array<char, 10> { "Elctr6Ch" })
	{
		*createdDevice = new EmgElectrode6Chn(name);
	}
	else if(deviceType == std::array<char, 10> { "BarDis6Ch" })
	{
		*createdDevice = new BarDisplay(name);
	}
	else if(deviceType == std::array<char, 10> { "BtSink6Ch" })
	{
		*createdDevice = new BtSink(name);
	}
	else
	{
		*createdDevice = nullptr;
		return Status::Error;
	}
	return Status::Ok;
}

/**
 * @brief Sends a sync signal to all devices
 * 
 * @return Status 	Ok if all peripherals were able to send the sync signal, Error otherwise
 */
Status ComInterface::sendSync() {
	Status status;
	for (auto& peripheral : peripheralHandlers)
	{
		status = peripheral.sendSync();
		assert(status == Status::Ok);
	}
	return Status::Ok;
}

/**
 * @brief Handles the status-word received from all devices
 * 
 * @return Status 		Ok if all status-words were handled, Error otherwise
 */
Status ComInterface::handleStatus() {
	[[maybe_unused]] Status status;
	StatusByte_t deviceStatus;

	//iterate over all devices
	for (uint32_t peripheralIndex = 0; peripheralIndex < devices.size(); peripheralIndex++)
	{
		for (auto& deviceHandle : devices[peripheralIndex])
		{
			deviceStatus = deviceHandle.device->getStatusByte();

			// TODO: Handle errors
			// Also check if i2c peripheral maybe able to work without device-ack
			// so that we can detect device availability like we would for other protocols
			
		}
	}
	return Status::Ok;
}

/**
 * @brief Checks if new devices were detected by the peripherals
 * 
 * @return true 	New devices were detected
 * @return false 	No new devices were detected
 */
bool ComInterface::detectedNewDevices() {
	bool detected = false;
	for (auto& peripheral : peripheralHandlers)
	{
		detected |= peripheral.detectedNewDevices();
	}
	return detected;
}

/**
 * @brief Adds a device to the ComInterface and installs it in the peripheral
 * 
 * @param deviceDescriptor 		Descriptor of the device to add
 * @return Status 				Ok if the device was added, 
 * 								Error if the device has not been assigned an address,
 * 								a peripheral or if the device type is not known
 */
Status ComInterface::addDevice(DeviceDescriptor* deviceDescriptor) {
	Status status;

	// Check if the device has been assigned an address and a peripheral
	if(deviceDescriptor->deviceAddress == -1 || deviceDescriptor->peripheralIndex == -1)
	{
		return Status::Error;
	}


	// Create a new device
	Device* newDevice = nullptr;
	status = createDevice(deviceDescriptor, &newDevice);
	if (status != Status::Ok)
	{
		// No such device, don't add it
		return Status::Error;
	}
	
	// Add the device to a handle and add the handle to the list of devices
	PeripheralHandler* peripheral = &peripheralHandlers[deviceDescriptor->peripheralIndex];
	devices[deviceDescriptor->peripheralIndex].push_back(DeviceHandle{
		.peripheral = peripheral,
		.deviceAddr = static_cast<uint32_t>(deviceDescriptor->deviceAddress), 
		.deviceType = deviceDescriptor->deviceType,
		.deviceIdentifier = deviceDescriptor->deviceIdentifier,
		.isAvailable = true, 
		.device = newDevice});

	// Install the device in the peripheral
	status = peripheral->addDevice(deviceDescriptor->deviceAddress, newDevice->getInBuffer(), newDevice->getOutBuffer());
	assert(status == Status::Ok);

	return Status::Ok;
}

/**
 * @brief Removes a device from the ComInterface and the peripheral, the device is matched by its address
 * 
 * @param deviceDescriptor 		Descriptor of the device to remove
 * @return Status 				Error if the device was not found, Ok otherwise
 */
Status ComInterface::removeDevice(DeviceDescriptor* deviceDescriptor) {
	Status status;
	uint32_t peripheralIndex = deviceDescriptor->peripheralIndex;

	// TODO: Maybe change to find by name
	// Get the address of the device from the device index
	uint32_t deviceAddress = deviceDescriptor->deviceAddress;
	uint32_t handleIndex = 0;
	for(auto& deviceHandle : devices[peripheralIndex])
	{
		if(deviceHandle.deviceAddr == deviceAddress)
		{
			break;
		}
		handleIndex++;
	}

	if(handleIndex == devices[peripheralIndex].size())
	{
		// No such device, don't remove it
		return Status::Error;
	}

	// Remove the device from the peripheral
	status = peripheralHandlers[peripheralIndex].removeDevice(deviceAddress);
	assert(status == Status::Ok);

	// Free the memory of the dynamic allocated device
	delete devices[peripheralIndex][handleIndex].device;
	
	// Remove the device handle from the list of devices
	devices[peripheralIndex].erase(devices[peripheralIndex].begin() + handleIndex);

	return Status::Ok;
}

/**
 * @brief Gets a device by its name
 * 
 * @param name 		Name of the device to get
 * @param device 	Pointer to the device
 * @return Status 	Error if the device was not found, Ok otherwise
 */
Status ComInterface::getDevice(std::string_view name, DeviceHandle** device) {
	for(auto& deviceVector : devices)
	{
		for(auto& deviceHandle : deviceVector)
		{
			std::string_view deviceName = deviceHandle.device->getName();
			const char* str = deviceName.data();
			int length = deviceName.length();
			if(deviceHandle.device->getName() == name)
			{
				*device = &deviceHandle;
				return Status::Ok;
			}
		}
	}
	return Status::Error;
}

/**
 * @brief Adds a PDS to the ComInterface, the PDS is not linked to any device
 * 
 * @param pdsDescriptor 		Descriptor of the PDS to add
 * @return Status 				Ok if the PDS was added, Error otherwise
 */
Status ComInterface::addPds(PDSDescriptor* pdsDescriptor) {
	// Create a new PDS
	ProcessDataStream* newPds = new ProcessDataStream(pdsDescriptor->name);

	// Can't add the channels without the knowledge of the devices, as only they
	//  know the sizes etc. of the channels

	// Add the PDS to the list of PDSs
	if (pdsDescriptor->isInput)
	{
		pdsIn.push_back(newPds);
	}
	else
	{
		pdsOut.push_back(newPds);
	}

	return Status::Ok;
}

/**
 * @brief Removes a PDS from the ComInterface
 * 
 * @param pdsDescriptor 		Descriptor of the PDS to remove
 * @return Status 				Error if a PDS with the name declared in the descriptor 
 * 									was not found, Ok otherwise
 */
Status ComInterface::removePds(PDSDescriptor* pdsDescriptor) {
	std::vector<ProcessDataStream*>* pdsVector;
	pdsVector = pdsDescriptor->isInput ? &pdsIn : &pdsOut;
	
	uint32_t pdsIndex = 0;
	for(auto& pds : *pdsVector)
	{
		if(pds->getName() == pdsDescriptor->name)
		{
			delete pds;
			pdsVector->erase(pdsVector->begin() + pdsIndex);
			return Status::Ok;
		}
		pdsIndex++;
	}
	return Status::Error;
}

/**
 * @brief Gets a PDS by its name
 * 
 * @param name 		Name of the PDS to get
 * @param pds 		Pointer to the PDS
 * @return Status 	Error if the PDS was not found, Ok otherwise
 */
Status ComInterface::getPds(std::string_view name, ProcessDataStream** pds) {
	for(auto& pdsVector : {pdsIn, pdsOut})
	{
		for(auto& pdsHandle : pdsVector)
		{
			if(pdsHandle->getName() == name)
			{
				*pds = pdsHandle;
				return Status::Ok;
			}
		}
	}
	pds = nullptr;
	return Status::Error;
}

/**
 * @brief Callback function that is called by the peripherals when new data is available for that peripheral
 * 
 * @param peripheralIndex 		Index of the peripheral that called the callback
 * @param pingPongIndex 		Index of the ping-pong buffer that can be written to
 */
void ComInterface::newDataCallback(uint8_t peripheralIndex) {
	static std::array<bool, 3> dataAvailable = {false, false, false};

	dataAvailable[peripheralIndex] = true;

	// Check if all peripherals have data available
	bool allDataAvailable = true;
	for (size_t i = 0; i < dataAvailable.size(); i++)
	{
		allDataAvailable &= (dataAvailable[i] || !peripheralHandlers[i].hasDevices());
	}

	if (allDataAvailable)
	{
		processIncomingData();
		for (auto& dataAvailable : dataAvailable)
		{
			dataAvailable = false;
		}
	}
}

/**
 * @brief Processes the incoming data from the peripherals and lets the devices process their buffers
 * 				and therefore let them update the data in the PDSs. Afterwards the status of the devices
 * 				is checked and the callback function is called
 * 
 * @return Status 	Ok if the data was processed, Error otherwise
 */
void ComInterface::processIncomingData() {
	// The incoming data is copied to the ioBuffers of the devices by DMA/peripheralhandler
	//  So we only need to let the devices to process their buffers and let them post the
	//  new data to the PDSs
	for (auto& deviceVector : devices)
	{
		for (auto& deviceHandle : deviceVector)
		{
			deviceHandle.device->proccessInData();
		}
	}
	Status status = handleStatus();
	assert(status == Status::Ok);

	//Call callback function
	dataReceivedCallback();
}

/**
 * @brief Configures the ComInterface according to the given configuration and removes the
 * 				configuration if given by oldConfig
 * 
 * @param newConfig 		Configuration to build the ComInterface from
 * @param oldConfig 		Old configuration to revert 
 * @return Status 			Ok if the configuration was built, Error otherwise
 */
Status ComInterface::buildFromConfiguration(Configuration* newConfig,  Configuration* oldConfig) {
	Status status;
	// Clean up and remove all devices
	if (oldConfig != nullptr)
	{
		// Using a set as the same device may be used in multiple PDSs
		std::set<DeviceDescriptor*> oldDevices;
		for (auto& pds : oldConfig->PDSs)
		{
			for (auto& channel : pds.channels)
			{
				oldDevices.insert(channel.device);
			}
		}
		
		for (auto& device : oldDevices)
		{
			status = removeDevice(device);
			assert(status == Status::Ok);
		}

		// Remove the PDSs
		for (auto& pds : oldConfig->PDSs)
		{
			status = removePds(&pds);
			assert(status == Status::Ok);
		}
	}

	// Add the PDSs
	for (auto& pds : newConfig->PDSs)
	{
		status = addPds(&pds);
		assert(status == Status::Ok);
	}

	// Add the devices
	// Using a set as the same device may be used in multiple PDSs
	std::set<DeviceDescriptor*> newDevices;
	for (auto& pds : newConfig->PDSs)
	{
		for (auto& channel : pds.channels)
		{
			newDevices.insert(channel.device);
		}
	}
	
	for (auto& device : newDevices)
	{
		SEGGER_RTT_printf(0, "ComInterface: Adding device: %s\n", device->name.data());
		status = addDevice(device);
		assert(status == Status::Ok);
	}

	// Link devices to PDSs (install channels)
	for (auto& pdsDescriptor : newConfig->PDSs)
	{
		ProcessDataStream* pdsHandle = nullptr;
		status = getPds(pdsDescriptor.name, &pdsHandle);
		assert(status == Status::Ok);

		// First add all channels to the PDS
		for (auto& channelDescriptor : pdsDescriptor.channels)
		{
			// Get the device for this channel
			std::string_view deviceName = channelDescriptor.device->name;
			DeviceHandle* deviceHandle = nullptr;
			status = getDevice(deviceName, &deviceHandle);
			assert(status == Status::Ok);

			// Get the PDSChannel
			uint32_t channelIndex = channelDescriptor.channelIndex;
			bool isInput = pdsDescriptor.isInput;
			PDSChannel* pdsChannel = nullptr;
			status = deviceHandle->device->getPdsChannel(channelIndex, isInput, &pdsChannel);
			assert(status == Status::Ok);

			// Add the channel to the PDS
			pdsHandle->addChannel(pdsChannel);	
		}

		// Then link the channels to the devices
		// Note: This is done in a seconds step as it otherwise may be possible that
		//  the pds reallocates memors for the channels and therefore invalidates all
		//  prior links to the deivces
		for (auto& channelDescriptor : pdsDescriptor.channels)
		{
			// Get the device for this channel
			std::string_view deviceName = channelDescriptor.device->name;
			DeviceHandle* deviceHandle = nullptr;
			status = getDevice(deviceName, &deviceHandle);
			assert(status == Status::Ok);

			// Gather the needed information
			uint32_t channelIndex = channelDescriptor.channelIndex;
			bool isInput = pdsDescriptor.isInput;
			MemoryRegion buffer;
			status = pdsHandle->getChannelBuffer(channelIndex, &buffer);
			assert(status == Status::Ok);

			// Install the channel
			status = deviceHandle->device->linkPdsChannel(channelIndex, isInput, buffer);
			assert(status == Status::Ok);			
		}
	}
	return Status::Ok;
}

/**
 * @brief Scans for devices on all peripherals and and returns a list of found devicesDescriptors 
 * 			with type, identifier, address and peripheral. It does not fill the name of the devices
 * 			
 * @note It does not add the devices to the ComInterface.
 * 
 * @param foundDevices 		Vector to store the found devices in
 * @return Status 			Ok if the devices were enumerated, Error otherwise
 */
Status ComInterface::enumrateDevices(std::vector<DeviceDescriptor>* foundDevices) {
	uint32_t peripheralIndex = 0;
	for(auto& peripheral : peripheralHandlers)
	{

		// Get the devices from the peripheral (with filled addresses)
		std::vector<DeviceDescriptor> peripheralDevices;
		Status status = peripheral.listNewDevices(&peripheralDevices);
		assert(status == Status::Ok);

		// Add the devices to the list of found devices
		for(auto& deviceDescriptor : peripheralDevices)
		{
			// While the type, identifier and address of the device is known, the name is not
			//  known yet, as it is delivered by the configuration
			deviceDescriptor.peripheralIndex = peripheralIndex;
			foundDevices->push_back(deviceDescriptor);
		}
		peripheralIndex++;
	}
	return Status::Ok;
}

/**
 * @brief Checks if the configuration is valid, i.e. all needed/installed devices are available
 * 
 * @return Status 	Ok if the configuration is valid, Error otherwise
 */
Status ComInterface::configurationValid() {
	bool valid = true;
	for (auto& deviceGroup : devices)
	{
		for (auto& deviceHandle : deviceGroup)
		{
			valid &= deviceHandle.isAvailable;
		}
	}
	return valid ? Status::Ok : Status::Error;
}

/**
 * @brief Gets a PDS by its index and the direction
 * 
 * @param pds 		Pointer to the returned PDS
 * @param index 	Index of the PDS to be returned
 * @param input 	True if the PDS that should be returned is an input, false otherwise
 * @return Status 	Error if the PDS was not found, Ok otherwise
 */
Status ComInterface::getPds(ProcessDataStream** pds, uint32_t index, bool input) {
	auto pdsVector = input ? pdsIn : pdsOut;
	if (index >= pdsVector.size())
	{
		return Status::Error;
	}
	*pds = pdsVector[index];
	return Status::Ok;
}

/**
 * @brief Gets all PDSs
 * 
 * @param pdsIn 		Vector to store the input PDSs in
 * @param pdsOut 		Vector to store the output PDSs in
 * @return Status 		Ok if the PDSs were returned, Error otherwise
 */
Status ComInterface::getAllPds(std::vector<ProcessDataStream*>** pdsIn, std::vector<ProcessDataStream*>** pdsOut) {
	*pdsIn = &this->pdsIn;
	*pdsOut = &this->pdsOut;
	return Status::Ok;
}

/**
 * @brief Processes the outgoing data of the devices
 * 
 * @return Status 	Ok if the data was processed, Error otherwise
 */
Status ComInterface::processOutgoingData() {
	Status status;
	for(size_t peripheralIndex = 0; peripheralIndex < peripheralHandlers.size(); peripheralIndex++)
	{
		uint32_t bufferIndex = peripheralHandlers[peripheralIndex].getPingPongIndex();
		for(auto& deviceHandle : devices[peripheralIndex])
		{
			status = deviceHandle.device->proccessOutData(bufferIndex);
			assert(status == Status::Ok);
		}
	}
	return Status::Ok;
}

/**
 * @brief Enters the realtime mode, i.e. the ComInterface will start to process the data
 * 				from the peripherals. It is now not possible to change the registers of the
 * 				devices
 * 
 * @return Status 	Ok if the realtime mode was entered, Error otherwise
 */
Status ComInterface::enterRealTimeMode() {
	realTimeMode = true;

	// Enter RT mode for the peripherals
	for(auto& peripheralHandler : peripheralHandlers)
	{
		Status status = peripheralHandler.enterRealTimeMode();
		assert(status == Status::Ok);
	}

	return Status::Ok;
}

/**
 * @brief Exits the realtime mode, i.e. the ComInterface will stop to process the data
 * 				from the peripherals. It is now possible to change the registers of the
 * 				devices
 * 
 * @return Status 	Ok if the realtime mode was exited, Error otherwise
 */
Status ComInterface::exitRealTimeMode() {
	realTimeMode = false;

	// Exit RT mode for the peripherals
	for(auto& peripheralHandler : peripheralHandlers)
	{
		Status status = peripheralHandler.exitRealTimeMode();
		assert(status == Status::Ok);
	}

	return Status::Ok;
}

/**
 * @brief Interrupt handler for the PIT, that should be called every 10ms
 * 
 */
void ComInterface::CyclicHandler() {
	if(realTimeMode)
	{
		Status status = sendSync();
		assert(status == Status::Ok);

		uint32_t index = 0;
		for(auto& peripheralHandler : peripheralHandlers)
		{
			Status status = peripheralHandler.startCycle();
			if(status != Status::Ok)
			{
				SEGGER_RTT_printf(0, "PeripheralHandler %d was not ready for the next cycle\n", index);
				//assert(status == Status::Ok);
			}
			index++;
		}
	}
}


} /* namespace freesthetics */
