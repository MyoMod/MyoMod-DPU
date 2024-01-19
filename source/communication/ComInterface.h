/*
 * ComInterface.h
 *
 *  Created on: Jan 9, 2024
 *      Author: leon
 */

#ifndef COMINTERFACE_H_
#define COMINTERFACE_H_

#include "stdint.h"
#include <vector>
#include <array>

#include "Status.h"
#include "ProcessDataStream.h"
#include "PeripheralHandler.h"
#include "Device.h"
#include "Configuration.h"
#include "ConfigurationManager.h"

namespace freesthetics {

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Data Types
 * ****************************************************************************/

struct DeviceHandle
{
	PeripheralHandler* peripheral;
	uint32_t deviceAddr;

	std::array<char, 10> deviceType;
	std::array<char, 10> deviceIdentifier;

	bool isAvailable;
	Device* device;
};

/*
 *
 */
class ComInterface {
private:
	static Status createDevice(DeviceDescriptor* device, Device** createdDevice);


	std::vector<ProcessDataStream*> pdsIn;
	std::vector<ProcessDataStream*> pdsOut;
	std::array<PeripheralHandler, 3> peripheralHandlers;
	std::array<std::vector< DeviceHandle>, 3> devices; // indexed by PeripheralHandler and Device
		
	//Callback to be called by analysis algorithms
	void (*dataReceivedCallback)(void);
	bool realTimeMode;

	Status sendSync();
	Status handleStatus();
	bool detectedNewDevices();
	// not needed? Status switchActiveDevice(PeripheralHandler* peripheral, Device* device);
	Status addDevice(DeviceDescriptor* device);
	Status removeDevice(DeviceDescriptor* deviceDescriptor);
	Status addPds(PDSDescriptor* pds);
	Status removePds(PDSDescriptor* pds);
	Status getPds(std::string_view name, ProcessDataStream** pds);
	void newDataCallback(uint8_t peripheralIndex);
	void processIncomingData();

public:
	ComInterface(void (*dataReceivedCallback)(void));
	virtual ~ComInterface();

	Status buildFromConfiguration(Configuration* config, Configuration* oldConfig);
	Status enumrateDevices(std::vector<DeviceDescriptor>* devices);
	Status configurationValid();

	Status getDevice(std::string_view name, DeviceHandle** device);
	Status getPds(ProcessDataStream** pds, uint32_t index, bool input);
	Status getAllPds(std::vector<ProcessDataStream*>* pdsIn, std::vector<ProcessDataStream*>* pdsOut);

	Status processOutgoingData();

	Status enterRealTimeMode();
	Status exitRealTimeMode();

	void CyclicHandler();
};

} /* namespace freesthetics */

#endif /* COMINTERFACE_H_ */
