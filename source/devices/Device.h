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
#include "PDSChannel.h"
#include "ProcessDataStream.h"
#include "commonRegisterDef.h"

namespace freesthetics {

struct ChannelDataHandle
{
	PDSChannel channel;
	MemoryRegion pdsDataRegion; // Position of the data in the PDS
	uint32_t ioDataOffset; // Position of the data in the raw IO data, filled in constructor
	bool isLinked = false;
};

/*
 *
 */
class Device {
public:
	Device(std::string_view name);
	virtual ~Device();

	/** User (common) Register Access **/
	Status requestRegisterUpdate(uint32_t registerIndex);
	Status getDeviceVersion(uint32_t* version);
	Status getDeviceInitialized(bool* initialized);
	Status setDeviceInitialized(bool initialized);

	/** Raw Register Access **/
	bool registersUpdated();
	uint32_t needsUpdate();
	StatusByte_t getStatusByte();
	Status getRegisterRawData(uint32_t registerIndex, void* value);
	Status setRegisterRawData(uint32_t registerIndex, void* value);

	/** PDS Access **/
	Status linkPdsChannel(uint32_t channelIndex, bool isInput, MemoryRegion pdsBuffer);
	Status unlinkPdsChannel(uint32_t channelIndex, bool isInput);
	Status getPdsChannel(uint32_t channelIndex, bool isInput, PDSChannel** channel);
	MemoryRegion getInBuffer();
	std::array<MemoryRegion, 2> getOutBuffer();
	virtual Status proccessInData();
	virtual Status proccessOutData(bool bufferIndex);

	virtual std::array<char, 10> getDeviceType() = 0;

	// Configuration
	std::string_view getName() { return name; }
protected:
	std::vector<ChannelDataHandle> channels[2]; //1: Input, 0: Output

	// IO Buffers 
	// Note1 that the input buffer doesn't need to be a ping pong buffer as it gets copied 
	//  to the PDS buffers at the beginning of each processing cycle and therefore isn't
	//  affected by a overwrite from the next cycle's data
	// Note2 the input buffer contains the stream data and the status byte
	std::vector<char> inBuffer; 
	std::array<std::vector<char>, 2> outBuffer; // Ouput ping-pong buffer

	// Internals
	std::string name; //Name given by configuration

	// Register Handling
	std::vector<uint8_t*> registerData;
	uint32_t registerPendingUpdate;

	// Common Registers
	uint32_t version;
	bool initialized;

	static const uint32_t STATUS_LENGTH = 1;
};

} /* namespace freesthetics */

#endif /* DEVICE_H_ */
