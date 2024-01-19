#include "Device.h"
#include <cstdint>
#include "SEGGER_RTT.h"

namespace freesthetics {

Device::Device(std::string_view name)
	: name(name),
	registerPendingUpdate(0b1110),
	version(0),
	initialized(false),
	channels{{}, {}}
{
	SEGGER_RTT_printf(0, "Channel 0 length: %d\n", channels[0].size());
	// Note: The inherited classes should add the appropriate channels in their constructor
}

Device::~Device() {
	// TODO Auto-generated destructor stub
}

/**
 * @brief Requests an update of the register at the given index
 * 
 * @param registerIndex 
 * @return Status 
 */
Status Device::requestRegisterUpdate(uint32_t registerIndex) {
	// TODO: Implement requestRegisterUpdate method
	return Status::Error;
}
/**
 * @brief Returns the version of the device, returns RegisterNotUpdated if the register has not been updated yet
 * 
 * @param version 
 * @return Status 
 */
Status Device::getDeviceVersion(uint32_t* version) {
	// TODO: Implement getDeviceVersion method
	return Status::Error;
}

/**
 * @brief Returns true if initialized flag is set, 0 otherwise, returns RegisterNotUpdated if the register has not been updated yet
 * 
 * @param initialized 
 * @return Status 
 */
Status Device::getDeviceInitialized(bool* initialized) {
	// TODO: Implement getDeviceInitialized method
	return Status::Error;
}

/**
 * @brief Shedules a write to the initialized flag of the device
 * 
 * @param initialized 
 * @return Status 
 */
Status Device::setDeviceInitialized(bool initialized) {
	// TODO: Implement setDeviceInitialized method
	return Status::Error;
}

/**
 * @brief  Returns true if all sheduled register accesses have been completed
 * 
 * @return true 
 * @return false 
 */
bool Device::registersUpdated() {
	// TODO: Implement registersUpdated method
	return false;
}

/**
 * @brief Returns a bitfield of the registers that need to be updated
 * 
 * @return uint32_t 
 */
uint32_t Device::needsUpdate() {
	// TODO: Implement needsUpdate method
	return 0xFFFFFFFF;
}

/**
 * @brief Returns the status byte of the device
 * 
 * @return StatusByte_t 
 */
StatusByte_t Device::getStatusByte() {
	return *(StatusByte_t*)registerData[0];
}

/**
 * @brief Returns the rawdata of the register at the given index
 * 
 * @param registerIndex 
 * @param value 
 * @return Status 
 */
Status Device::getRegisterRawData(uint32_t registerIndex, void* value) {
	// TODO: Implement getRegisterRawData method
	return Status::Error;
}

/**
 * @brief Sets the rawdata of the register at the given index
 * 
 * @param registerIndex 
 * @param value 
 * @return Status 
 */
Status Device::setRegisterRawData(uint32_t registerIndex, void* value) {
	// TODO: Implement setRegisterRawData method
	return Status::Error;
}

/**
 * @brief Links a channel, so that the device knows where to write/read the data of the indexed
 * 			channel to/from the PDS. Checks if the channel is already linked. Does not check if the
 * 			pdsBuffer is valid.
 * 
 * @param channelIndex 		Index of the channel in the device
 * @param isInput 			True if the channel is an input channel, false if it is an output channel
 * @param pdsBuffer 		MemoryRegion of the PDS buffer to link the channel to
 * 
 * @return Status 			OutOfRange if the channelIndex is out of range, 
 * 							Error if the channel is already linked, 
 * 							Ok otherwise
 */
Status Device::linkPdsChannel(uint32_t channelIndex, bool isInput, MemoryRegion pdsBuffer) {
	if(channelIndex >= channels[isInput].size()) {
		return Status::OutOfRange;
	}

	if(channels[isInput][channelIndex].isLinked) {
		return Status::Error;
	}

	channels[isInput][channelIndex].isLinked = true;
	channels[isInput][channelIndex].pdsDataRegion = pdsBuffer;

	return Status::Ok;
}

/**
 * @brief Unlinks a channel, so that the device no longer accesses the install data region
 * 
 * @param channelIndex 		Index of the channel in the device
 * @param isInput 			True if the channel is an input channel, false if it is an output channel
 * 
 * @return Status 			OutOfRange if the channelIndex is out of range, 
 * 							Error if the channel is not linked, 
 * 							Ok otherwise 
 */
Status Device::unlinkPdsChannel(uint32_t channelIndex, bool isInput) {
	if(channelIndex >= channels[isInput].size()) {
		return Status::OutOfRange;
	}

	if(!channels[isInput][channelIndex].isLinked) {
		return Status::Error;
	}

	channels[isInput][channelIndex].isLinked = false;

	return Status::Ok;
}

/**
 * @brief Returns the PDSChannel of the indexed channel
 * 
 * @param channelIndex 		Index of the channel in the device
 * @param isInput 			True if the channel is an input channel, false if it is an output channel
 * @param channel 			Pointer to the PDSChannel
 * 
 * @return Status 			OutOfRange if the channelIndex is out of range, 
 * 							Ok otherwise
 */
Status Device::getPdsChannel(uint32_t channelIndex, bool isInput, PDSChannel** channel) {
	if(channelIndex >= channels[isInput].size()) {
		return Status::OutOfRange;
	}

	*channel = &channels[isInput][channelIndex].channel;

	return Status::Ok;
}

/**
 * @brief Returns the input buffer of the device
 * 
 * @return MemoryRegion 
 */
MemoryRegion Device::getInBuffer() {
	MemoryRegion inMemoryRegion = {inBuffer.data(), inBuffer.size()};
	return inMemoryRegion;
}

/**
 * @brief Returns the output buffers of the device
 * 
 * @return std::array<MemoryRegion, 2> 
 */
std::array<MemoryRegion, 2> Device::getOutBuffer() {
	MemoryRegion outMemoryRegion0 = {outBuffer[0].data(), outBuffer[0].size()};
	MemoryRegion outMemoryRegion1 = {outBuffer[1].data(), outBuffer[1].size()};
	return {outMemoryRegion0, outMemoryRegion1};
}

/**
 * @brief Processes the raw data from the (I2C etc.) device into the installed PDS
 * 			Should be overriden by inheriting classes if 
 * 
 * @param dataBuffer 
 * @return Status 
 */
Status Device::proccessInData() {
	// Copy the data for the status byte
	*(StatusByte_t*)registerData[0] = *(StatusByte_t*)inBuffer.data();

	// Copy the data for the channels
	for(auto& channel : channels[1]) {
		char* ioBufferAddr = inBuffer.data() + channel.ioDataOffset;
		memcpy(channel.pdsDataRegion.address,  ioBufferAddr, channel.pdsDataRegion.size);
	}
	return Status::Ok;
}

/**
 * @brief Processes the data from the installed PDS into the raw data for the (I2C etc.) device
 * 
 * @param dataBuffer 
 * @return Status 
 */
Status Device::proccessOutData(bool bufferIndex) {
	// Copy the data for the channels
	for(auto& channel : channels[0]) {
		char* ioBufferAddr = outBuffer[bufferIndex].data() + channel.ioDataOffset;
		memcpy(ioBufferAddr, channel.pdsDataRegion.address, channel.pdsDataRegion.size);
	}
	return Status::Ok;
}

} /* namespace freesthetics */
