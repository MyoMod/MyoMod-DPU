/*
 * ProcessDataStream.cpp
 *
 *  Created on: Jan 9, 2024
 *      Author: leon
 */

#include "ProcessDataStream.h"

namespace freesthetics {

ProcessDataStream::ProcessDataStream(std::string_view name):
	name(name)
{
}

ProcessDataStream::~ProcessDataStream() {
	// TODO Auto-generated destructor stub
}

/**
 * @brief Adds a channel to the process data stream. And allocates the memory for the data.
 * 
 * @param channel 		Pointer to the channel to add.
 */
void ProcessDataStream::addChannel(PDSChannel* channel)
{
	channels.push_back(channel);
	auto dataSize = channel->length * channel->sampleSize;
	dataHandles.push_back(std::vector<uint8_t>(dataSize));
}

/**
 * @brief Returns a pointer to the data buffer of the channel.
 * 
 * @param channelIndex 		Index of the channel to get the buffer from.
 * @param buffer 			Pointer to the memory region of the buffer.
 * @return Status 			Ok if the channelIndex is valid,
 * 							Error if the channelIndex is out of bounds
 */
Status ProcessDataStream::getChannelBuffer(uint32_t channelIndex, MemoryRegion* buffer)
{
	if(channelIndex >= channels.size())
	{
		return Status::Error;
	}
	buffer->address = dataHandles[channelIndex].data();
	buffer->size = dataHandles[channelIndex].size();
	return Status::Ok;
}

/**
 * @brief Returns the PDSCannel of the indexed channel that contains the metadata.
 * 
 * @param channelIndex 		Index of the channel to get the data from.
 * @param channel 			PDSChannel that contains the metadata.
 * @return Status 			Ok if the channelIndex is valid,
 * 							Error if the channelIndex is out of bounds
 */
Status ProcessDataStream::getChannelMetaData(uint32_t channelIndex, PDSChannel* channel)
{
	if(channelIndex >= channels.size())
	{
		return Status::Error;
	}
	channel = channels[channelIndex];
	return Status::Ok;
}

/**
 * @brief Returns whether the process data stream is an input or output stream.
 * 
 * @param isInput 			Pointer to the boolean that is set to true if the process data stream is an input stream.
 * @return Status 			Ok if the channelIndex is valid,
 * 							Error if the channelIndex is out of bounds
 */
Status ProcessDataStream::isInput(bool* isInput)
{
	if(channels.size() == 0)
	{
		return Status::Error;
	}

	*isInput = channels[0]->isInput;
	return Status::Ok;
}
} /* namespace freesthetics */
