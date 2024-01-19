/*
 * ProcessDataStream.h
 *
 *  Created on: Jan 9, 2024
 *      Author: leon
 */

#ifndef PROCESSDATASTREAM_H_
#define PROCESSDATASTREAM_H_

#include "stdint.h"
#include <string>
#include <string_view>
#include <vector>
#include <span>

#include "Status.h"
#include "MemoryRegion.h"
#include "PDSChannel.h"

namespace freesthetics {


/*
 *
 */
class ProcessDataStream {
public:
	ProcessDataStream(std::string_view name);
	virtual ~ProcessDataStream();

	void addChannel(PDSChannel* channel);
	// There is no removeChannel, because a removed channel would invalidate the dara regions
	Status getChannelBuffer(uint32_t channelIndex, MemoryRegion* buffer);
	template<typename T>
	Status getChannelData(uint32_t channelIndex, std::span<T>& data);
	Status getChannelMetaData(uint32_t channelIndex, PDSChannel* channel);

	Status isInput(bool* isInput);
	std::string_view getName() const {return name;}
private:
	std::string name;
	std::vector<PDSChannel*> channels;
	std::vector<std::vector<uint8_t>> dataHandles;
};


// Implementation of template functions

/**
 * @brief Returns a span/view of the channel data. The size of the span
 * 			is automatically calculated from the size of templated type T
 * 
 * @param channelIndex 		Index of the channel to get the data from.
 * @param data 				Span/view of the data.
 * @return Status 			Ok if the channelIndex is valid,
 * 							Error if the channelIndex is out of bounds
 */
template<typename T>
Status ProcessDataStream::getChannelData(uint32_t channelIndex, std::span<T>& data)
{
	if(channelIndex >= channels.size())
	{
		return Status::Error;
	}
	auto numElements = dataHandles[channelIndex].size() / sizeof(T);
	data = std::span<T>((T*)dataHandles[channelIndex].data(), numElements);
	return Status::Ok;
}

} /* namespace freesthetics */

#endif /* PROCESSDATASTREAM_H_ */
