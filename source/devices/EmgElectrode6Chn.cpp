#include "EmgElectrode6Chn.h"
#include <cstdint>



EmgElectrode6Chn::EmgElectrode6Chn(std::string_view name) : 
    Device(name) 
{
    const uint32_t samplesPerCycle = 15;
    const uint32_t sampleSize = 4;
    const uint32_t channelNumber = 6;

    uint32_t pdsInSize = STATUS_LENGTH;

    // Add channels
    for (uint32_t i = 0; i < channelNumber; i++) {
        PDSChannel channel {
            .name = "Channel " + std::to_string(i + 1),
            .type = PDSChannelTypes::EMG_32Bit,
            .unit = "mV",
            .length = samplesPerCycle, //just temporary
            .sampleSize = sampleSize,
            .isInput = true
        };
        const uint32_t channelSize = channel.length * channel.sampleSize;
        pdsInSize += channelSize;
        ChannelDataHandle channelHandle {
            .channel = channel,
            .pdsDataRegion = MemoryRegion(),
            .ioDataOffset = channelSize * i,
            .isLinked = false
        };
        channels[1].push_back(channelHandle);
    }

    // Allocate memory for the PDS
    inBuffer.resize(pdsInSize);
    std::fill(inBuffer.begin(), inBuffer.end(), 0xDF);
}

EmgElectrode6Chn::~EmgElectrode6Chn() {
    // TODO Auto-generated destructor stub
}

}
