#include "BarDisplay.h"
#include <cstdint>
#include "SEGGER_RTT.h"



BarDisplay::BarDisplay(std::string_view name) : 
    Device(name) 
{
    // Add display channels
    for (uint32_t i = 0; i < 7; i++) {
        PDSChannel channel {
            .name = "Bar " + std::to_string(i + 1),
            .type = PDSChannelTypes::GENERIC_8Bit,
            .unit = "%",
            .length = 1, //just temporary
            .sampleSize = 1,
            .isInput = false
        };
        const uint32_t channelSize = channel.length * channel.sampleSize;
        ChannelDataHandle channelHandle {
            .channel = channel,
            .pdsDataRegion = MemoryRegion(),
            .ioDataOffset = channelSize * i,
            .isLinked = false
        };
        channels[0].push_back(channelHandle);
    }

    // Add button channels
    for (uint32_t i = 0; i < 4; i++) {
        PDSChannel channel {
            .name = "Button " + std::to_string(i + 1),
            .type = PDSChannelTypes::GENERIC_8Bit,
            .unit = "bool",
            .length = 1, //just temporary
            .sampleSize = 1,
            .isInput = true
        };
        const uint32_t channelSize = channel.length * channel.sampleSize;
        ChannelDataHandle channelHandle {
            .channel = channel,
            .pdsDataRegion = MemoryRegion(),
            .ioDataOffset = channelSize * i,
            .isLinked = false
        };
        channels[1].push_back(channelHandle);
    }

    // Allocate memory for the PDS
    const uint32_t pdsInSize = 1 * 4 + STATUS_LENGTH;
    const uint32_t pdsOutSize = 1 * 7;
    inBuffer.resize(pdsInSize);
    outBuffer[0].resize(pdsOutSize);
    outBuffer[1].resize(pdsOutSize);

    std::fill(inBuffer.begin(), inBuffer.end(), 0xDF);
    std::fill(outBuffer[0].begin(), outBuffer[0].end(), 0xDF);
    std::fill(outBuffer[1].begin(), outBuffer[1].end(), 0xDF);
}

BarDisplay::~BarDisplay() {
    // TODO Auto-generated destructor stub
}

}
