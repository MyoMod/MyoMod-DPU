#include "BtSink.h"
#include <cstdint>
#include "SEGGER_RTT.h"

namespace freesthetics {

BtSink::BtSink(std::string_view name) : 
    Device(name) 
{

    // Add display channels
    // Reserve space for 7 channels in the vector
    for (uint32_t i = 0; i < 7; i++) {
        PDSChannel channel {
            .name = "Channel " + std::to_string(i + 1),
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

    // Allocate memory for the PDS
    const uint32_t pdsInSize = STATUS_LENGTH;
    const uint32_t pdsOutSize = 1 * 7;
    inBuffer.resize(pdsInSize);
    outBuffer[0].resize(pdsOutSize);
    outBuffer[1].resize(pdsOutSize);

    std::fill(inBuffer.begin(), inBuffer.end(), 0xDF);
    std::fill(outBuffer[0].begin(), outBuffer[0].end(), 0xDF);
    std::fill(outBuffer[1].begin(), outBuffer[1].end(), 0xDF);
}

BtSink::~BtSink() {
    // TODO Auto-generated destructor stub
}

}
