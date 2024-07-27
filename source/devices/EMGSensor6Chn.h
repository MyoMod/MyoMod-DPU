#pragma once

#include "deviceNode.h"
#include <array>


class EMGSensor6Chn : public DeviceNode {
public:
    // Constructor
    EMGSensor6Chn(std::array<char, 10> id);

    // Destructor
    ~EMGSensor6Chn() = default;

    void processInData() override;
    void processOutData() override;
    DeviceNodeStorage getNodeStorage() override;
private:
    std::array<std::shared_ptr<OutputPort<std::array<uint32_t, 15>>>, 6> m_emgPorts;
    std::shared_ptr<InputStorage<std::array<std::array<uint32_t,15>,6>>> m_hostInStorage;

    std::array<std::shared_ptr<uint8_t>,2> m_hostOutStorage;

    int32_t m_statusByte;
};


