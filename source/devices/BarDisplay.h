#pragma once

#include "deviceNode.h"



class BarDisplay : public DeviceNode {
public:
    // Constructor
    BarDisplay(std::array<char, 10> id);

    // Destructor
    ~BarDisplay() = default;

    void processInData() override;
    void processOutData() override;
    DeviceNodeStorage getNodeStorage() override;
private:
    std::shared_ptr<OutputPort<std::array<uint8_t, 4>>> m_outputPort;
    std::shared_ptr<std::array<uint8_t, 4>> m_hostInStorage;

    std::shared_ptr<InputPort<std::array<uint8_t, 7>>> m_inputPort;
    std::array<std::shared_ptr<std::array<uint8_t, 7>>,2> m_hostOutStorage;

    int32_t m_statusByte;
};


