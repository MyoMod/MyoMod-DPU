#pragma once

#include "deviceNode.h"

enum class BarDisplayButtonNames
{
    A = 0,
    B = 1,
    X = 2,
    Y = 3
};

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
    std::array<std::shared_ptr<OutputPort<bool>>, 4> m_buttonPorts;
    std::shared_ptr<InputStorage<std::array<uint8_t, 4>>> m_hostInStorage;

    std::array<std::shared_ptr<InputPort<uint8_t>>, 7> m_barPorts;
    std::array<std::shared_ptr<std::array<uint8_t, 7>>,2> m_hostOutStorage;

    int32_t m_statusByte;
};


