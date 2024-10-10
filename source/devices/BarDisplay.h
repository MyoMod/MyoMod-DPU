#pragma once

#include "deviceNode.h"

enum class BarDisplayButtonNames
{
    A = 0,
    B = 1,
    X = 2,
    Y = 3
};

namespace bar_display
{
struct DeviceSpecificConfiguration
{
    std::array<uint32_t, 7> barColors =
    {
        0x00ff00,
        0xffff00,
        0xff0000,
        0xff00ff,
        0x0000ff,
        0x00ffff,
        0x000000
    };
};
}

class BarDisplay : public DeviceNode {
public:
    // Constructor
    BarDisplay(std::array<char, 10> id, std::array<uint32_t, 7> barColors);

    // Destructor
    ~BarDisplay() = default;

    void processInData() override;
    void processOutData() override;
    DeviceNodeStorage getNodeStorage() override;
private:
    bar_display::DeviceSpecificConfiguration m_deviceSpecificConfiguration;
    std::span<const std::byte> getRegisterRawData (DeviceRegisterType registerType, Status& status) const override;
    Status setRegisterRawData(DeviceRegisterType registerType, std::span<const std::byte> value) override;

    std::array<std::shared_ptr<OutputPort<uint8_t>>, 4> m_buttonPorts;
    std::shared_ptr<InputStorage<std::array<uint8_t, 4>>> m_hostInStorage;

    std::array<std::shared_ptr<InputPort<uint8_t>>, 7> m_barPorts;
    std::array<std::shared_ptr<std::array<uint8_t, 7>>,2> m_hostOutStorage;

    int32_t m_statusByte;
};


