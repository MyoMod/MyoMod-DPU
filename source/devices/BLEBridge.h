#pragma once

#include "deviceNode.h"

namespace ble_bridge
{
    //todo: Add a way to config the BLE name 
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

class BLEBridge : public DeviceNode {
public:
    // Constructor
    BLEBridge(std::array<char, 10> id);

    // Destructor
    ~BLEBridge() = default;

    void processInData() override;
    void processOutData() override;
    DeviceNodeStorage getNodeStorage() override;
private:
    ble_bridge::DeviceSpecificConfiguration m_deviceSpecificConfiguration;
    std::span<const std::byte> getRegisterRawData (DeviceRegisterType registerType, Status& status) const override;
    Status setRegisterRawData(DeviceRegisterType registerType, std::span<const std::byte> value) override;

    std::array<std::shared_ptr<OutputPort<uint8_t>>, 1> m_buttonPorts;
    std::shared_ptr<InputStorage<std::array<uint8_t, 1>>> m_hostInStorage;

    std::array<std::shared_ptr<InputPort<uint8_t>>, 8> m_barPorts;
    std::array<std::shared_ptr<std::array<uint8_t, 8>>,2> m_hostOutStorage;

    int32_t m_statusByte;
};


