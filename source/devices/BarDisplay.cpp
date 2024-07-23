#include "BarDisplay.h"
#include <cstdint>
#include "SEGGER_RTT.h"



BarDisplay::BarDisplay(std::array<char, 10> id) : 
    DeviceNode(id, 
        std::array<char, 10>({'B','a','r','D','i','s','p','7','C','h'})),
    m_buttonPorts(),
    m_hostInStorage(std::make_shared<InputStorage<std::array<uint8_t, 4>>>()),
    m_barPorts(),
    m_hostOutStorage({
        std::make_shared<std::array<uint8_t, 7>>(std::array<uint8_t, 7>({0,0,0,0,0,0,0})),
        std::make_shared<std::array<uint8_t, 7>>(std::array<uint8_t, 7>({0,0,0,0,0,0,0}))
    })
{
    for(size_t i = 0; i < m_barPorts.size(); i++)
    {
        m_barPorts[i] = std::make_shared<InputPort<uint8_t>>(0);
        m_inputPorts.push_back(m_barPorts[i]);
    }

    for(size_t i = 0; i < m_buttonPorts.size(); i++)
    {
        m_buttonPorts[i] = std::make_shared<OutputPort<uint8_t>>();
        m_outputPorts.push_back(m_buttonPorts[i]);
    }
}

void BarDisplay::processInData()
{
    // TODO: Do some validation here
    auto& data = m_hostInStorage->data;
    for (size_t i = 0; i < m_buttonPorts.size(); i++)
    {
        m_buttonPorts[i]->setValue(data[i] == 1);
        m_buttonPorts[i]->setValid(true);
    }
    SEGGER_RTT_Write(1, data.data(), data.size());
}

void BarDisplay::processOutData()
{
    auto data = m_hostOutStorage[m_activeBuffer].get();
    m_activeBuffer = !m_activeBuffer;
    for (size_t i = 0; i < m_barPorts.size(); i++)
    {
        (*data)[i] = m_barPorts[i]->getValue();
    }
}

DeviceNodeStorage BarDisplay::getNodeStorage()
{
    DeviceNodeStorage storage;
    storage.inStorage = reinterpret_pointer_cast<std::byte>(m_hostInStorage);
    storage.inSize = sizeof(*m_hostInStorage);
    storage.outStorage[0] = reinterpret_pointer_cast<std::byte>(m_hostOutStorage[0]);
    storage.outStorage[1] = reinterpret_pointer_cast<std::byte>(m_hostOutStorage[1]);
    storage.outSize = sizeof(*(m_hostOutStorage[0]));

    return storage;
}

/**
 * @brief Returns the raw data of the register specified by registerType
 * 
 * @param registerType 		The type of the register to get the raw data from
 * @param status 			The status of the operation
 * 								- Status::Ok if the operation was successful
 *                              - Status::RegisterNotSupported if this device doesn't implement the register
 * @return std::span<const std::byte> 
 */
std::span<const std::byte> BarDisplay::getRegisterRawData(DeviceRegisterType registerType, Status& status) const
{
    status = Status::Ok;
    switch (registerType)
    {
    case DeviceRegisterType::DeviceSpecificConfiguration:
        return std::span<const std::byte>(
            reinterpret_cast<const std::byte*>(&m_deviceSpecificConfiguration),
            sizeof(m_deviceSpecificConfiguration));
    default:
        return DeviceNode::getRegisterRawData(registerType, status);
    }
}

/**
 * @brief Sets the raw data of the register specified by registerType
 * 
 * 
 * @param registerType 		The type of the register to set the raw data to
 * @param value 			The raw data to set
 * @return  				The status of the operation
 * 								- Status::Ok if the operation was successful
 *                              - Status::RegisterNotSupported if this device doesn't implement the register
 */
Status BarDisplay::setRegisterRawData(DeviceRegisterType registerType, std::span<const std::byte> value)
{
    switch (registerType)
    {
    case DeviceRegisterType::DeviceSpecificConfiguration:
        assert(value.size() == sizeof(m_deviceSpecificConfiguration));
        m_deviceSpecificConfiguration = *reinterpret_cast<const DeviceSpecificConfiguration*>(value.data());
        return Status::Ok;
    default:
        return DeviceNode::setRegisterRawData(registerType, value);
    }
}

