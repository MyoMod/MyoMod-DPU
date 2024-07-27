#include "EMGSensor6Chn.h"
#include <cstdint>
#include "SEGGER_RTT.h"
#include "charArray.h"



EMGSensor6Chn::EMGSensor6Chn(std::array<char, 10> id) : 
    DeviceNode(id, 
        idArr("EMGSens6Ch")),
    m_emgPorts(),
    m_hostInStorage(std::make_shared<InputStorage<std::array<std::array<uint32_t,15>,6>>>()),
    m_hostOutStorage({
        std::make_shared<uint8_t>(0),
        std::make_shared<uint8_t>(0)
    })
{
    for(size_t i = 0; i < m_emgPorts.size(); i++)
    {
        m_emgPorts[i] = std::make_shared<OutputPort<std::array<uint32_t, 15>>>();
        m_outputPorts.push_back(m_emgPorts[i]);
    }
}

void EMGSensor6Chn::processInData()
{
    // TODO: Do some validation here
    auto* data = &(m_hostInStorage->data);
    std::array<uint32_t, 15> temp;
    for (size_t i = 0; i < m_emgPorts.size(); i++)
    {
        memcpy(temp.data(), (*data)[i].data(), sizeof(temp));
        m_emgPorts[i]->setValue(temp);
        m_emgPorts[i]->setValid(true);
    }
}

void EMGSensor6Chn::processOutData()
{
    // do nothing, as the sensor has no real outdata
}

DeviceNodeStorage EMGSensor6Chn::getNodeStorage()
{
    DeviceNodeStorage storage;
    storage.inStorage = reinterpret_pointer_cast<std::byte>(m_hostInStorage);
    storage.inSize = sizeof(*m_hostInStorage);
    storage.outStorage[0] = reinterpret_pointer_cast<std::byte>(m_hostOutStorage[0]);
    storage.outStorage[1] = reinterpret_pointer_cast<std::byte>(m_hostOutStorage[1]);
    storage.outSize = sizeof(*(m_hostOutStorage[0]));

    return storage;
}

