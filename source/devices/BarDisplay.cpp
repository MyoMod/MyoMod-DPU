#include "BarDisplay.h"
#include <cstdint>
#include "SEGGER_RTT.h"



BarDisplay::BarDisplay(std::array<char, 10> id) : 
    DeviceNode(id, 
        std::array<char, 10>({'B','a','r','D','i','s','p','7','C','h'})),
    m_outputPort(std::make_shared<OutputPort<std::array<uint8_t, 4>>>()),
    m_hostInStorage(std::make_shared<std::array<uint8_t, 4>>()),
    m_inputPort(std::make_shared<InputPort<std::array<uint8_t, 7>>>(std::array<uint8_t, 7>({0,0,0,0,0,0,0}))),
    m_hostOutStorage({
        std::make_shared<std::array<uint8_t, 7>>()
    })
{
    m_outputPorts.push_back(m_outputPort);
    m_inputPorts.push_back(m_inputPort);
}

void BarDisplay::processInData()
{
    // TODO: Do some validation here
    m_outputPort->setValue(*m_hostInStorage);
}

void BarDisplay::processOutData()
{
    if (m_inputPort->isValid())
    {
        *(m_hostOutStorage[m_activeBuffer]) = m_inputPort->getValue();
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

