#include "ServoHand.h"
#include <cstdint>
#include "SEGGER_RTT.h"



ServoHand::ServoHand(std::array<char, 10> id) : 
    DeviceNode(id, 
        std::array<char, 10>({'S','e','r','v','o',' ','H','a','n','d'})),
    m_currentPos(std::make_shared<OutputPort<ServoHandHInData_t>>()),
    m_hostInStorage(std::make_shared<InputStorage<ServoHandHInData_t>>()),
    m_targetPos(std::make_shared<InputPort<ServoHandHInData_t>>(0)),
    m_hostOutStorage({
        std::make_shared<ServoHandHInData_t>(90),
        std::make_shared<ServoHandHInData_t>(90)
    })
{
    m_inputPorts.push_back(m_targetPos);
    m_outputPorts.push_back(m_currentPos);
}

void ServoHand::processInData()
{
    // TODO: Do some validation here
    auto& data = m_hostInStorage->data;
    m_currentPos->setValue(data);
    m_currentPos->setValid(true);

    //SEGGER_RTT_Write(1, data.data(), data.size());
}

void ServoHand::processOutData()
{
    auto data = m_hostOutStorage[m_activeBuffer].get();
    m_activeBuffer = !m_activeBuffer;
    *data = m_targetPos->getValue();
}

DeviceNodeStorage ServoHand::getNodeStorage()
{
    DeviceNodeStorage storage;
    storage.inStorage = reinterpret_pointer_cast<std::byte>(m_hostInStorage);
    storage.inSize = sizeof(*m_hostInStorage);
    storage.outStorage[0] = reinterpret_pointer_cast<std::byte>(m_hostOutStorage[0]);
    storage.outStorage[1] = reinterpret_pointer_cast<std::byte>(m_hostOutStorage[1]);
    storage.outSize = sizeof(*(m_hostOutStorage[0]));

    return storage;
}

