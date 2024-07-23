#pragma once

#include "deviceNode.h"


typedef uint8_t ServoHandHInData_t;
typedef uint8_t ServoHandHOutData_t;

class ServoHand : public DeviceNode {
public:
    // Constructor
    ServoHand(std::array<char, 10> id);

    // Destructor
    ~ServoHand() = default;

    void processInData() override;
    void processOutData() override;
    DeviceNodeStorage getNodeStorage() override;
private:
    std::shared_ptr<OutputPort<ServoHandHInData_t>> m_currentPos;
    std::shared_ptr<InputStorage<ServoHandHInData_t>> m_hostInStorage;

    std::shared_ptr<InputPort<ServoHandHInData_t>> m_targetPos;
    std::array<std::shared_ptr<ServoHandHInData_t>,2> m_hostOutStorage;

    int32_t m_statusByte;
};


