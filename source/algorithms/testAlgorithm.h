/**
 * @file testAlgorithm.h
 * @author Leon Farchau (leon2225)
 * @brief A simple algorithm for testing the core system
 *          that mirrors the buttons of the bardisplay
 *          to the leds of the bardisplay
 *          
 * @version 0.1
 * @date 15.07.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <array>

#include "algorithmicNode.h"

class TestAlgorithm : public AlgorithmicNode
{
public:
    TestAlgorithm():
        m_outputPort{std::make_shared<OutputPort<std::array<uint8_t, 7>>>()},
        m_buttonInputPort{std::make_shared<InputPort<std::array<uint8_t, 4>>>()}
    {
        m_outputPorts.push_back(m_outputPort);
        m_inputPorts.push_back(m_buttonInputPort);
    }
    ~TestAlgorithm() = default;

    void process() override{
        if (m_buttonInputPort->isValid())
        {
            std::array<uint8_t, 7> output;
            output[0] = 100 * m_buttonInputPort->getValue()[0];
            output[1] = 100 * m_buttonInputPort->getValue()[1];
            output[2] = 100 * m_buttonInputPort->getValue()[2];
            output[3] = 100 * m_buttonInputPort->getValue()[3];
            m_outputPort->setValue(output);
        }
        return;
    
    }
protected :
    std::shared_ptr<OutputPort<std::array<uint8_t, 7>>> m_outputPort;
    std::shared_ptr<InputPort<std::array<uint8_t, 4>>> m_buttonInputPort;
};
