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
#include "math.h"

#include "algorithmicNode.h"

class TestAlgorithm : public AlgorithmicNode
{
protected :
    std::array<std::shared_ptr<OutputPort<uint8_t>>, 7> m_barOutputPorts;
    std::array<std::shared_ptr<InputPort<bool>>, 4> m_buttonInputPorts;
public:
    TestAlgorithm()
    {
        for (size_t i = 0; i < m_barOutputPorts.size(); i++)
        {
            m_barOutputPorts[i] = std::make_shared<OutputPort<uint8_t>>();
            m_outputPorts.push_back(m_barOutputPorts[i]);
        }
        for (size_t i = 0; i < m_buttonInputPorts.size(); i++)
        {
            m_buttonInputPorts[i] = std::make_shared<InputPort<bool>>(false);
            m_inputPorts.push_back(m_buttonInputPorts[i]);
        }
    }
    ~TestAlgorithm() = default;

    void process() override{
        bool allValid = true;
        for (const auto& button : m_buttonInputPorts)
        {
            allValid &= button->isValid();
        }
        if (allValid)
        {
            static float counter = 0;
            counter += 0.01;
            static uint32_t counter2 = 0;

            uint8_t value = 0;
            for (size_t i = 0; i < m_buttonInputPorts.size(); i++)
            {
                value += m_buttonInputPorts[i]->getValue() * (i + 1) * 20;
            }

            m_barOutputPorts[0]->setValue(100 * m_buttonInputPorts[0]->getValue());
            m_barOutputPorts[1]->setValue(100 * m_buttonInputPorts[2]->getValue());
            m_barOutputPorts[2]->setValue(sinf(counter * 2*3.14) * 50 + 50);
            m_barOutputPorts[3]->setValue((counter2++)%100);
            m_barOutputPorts[4]->setValue(value);
            m_barOutputPorts[5]->setValue(100 * m_buttonInputPorts[1]->getValue());
            m_barOutputPorts[6]->setValue(100 * m_buttonInputPorts[3]->getValue());
        }    
    }
};
