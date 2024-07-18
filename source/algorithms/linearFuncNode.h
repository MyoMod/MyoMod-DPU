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

template<typename T>
class LinearFunctionNode : public AlgorithmicNode
{
protected :
    std::shared_ptr<OutputPort<T>> m_y;
    T m_a;
public:
    LinearFunctionNode(T a, T b): 
        AlgorithmicNode(),
        m_y(std::make_shared<OutputPort<T>>()),
        m_a(a)
    {
        m_y->setValue(b);
        m_outputPorts.push_back(m_y);
    }
    ~LinearFunctionNode() = default;

    void process() override{
        m_y->setValue(m_y->getValue() + m_a);
    }
};
