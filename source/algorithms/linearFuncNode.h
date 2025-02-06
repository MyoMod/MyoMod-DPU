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
    std::shared_ptr<InputPort<T>> m_x;
    T m_a;
    T m_b;
public:
    LinearFunctionNode(T a, T b): 
        AlgorithmicNode(),
        m_y(std::make_shared<OutputPort<T>>()),
        m_x(std::make_shared<InputPort<T>>()),
        m_a(a),
        m_b(b)
    {
        m_outputPorts.push_back(m_y);
        m_inputPorts.push_back(m_x);
    }
    ~LinearFunctionNode() = default;

    void process() override{
        m_y->setValue(m_x->getValue() * m_a + m_b);
    }
};
