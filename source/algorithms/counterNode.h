/**
 * @file counterNode.h
 * @author Leon Farchau (leon2225)
 * @brief A simple algorithm that counts up a value with a given step size and start value
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
class LinearCounterNode : public AlgorithmicNode
{
protected :
    std::shared_ptr<OutputPort<T>> m_y;
    T m_a;
public:
    LinearCounterNode(T a, T b): 
        AlgorithmicNode(),
        m_y(std::make_shared<OutputPort<T>>()),
        m_a(a)
    {
        m_y->setValue(b);
        m_outputPorts.push_back(m_y);
    }
    ~LinearCounterNode() = default;

    void process() override{
        m_y->setValue(m_y->getValue() + m_a);
    }
};
