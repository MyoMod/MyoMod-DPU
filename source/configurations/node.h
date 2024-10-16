/**
 * @file node.h
 * @author Leon Farchau (leon2225)
 * @brief A Node is the basic building block of the internal structure. It is used to process data and can be linked to other nodes via Ports.
 *          A node does not have any knowledge about the structure it is part of. It only knows its own input and/or output ports and may link them to other nodes.

 * @version 0.1
 * @date 19.06.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <vector>
#include <span>
#include <memory>
#include <array>
#include <tuple>
#include <cstdint>
#include <iostream>
#include "SEGGER_RTT.h"

#include "port.h"

enum class NodeCategory
{
    None,
    DeviceNodes,
    AlgorithmNodes
};

struct DeviceNodeStorage
{
    std::shared_ptr<std::byte> inStorage;
    size_t inSize;
    std::array<std::shared_ptr<std::byte>, 2> outStorage;
    size_t outSize;
};

struct DeviceIdentifier
{
    std::array<char, 10> type;
    std::array<char, 10> id;

	friend bool operator== (const DeviceIdentifier& lhs, const DeviceIdentifier& rhs)
	{
		return lhs.type == rhs.type &&
			   lhs.id == rhs.id;
	}
    void print() const
    {
        char str[11];
        std::copy(type.begin(), type.end(), str);
        SEGGER_RTT_printf(0, "%s:", str);
        std::copy(id.begin(), id.end(), str);
        SEGGER_RTT_printf(0, "%s", str);
    }
};


class BaseNode
{

public:
    virtual ~BaseNode() = default;

    virtual bool linkInputPort(uint32_t inputIndex, BaseNode* outputNode, uint32_t outputIndex);
    std::shared_ptr<BaseOutputPort> getOutputPort(uint32_t index) const;
protected:
    
    std::vector<std::shared_ptr<BaseOutputPort>> m_outputPorts;
    std::vector<std::shared_ptr<BaseInputPort>> m_inputPorts;
};


/*
class AdditonNode : public AlgorithmicNode
{
public:
    AdditonNode(bool subtraction);
    ~AdditonNode() = default;

    void process() override;
protected :
    std::shared_ptr<OutputPort<int>> m_outputPort;
    std::shared_ptr<InputPort<int>> m_inputPortA;
    std::shared_ptr<InputPort<int>> m_inputPortB;

    bool m_subtraction = false;
};

template <typename T_in, typename T_out>
class CastNode : public AlgorithmicNode
{
public:
    CastNode():
        m_outputPort(std::make_shared<OutputPort<T_out>>()),
        m_inputPort(std::make_shared<InputPort<T_in>>(-42))
    {
        m_outputPorts.push_back(m_outputPort);
        m_inputPorts.push_back(m_inputPort);
    }
    ~CastNode() = default;

    void process() override {
        if (m_inputPort->isValid())
        {
            m_outputPort->setValue(static_cast<T_out>(m_inputPort->getValue()));
        }
        return;
    }
protected:
    std::shared_ptr<OutputPort<T_out>> m_outputPort;
    std::shared_ptr<InputPort<T_in>> m_inputPort;
};

class ArraySelectorNode : public AlgorithmicNode
{
public:
    ArraySelectorNode(uint32_t index = 0):
        m_outputPort{std::make_shared<OutputPort<int>>()},
        m_inputPort{std::make_shared<InputPort<std::array<int, 10>>>()},
        m_index{index}
    {
        m_outputPorts.push_back(m_outputPort);
        m_inputPorts.push_back(m_inputPort);
    }
    ~ArraySelectorNode() = default;

    void process() override {
        if (m_inputPort->isValid())
        {
            m_outputPort->setValue(m_inputPort->getValue()[m_index]);
        }
        return;
    }

    void setIndex(uint32_t index) {m_index = index;};
protected:
    std::shared_ptr<OutputPort<int>> m_outputPort;
    std::shared_ptr<InputPort<std::array<int, 10>>> m_inputPort;
    uint32_t m_index;
};

class AdvancedTestNode : public AlgorithmicNode
{
public:
    AdvancedTestNode(const std::array<int, 3>& numericalParameters, const std::array<bool, 3>& booleanParameters):
        m_outputPort{std::make_shared<OutputPort<int>>()},
        m_inputPort{std::make_shared<InputPort<int>>()},
        m_numericalParameters{numericalParameters},
        m_booleanParameters{booleanParameters}
    {
        m_outputPorts.push_back(m_outputPort);
        m_inputPorts.push_back(m_inputPort);
    }
    ~AdvancedTestNode() = default;

    void process() override {
        if (m_inputPort->isValid())
        {
            m_outputPort->setValue(m_inputPort->getValue() + m_numericalParameters[0]);
        }
        return;
    }
protected:
    std::shared_ptr<OutputPort<int>> m_outputPort;
    std::shared_ptr<InputPort<int>> m_inputPort;
    std::array<int, 3> m_numericalParameters;
    std::array<bool, 3> m_booleanParameters;
};
*/
/*
class ArrayInputNode : public DeviceNode
{
public:
    ArrayInputNode(std::array<char, 10> id):
        DeviceNode(id),
        m_outputPort(std::make_shared<OutputPort<std::array<int, 10>>>())
    {
        m_outputPorts.push_back(m_outputPort);
    }
    ~ArrayInputNode() = default;

    void processOutData() override {}

    void processInData() override {
        std::array<int, 10> array;
        for (int i = 0; i < 10; i++)
        {
            array[i] = i + m_counter;
        }
        m_outputPort->setValue(array);
        return;
    }
protected:
    int m_counter = 0;
    std::shared_ptr<OutputPort<std::array<int, 10>>> m_outputPort;
};

class FloatInputNode : public DeviceNode
{
public:
    FloatInputNode(std::array<char, 10> id):
        DeviceNode(id),
        m_outputPort(std::make_shared<OutputPort<float>>())
    {
        m_outputPorts.push_back(m_outputPort);
    }
    ~FloatInputNode() = default;

    void processOutData() override {}

    void processInData() override {
        m_outputPort->setValue(m_counter+=0.5);
        return;
    }
protected:
    float m_counter = 0;
    std::shared_ptr<OutputPort<float>> m_outputPort;
};

class InputNode : public DeviceNode
{
public:
    InputNode(std::array<char, 10> id);
    ~InputNode() = default;

    void processOutData() override {};

    void processInData() override {
        m_outputPort->setValue(m_counter++);
        return;
    }
protected:
    int m_counter = 0;
    std::shared_ptr<OutputPort<int>> m_outputPort;
};

class OutputNode : public DeviceNode
{
public:
    OutputNode(std::array<char, 10> id);
    ~OutputNode() = default;

    void processInData() override {};

    void processOutData() override {
        // Print the value of the input port
        if (m_inputPort->isValid())
        {
            std::cout << m_id.data() << ": " << m_inputPort->getValue() << std::endl;
        }
        return;
    }
protected:
    std::shared_ptr<InputPort<int>> m_inputPort;
};
*/