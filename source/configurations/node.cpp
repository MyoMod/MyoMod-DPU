/**
 * @file node.cpp
 * @author Leon Farchau (leon2225)
 * @brief A Node is the basic building block of the internal structure. It is used to process data and can be linked to other nodes via Ports.
 *          A node does not have any knowledge about the structure it is part of. It only knows its own input and/or output ports and may link them to other nodes.
 * @version 0.1
 * @date 20.06.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "node.h"

bool BaseNode::linkInputPort(uint32_t inputIndex, BaseNode* outputNode, uint32_t outputIndex)
{
    if (inputIndex >= m_inputPorts.size())
    {
        return false;
    }

    if (outputIndex >= outputNode->m_outputPorts.size())
    {
        return false;
    }

    return m_inputPorts[inputIndex]->linkTo(outputNode->getOutputPort(outputIndex));
}

std::shared_ptr<BaseOutputPort> BaseNode::getOutputPort(uint32_t index) const
{
    if (index >= m_outputPorts.size())
    {
        return nullptr;
    }

    return std::shared_ptr<BaseOutputPort>{m_outputPorts[index]};
}

AdditonNode::AdditonNode(bool substract):
    m_outputPort(std::make_shared<OutputPort<int>>()),
    m_inputPortA(std::make_shared<InputPort<int>>(-42)),
    m_inputPortB(std::make_shared<InputPort<int>>(-42)),
    m_subtraction(substract)
{
    m_outputPorts.push_back(m_outputPort);
    m_inputPorts.push_back(m_inputPortA);
    m_inputPorts.push_back(m_inputPortB);
}

void AdditonNode::process()
{
    if (m_inputPortA->isValid() && m_inputPortB->isValid())
    {
        m_outputPort->setValue(m_inputPortA->getValue() + (m_subtraction?-1:1)*m_inputPortB->getValue());
        m_outputPort->setValid(true);
    }
    else
    {
        m_outputPort->setValid(false);
    }
}