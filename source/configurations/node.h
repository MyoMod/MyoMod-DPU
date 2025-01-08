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
    EmbeddedDeviceNodes,
    AlgorithmNodes
};

struct DeviceNodeStorage
{
    std::shared_ptr<std::byte> inStorage;
    size_t inSize;
    std::array<std::shared_ptr<std::byte>, 2> outStorage;
    size_t outSize;
};

/**
 * @brief This identifier is used to identify a device, it does not contain any information about the device itself.
 * 
 */
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