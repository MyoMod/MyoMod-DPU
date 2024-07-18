/**
 * @file logNode.h
 * @author Leon Farchau (leon2225)
 * @brief A node that logs the data from the input ports to RTT
 * @version 0.1
 * @date 18.07.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include "algorithmicNode.h"
#include <string>
#include <vector>
#include <typeinfo>
#include <cxxabi.h>
#include "SEGGER_RTT.h"

#include "base64.h"

class LogNode : public AlgorithmicNode
{
public:
    LogNode(std::string_view longId, std::string_view shortId)
     : AlgorithmicNode(),
         m_type(""),
         m_shortId(shortId)
    {


        // check if Ids are valid
        std::array<std::string_view, 2> strings = {longId, m_shortId};
        for (const auto& str : strings)
        {
            assert(str.size() > 0);
            // they may not contain newlines
            assert(str.find("\n") == std::string::npos);
        }
        
        // print information on node to RTT
        SEGGER_RTT_printf(1, "{node} %s:%s\n", longId.data(), m_shortId.c_str());
    
    }
    ~LogNode() = default;

    void process() override
    {
        std::vector<std::byte> inData;
        for (const auto& m_opposingOutputPort : m_opposingOutputPorts)
        {
            if (m_opposingOutputPort == nullptr || !m_opposingOutputPort->isValid())
            {
                return;
            }
            auto data = m_opposingOutputPort->getRawData();
            inData.insert(inData.end(), data.begin(), data.end());
        }

        const unsigned char *binData = reinterpret_cast<const unsigned char *> (&inData[0]);
        std::string encodedData = base64_encode(binData, inData.size() * sizeof(inData[0]));
        SEGGER_RTT_printf(1, "%s:%s\n", m_shortId.c_str(), encodedData.c_str());
    }


    bool linkInputPort(uint32_t inputIndex, BaseNode* outputNode, uint32_t outputIndex) override
    {
        // size is variable, but the assignment order must be accending, 
        //  so that there are no empty slots
        if (inputIndex != m_opposingOutputPorts.size())
        {
            return false;
        }

        auto outputPort = outputNode->getOutputPort(outputIndex);
        if (outputPort == nullptr)
        {
            return false;
        }
        m_opposingOutputPorts.push_back(outputPort);
        if (m_type.empty())
        {
            m_type = outputPort->getDataType();

            // print type information to RTT

            int status;
            std::string typeName = abi::__cxa_demangle(m_type.c_str(), NULL, NULL, &status);
            SEGGER_RTT_printf(1, "{type} %s:%s\n", m_shortId.c_str(), typeName.c_str());
        }
        else if (m_type != outputPort->getDataType())
        {
            return false;
        }
        return true;
    }


private:
    std::vector<std::shared_ptr<BaseOutputPort>> m_opposingOutputPorts;
    std::string m_type;
    std::string m_shortId;
};