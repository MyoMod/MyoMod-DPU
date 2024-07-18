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

template <typename T>
class LogNode : public AlgorithmicNode
{
public:
    LogNode(uint32_t nInputs, std::string_view longId, std::string_view shortId)
     : AlgorithmicNode(),
         m_shortId(shortId)
    {
        for (uint32_t i = 0; i < nInputs; i++)
        {
            m_TinputPorts.push_back(std::make_shared<InputPort<T>>());
            m_inputPorts.push_back(m_TinputPorts.back());
        }

        // check if Ids are valid
        std::array<std::string_view, 2> strings = {longId, m_shortId};
        for (const auto& str : strings)
        {
            assert(str.size() > 0);
            // they may not contain newlines
            assert(str.find("\n") == std::string::npos);
        }
        
        // print information on node (ids + data type) to RTT
        const std::type_info  &ti = typeid(T);
        int status;
        std::string typeName = abi::__cxa_demangle(ti.name(), NULL, NULL, &status);
        SEGGER_RTT_printf(1, "{node} %s:%s:%d:%s\n", longId.data(), m_shortId.c_str(), nInputs, typeName.c_str());
    
    }
    ~LogNode() = default;

    void process() override
    {
        std::vector<T> inData;
        bool allValid = true;
        for (const auto& inputPort : m_TinputPorts)
        {
            if (inputPort->isValid())
            {
                inData.emplace_back(inputPort->getValue());
                allValid &= true;
            }
        }
        const unsigned char *binData = reinterpret_cast<const unsigned char *> (&inData[0]);
        std::string encodedData = base64_encode(binData, inData.size() * sizeof(inData[0]));
        SEGGER_RTT_printf(1, "%s:%s\n", m_shortId.c_str(), encodedData.c_str());
    }


private:
    std::vector<std::shared_ptr<InputPort<T>> > m_TinputPorts;
    std::string m_shortId;
};