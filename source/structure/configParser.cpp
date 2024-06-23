/**
 * @file configParser.cpp
 * @author Leon Farchau (leon2225)
 * @brief 
 * @version 0.1
 * @date 21.06.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <fstream>
#include "configParser.h"

// Data structures

// private functions
static NodeType getNodeType(const char* const type);
static bool getID(const NodeData& nodeData, std::array<char, 10>& ID);
static bool createPortDescriptor(std::string_view stringDescriptor, PortDescriptor& PortDescriptor);

static std::unique_ptr<DeviceNode> createSimpleInputNode(const NodeData& nodeData);
static std::unique_ptr<DeviceNode> createSimpleOutputNode(const NodeData& nodeData);
static std::unique_ptr<AlgorithmicNode> createAdvancedTestNode(const NodeData& nodeData);



NodesTuple ConfigParser::parseConfig(std::string_view configPath, uint32_t configId)
{
    // Reset parsed data
    m_devices.clear();
    m_algorithms.clear();
    m_currentConfigId = 0;


    // initialize parser
    m_configId = configId;
    lwjson_stream_init(&m_parser, speceficTokenParser);

    // Open file
    std::ifstream configFile = std::ifstream(configPath.data());
    if (!configFile.is_open())
    {
        throw std::runtime_error("Could not open file: " + std::string(configPath));
    }

    // Parse file
    // iterate over all characters in the file
    lwjsonr_t parseRes;
    while (configFile.good())
    {
        char c = configFile.get();
        parseRes = lwjson_stream_parse(&m_parser, c);
        if (parseRes == lwjsonSTREAMINPROG) {} 
        else if (parseRes == lwjsonSTREAMWAITFIRSTCHAR) {
            printf("Waiting first character\r\n");
        } 
        else if (parseRes == lwjsonSTREAMDONE) {
            printf("Done\r\n");
            break;
        } 
        else {
            printf("Error\r\n");
            m_parseError = true;
            break;
        }

        if (m_parseError)
        {
            std::cerr << "Parse error" << std::endl;
            break;
        }
    }

    if (m_parseError)
    {
        return std::make_tuple(std::vector<std::unique_ptr<DeviceNode>>{}, std::vector<std::unique_ptr<AlgorithmicNode>>{});
    }

    // Move nodes to vectors 
    return std::make_tuple(std::move(m_devices), std::move(m_algorithms));
}

void ConfigParser::speceficTokenParser(lwjson_stream_parser_t* jsp, lwjson_stream_type_t type)
{
    ConfigParser& parser = ConfigParser::getInstance();

    // Handle config id
    if (jsp->stack_pos == 1 && type == LWJSON_STREAM_TYPE_OBJECT_END)
    {
        // new config ended, increase config id
        std::cout << "^Config id: " << parser.m_currentConfigId << std::endl;
        parser.m_currentConfigId++;
    }

    // Check if the current config is the one we are looking for
    if (parser.m_currentConfigId != parser.m_configId)
    {
        return;
    }

    // Handle ConfigSection
    if (lwjson_stack_seq_4(jsp, 0, ARRAY, OBJECT, KEY, ARRAY)
         && type == LWJSON_STREAM_TYPE_ARRAY
         && jsp->stack_pos == 4)
    {

        // set current node category
        if (strcmp(jsp->stack[2].meta.name, "deviceNodes") == 0)
        {
            parser.m_currentConfigSection = ConfigSection::DeviceNodes;
        }
        else if (strcmp(jsp->stack[2].meta.name, "algorithmicNodes") == 0)
        {
            parser.m_currentConfigSection = ConfigSection::AlgorithmicNodes;
        }
        else
        {
            parser.m_currentConfigSection = ConfigSection::None;
        }

        // print section
        std::cout << "ConfigSection: " << jsp->stack[2].meta.name << std::endl;
    }
    else if (lwjson_stack_seq_4(jsp, 0, ARRAY, OBJECT, KEY, OBJECT)
         && type == LWJSON_STREAM_TYPE_OBJECT
         && jsp->stack_pos == 4)
    {
        if (strcmp(jsp->stack[2].meta.name, "links") == 0)
        {
            parser.m_currentConfigSection = ConfigSection::Links;
        }
        else
        {
            parser.m_currentConfigSection = ConfigSection::None;
        }

        // print section
        std::cout << "ConfigSection: " << jsp->stack[2].meta.name << std::endl;
    }

    // Handle NodeData

    // New node started
    else if (lwjson_stack_seq_5(jsp, 0, ARRAY, OBJECT, KEY, ARRAY, OBJECT)
         && type == LWJSON_STREAM_TYPE_OBJECT
         && jsp->stack_pos == 4)
    {
        // reset current node data
        parser.m_currentNodeData = NodeData{};
    }

    // Node field scalar string
    else if (lwjson_stack_seq_6(jsp, 0, ARRAY, OBJECT, KEY, ARRAY, OBJECT, KEY)
         && jsp->stack_pos == 6)
    {
        // type field
        if (type == LWJSON_STREAM_TYPE_STRING && strcmp(jsp->stack[5].meta.name, "type") == 0)
        {
            parser.m_currentNodeData.type = getNodeType(jsp->data.str.buff);
            std::cout << "Type: " << jsp->data.str.buff << std::endl;
        }

        // scalar string field
        else if (type == LWJSON_STREAM_TYPE_STRING)
        {
            parser.m_currentNodeData.scalarParams[jsp->stack[5].meta.name] = jsp->data.str.buff;
            std::cout << "Scalar(s): " << jsp->stack[5].meta.name << " " << jsp->data.str.buff << std::endl;
        }

        // scalar number field
        else if (type == LWJSON_STREAM_TYPE_NUMBER)
        {
            parser.m_currentNodeData.scalarParams[jsp->stack[5].meta.name] = jsp->data.str.buff;
            std::cout << "Scalar(n): " << jsp->stack[5].meta.name << " " << jsp->data.str.buff << std::endl;
        }

        // scalar boolean field
        else if (type == LWJSON_STREAM_TYPE_TRUE || type == LWJSON_STREAM_TYPE_FALSE)
        {
            parser.m_currentNodeData.scalarParams[jsp->stack[5].meta.name] = type == LWJSON_STREAM_TYPE_TRUE?"1":"0";
            std::cout << "Scalar(b): " << jsp->stack[5].meta.name << " " << jsp->data.str.buff << std::endl;
        }
    }

    // Node field array
    else if (lwjson_stack_seq_7(jsp, 0, ARRAY, OBJECT, KEY, ARRAY, OBJECT, KEY, ARRAY)
         && jsp->stack_pos == 7)
    {
        static std::string lastArrayName;
        // array field
        if (type == LWJSON_STREAM_TYPE_ARRAY)
        {
            lastArrayName = jsp->data.str.buff;
            parser.m_currentNodeData.arrayParams[lastArrayName] = std::vector<std::string>{};
            std::cout << "Array: " << lastArrayName << ":" << std::endl;
        }

        // scalar string field
        if (type == LWJSON_STREAM_TYPE_STRING || type == LWJSON_STREAM_TYPE_NUMBER)
        {
            parser.m_currentNodeData.arrayParams[lastArrayName].push_back(jsp->data.str.buff);
            std::cout << " - " << jsp->data.str.buff << std::endl;
        }

        // scalar boolean field
        else if (type == LWJSON_STREAM_TYPE_TRUE || type == LWJSON_STREAM_TYPE_FALSE)
        {
            parser.m_currentNodeData.arrayParams[lastArrayName].push_back(type == LWJSON_STREAM_TYPE_TRUE?"1":"0");
            std::cout << " - " << jsp->data.str.buff << std::endl;
        }
    }

    // Node is finished
    else if (lwjson_stack_seq_4(jsp, 0, ARRAY, OBJECT, KEY, ARRAY)
         && type == LWJSON_STREAM_TYPE_OBJECT_END
         && jsp->stack_pos == 4)
    {
        switch (parser.m_currentConfigSection)
        {
        case ConfigSection::DeviceNodes:
            parser.m_devices.push_back(parser.createDeviceNode(parser.m_currentNodeData));

            // check if creation was successful
            if (parser.m_devices.back() == nullptr)
            {
                parser.m_parseError = true;
                std::cerr << "Could not create device node" << std::endl;
            }
            break;
        case ConfigSection::AlgorithmicNodes:
            parser.m_algorithms.push_back(parser.createAlgorithmicNode(parser.m_currentNodeData));

            // check if creation was successful
            if (parser.m_algorithms.back() == nullptr)
            {
                parser.m_parseError = true;
                std::cerr << "Could not create algorithmic node" << std::endl;
            }
            break;
        default:
            break;
        }
    }

    // Handle links
    else if (lwjson_stack_seq_5(jsp, 0, ARRAY, OBJECT, KEY, OBJECT, KEY)
         && type == LWJSON_STREAM_TYPE_STRING
         && jsp->stack_pos == 5
         && parser.m_currentConfigSection == ConfigSection::Links)
    {
        std::cout << "Link: " << jsp->data.str.buff << "->" << jsp->stack[4].meta.name;
        PortDescriptor inputPort;
        PortDescriptor outputPort;
        if (createPortDescriptor(jsp->stack[4].meta.name, inputPort) && createPortDescriptor(jsp->data.str.buff, outputPort))
        {
            if (!parser.linkNodes(inputPort, outputPort))
            {
                parser.m_parseError = true;
                std::cout << " - error" << std::endl;
            }
            else
            {
                std::cout << " - linked" << std::endl;
            }
        }
    }
}

std::unique_ptr<DeviceNode> ConfigParser::createDeviceNode(const NodeData& nodeData)
{
    switch (nodeData.type)
    {
    case NodeType::InputNode:       // fallthrough
    case NodeType::FloatInputNode:  // fallthrough
    case NodeType::ArrayInputNode:  
        return createSimpleInputNode(nodeData);
    case NodeType::OutputNode:
        return createSimpleOutputNode(nodeData);
    default:
        return nullptr;
    }
}

std::unique_ptr<AlgorithmicNode> ConfigParser::createAlgorithmicNode(const NodeData& nodeData)
{
    switch (nodeData.type)
    {
    case NodeType::Float2IntNode:   // fallthrough - no specific params needed
    case NodeType::Int2FloatNode:
        return NodeFactory::createAlgorithmicNode(nodeData.type);
    case NodeType::AdditonNode:
        if (nodeData.scalarParams.find("subtraction") == nodeData.scalarParams.end())
        {
            return nullptr;
        }
        return NodeFactory::createAlgorithmicNode(nodeData.type, std::stoi(nodeData.scalarParams.at("subtraction")));
    case NodeType::ArraySelectorNode:
        if (nodeData.scalarParams.find("index") == nodeData.scalarParams.end())
        {
            return nullptr;
        }
        return NodeFactory::createAlgorithmicNode<uint32_t>(NodeType::ArraySelectorNode, std::stoi(nodeData.scalarParams.at("index")));

    default:
        return nullptr;
    }
}


/**
 * @brief Link the given inputPort to the given outputPort.
 * 
 * @param inputPort         PortDescriptor of the input port
 * @param outputPort        PortDescriptor of the output port
 * @return true             Ports were linked
 * @return false            Ports could not be linked
 */
bool ConfigParser::linkNodes(const PortDescriptor& inputPort, const PortDescriptor& outputPort)
{
    auto nDeviceNodes = m_devices.size();
    auto nAlgorithmNodes = m_algorithms.size();
    bool valid = true;

    valid &= (inputPort.category == NodeCategory::DeviceNodes)?(inputPort.nodeIndex < nDeviceNodes):(inputPort.nodeIndex < nAlgorithmNodes);
    valid &= (outputPort.category == NodeCategory::DeviceNodes)?(outputPort.nodeIndex < nDeviceNodes):(outputPort.nodeIndex < nAlgorithmNodes);

    if (!valid)
    {
        return false;
    }

    BaseNode* outputNode;
    if(outputPort.category == NodeCategory::DeviceNodes)
    {
        outputNode = m_devices[outputPort.nodeIndex].get();
    }
    else
    {
        outputNode = m_algorithms[outputPort.nodeIndex].get();
    }

    BaseNode* inputNode;
    if(inputPort.category == NodeCategory::DeviceNodes)
    {
        inputNode = m_devices[inputPort.nodeIndex].get();
    }
    else
    {
        inputNode = m_algorithms[inputPort.nodeIndex].get();
    }
    return inputNode->linkInputPort(inputPort.nodePort, outputNode, outputPort.nodePort);
}

// private functions
static NodeType getNodeType(const char* const type)
{
    if (strcmp(type, "InputNode") == 0)
    {
        return NodeType::InputNode;
    }
    else if (strcmp(type, "FloatInputNode") == 0)
    {
        return NodeType::FloatInputNode;
    }
    else if (strcmp(type, "ArrayInputNode") == 0)
    {
        return NodeType::ArrayInputNode;
    }
    else if (strcmp(type, "Float2IntNode") == 0)
    {
        return NodeType::Float2IntNode;
    }
    else if (strcmp(type, "AdditonNode") == 0)
    {
        return NodeType::AdditonNode;
    }
    else if (strcmp(type, "ArraySelectorNode") == 0)
    {
        return NodeType::ArraySelectorNode;
    }
    else if (strcmp(type, "OutputNode") == 0)
    {
        return NodeType::OutputNode;
    }
    else
    {
        return NodeType::None;
    }
}


/**
 * @brief Create a Simple Input Node object from the given NodeData.
 *          If the nodeData does not contain the ID field, nullptr is returned.
 * 
 * @param nodeData                      NodeData to create the node from 
 * @return std::unique_ptr<DeviceNode>  Created node or nullptr if the ID field is missing
 */
static std::unique_ptr<DeviceNode> createSimpleInputNode(const NodeData& nodeData)
{
    std::array<char, 10> ID;
    if (!getID(nodeData, ID))
    {
        return nullptr;
    }
    return NodeFactory::createDeviceNode(nodeData.type, ID);
}

/**
 * @brief Create a Simple Output Node object from the given NodeData.
 *          If the nodeData does not contain the ID field, nullptr is returned.
 * 
 * @param nodeData                      NodeData to create the node from 
 * @return std::unique_ptr<DeviceNode>  Created node or nullptr if the ID field is missing
 */
static std::unique_ptr<DeviceNode> createSimpleOutputNode(const NodeData& nodeData)
{
    std::array<char, 10> ID;
    if (!getID(nodeData, ID))
    {
        return nullptr;
    }
    return NodeFactory::createDeviceNode(nodeData.type, ID);
}

/*
static std::unique_ptr<AlgorithmicNode> createAdvancedTestNode(const NodeData& nodeData)
{
    // Parameters; std::array<int, 3> numericalParameters, std::array<bool, 3> booleanParameters

    if (nodeData.arrayParams.find("numericalParameters") == nodeData.arrayParams.end() || nodeData.arrayParams.find("booleanParameters") == nodeData.arrayParams.end())
    {
        return nullptr;
    }

    std::array<int, 3> numericalParameters;
    std::array<bool, 3> booleanParameters;

    for (int i = 0; i < 3; i++)
    {
        numericalParameters[i] = std::stoi(nodeData.arrayParams.at("numericalParameters")[i]);
        booleanParameters[i] = nodeData.arrayParams.at("booleanParameters")[i] == "1";
    }

    return NodeFactory::createAlgorithmicNode(NodeType::AdvancedTestNode, numericalParameters, booleanParameters);
}
*/

/**
 * @brief Get the ID from the given NodeData.
 *          If the ID field is or has wrong length missing, false is returned.
 * 
 * @param nodeData                  NodeData to get the ID from
 * @param ID                        ID to store the ID in
 * @return true                     ID was found and valid
 * @return false                    ID was not found or invalid
 */
static bool getID(const NodeData& nodeData, std::array<char, 10>& ID)
{
    if (nodeData.scalarParams.find("ID") == nodeData.scalarParams.end())
    {
        return false;
    }
    const auto& idString = nodeData.scalarParams.at("ID");
    if (idString.size() != 10)
    {
        return false;
    }
    std::copy(idString.begin(), idString.end(), ID.data());
    return true;
}

/**
 * @brief Create a PortDescriptor from the given stringDescriptor.
 * 
 * @param stringDescriptor              String to create the PortDescriptor from
 * @param PortDescriptor                PortDescriptor to store the created PortDescriptor in
 * @return true                         PortDescriptor was created
 * @return false                        Invalid stringDescriptor format
 */
static bool createPortDescriptor(std::string_view stringDescriptor, PortDescriptor& portDescriptor)
{
    if (stringDescriptor.size() < 4)
    {
        return false;
    }
    char category = stringDescriptor[0];
    // uppercase the category
    if (category >= 'a' && category <= 'z')
    {
        category -= 32;
    }

    // get seperated values (seperated by ':')
    size_t seperatorPos = stringDescriptor.find(':');
    if (seperatorPos == std::string::npos)
    {
        return false;
    }
    portDescriptor.category = category == 'D'?NodeCategory::DeviceNodes:
                            category == 'A'?NodeCategory::AlgorithmNodes:NodeCategory::None;
    portDescriptor.nodeIndex = std::stoi(stringDescriptor.substr(1, seperatorPos - 1).data());
    portDescriptor.nodePort = std::stoi(stringDescriptor.substr(seperatorPos + 1).data());
    return true;
}