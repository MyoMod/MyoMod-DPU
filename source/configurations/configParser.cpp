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

#include <limits>
#include "SEGGER_RTT.h"

#include "configParser.h"
#include "BarDisplay.h"
#include "testAlgorithm.h"
#include "logNode.h"
#include "linearFuncNode.h"

// Data structures
#ifdef CONFIG_PARSER_DEBUG
    #define PARSE_DEBUG(...) SEGGER_RTT_printf(0, __VA_ARGS__)
#else
    #define PARSE_DEBUG(...)
#endif

// private functions
static bool getID(const NodeData& nodeData, std::array<char, 10>& ID);
static bool createPortDescriptor(std::string_view stringDescriptor, PortDescriptor& PortDescriptor);
template <typename T>
static bool parseParameter(const std::string& parameterName, const NodeData& nodeData, T& parameter);
template <typename T, size_t N>
static bool parseParameter(const std::string& parameterName, const NodeData& nodeData, std::array<T, N>& parameter);
static uint32_t stringToUint(std::string_view str, bool& success);
static int32_t stringToInt(std::string_view str, bool& success);
static float stringToFloat(std::string_view str, bool& success);


ConfigParser::ConfigParser():
        m_currentNodeData{},
        m_configId{0},
        m_currentConfigId{0},
        m_currentConfigSection{ConfigSection::None},
        m_parseError{false},
        m_devices{},
        m_algorithms{}
{

}

NodesTuple ConfigParser::loadConfig(std::string_view configString, uint32_t configId)
{
    // Reset parsed data
    m_devices.clear();
    m_algorithms.clear();
    m_currentConfigId = 0;


    // initialize parser
    m_configId = configId;
    lwjson_stream_init(&m_parser, deepTokenParser);

    // Parse file
    // iterate over all characters in the file
    lwjsonr_t parseRes;
    for (char c : configString)
    {
        parseRes = lwjson_stream_parse(&m_parser, c);
        if (parseRes == lwjsonSTREAMINPROG) {} 
        else if (parseRes == lwjsonSTREAMWAITFIRSTCHAR) {
            PARSE_DEBUG("Parser: Waiting for first character\n");
        } 
        else if (parseRes == lwjsonSTREAMDONE) {
            PARSE_DEBUG("Parser: Done\r\n");
            break;
        } 
        else {
            PARSE_DEBUG("Parser: Error\r\n");
            m_parseError = true;
            break;
        }

        if (m_parseError)
        {
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

std::vector<Configuration> ConfigParser::scanConfigurations(std::string_view configString)
{
    // Reset parsed data
    m_currentNodeIdentifier = DeviceIdentifier{};
    m_configurations.clear();
    m_currentConfigId = 0;

    // initialize parser
    lwjson_stream_init(&m_parser, shallowTokenParser);

    // Open file

    // Parse file
    // iterate over all characters in the file
    lwjsonr_t parseRes;
    for (char c : configString)
    {
        parseRes = lwjson_stream_parse(&m_parser, c);
        if (parseRes == lwjsonSTREAMINPROG) {} 
        else if (parseRes == lwjsonSTREAMWAITFIRSTCHAR) {
            PARSE_DEBUG("Parser: Waiting first character\r\n");
        } 
        else if (parseRes == lwjsonSTREAMDONE) {
            PARSE_DEBUG("Parser: Done\r\n");
            break;
        } 
        else {
            PARSE_DEBUG("Parser: Error\r\n");
            m_parseError = true;
            break;
        }

        if (m_parseError)
        {
            break;
        }
    }

    if (m_parseError)
    {
        return std::vector<Configuration>{};
    }

    // Move nodes to vectors 
    return std::move(m_configurations);
}

void ConfigParser::shallowTokenParser(lwjson_stream_parser_t* jsp, lwjson_stream_type_t type)
{
    ConfigParser& parser = ConfigParser::getInstance();

    // Handle config id
    // config object start
    if (jsp->stack_pos == 2 && type == LWJSON_STREAM_TYPE_OBJECT)
    {
        PARSE_DEBUG("\r\nConfig[%d]: ", parser.m_currentConfigId);
        parser.m_configurations.push_back(Configuration{});
    }
    if (jsp->stack_pos == 1 && type == LWJSON_STREAM_TYPE_OBJECT_END)
    {
        // new config ended, increase config id
        parser.m_currentConfigId++;
    }

    // Get config name
    if (lwjson_stack_seq_3(jsp, 0, ARRAY, OBJECT, KEY)
         && type == LWJSON_STREAM_TYPE_STRING
         && jsp->stack_pos == 3)
    {
        parser.m_configurations.back().name = jsp->data.str.buff;
        PARSE_DEBUG("%s\r\n", jsp->data.str.buff);
    }

    // Handle ConfigSection
    if (lwjson_stack_seq_4(jsp, 0, ARRAY, OBJECT, KEY, ARRAY)
         && type == LWJSON_STREAM_TYPE_ARRAY
         && jsp->stack_pos == 4)
    {

        // set current node category
        if (strcmp(jsp->stack[2].meta.name, "deviceNodes") == 0)
        {
            parser.m_currentConfigSection = ConfigSection::DeviceNodes;// print section
            PARSE_DEBUG("\r\n Scan %s\r\n", jsp->stack[2].meta.name);
        }
        else if (strcmp(jsp->stack[2].meta.name, "algorithmicNodes") == 0)
        {
            parser.m_currentConfigSection = ConfigSection::AlgorithmicNodes;
        }
        else
        {
            parser.m_currentConfigSection = ConfigSection::None;
        }
    }

    // Only load device nodes
    if (parser.m_currentConfigSection != ConfigSection::DeviceNodes)
    {
        return;
    }

    // Handle NodeData
    // New node started
    else if (lwjson_stack_seq_5(jsp, 0, ARRAY, OBJECT, KEY, ARRAY, OBJECT)
         && type == LWJSON_STREAM_TYPE_OBJECT
         && jsp->stack_pos == 4)
    {
        // reset current node data
        parser.m_currentNodeIdentifier = DeviceIdentifier{};
        parser.m_configIdFound = false;
        parser.m_typeFound = false;
    }

    // Node field scalar string
    else if (lwjson_stack_seq_6(jsp, 0, ARRAY, OBJECT, KEY, ARRAY, OBJECT, KEY)
         && jsp->stack_pos == 6)
    {
        // type field
        if (type == LWJSON_STREAM_TYPE_STRING && strcmp(jsp->stack[5].meta.name, "type") == 0)
        {
            parser.m_typeFound = true;

            // check if type is valid
            if (strlen(jsp->data.str.buff) != 10)
            {
                parser.m_parseError = true;
                PARSE_DEBUG("Invalid type\r\n");
                return;
            }

            std::copy(jsp->data.str.buff, jsp->data.str.buff + 10, parser.m_currentNodeIdentifier.type.data());
            PARSE_DEBUG("  %s\r\n", jsp->data.str.buff);
        }

        // ID field
        else if (type == LWJSON_STREAM_TYPE_STRING)
        {
            if (strcmp(jsp->stack[5].meta.name, "ID") == 0)
            {
                parser.m_configIdFound = true;
                // check if ID is valid
                if (strlen(jsp->data.str.buff) != 10)
                {
                    parser.m_parseError = true;
                    PARSE_DEBUG("Invalid ID\r\n");
                    return;
                }
                
                // copy ID
                std::copy(jsp->data.str.buff, jsp->data.str.buff + 10, parser.m_currentNodeIdentifier.id.data());
                PARSE_DEBUG("   %s: \"%s\"\r\n", jsp->stack[5].meta.name, jsp->data.str.buff);
            }
        }
    }

    // Node is finished
    else if (lwjson_stack_seq_4(jsp, 0, ARRAY, OBJECT, KEY, ARRAY)
         && type == LWJSON_STREAM_TYPE_OBJECT_END
         && jsp->stack_pos == 4)
    {
        if (parser.m_currentConfigSection == ConfigSection::DeviceNodes)
        {
            if (parser.m_configurations.empty())
            {
                parser.m_parseError = true;
                PARSE_DEBUG("No configuration to add device to\r\n");
                return;
            }
            if (!parser.m_configIdFound)
            {
                parser.m_parseError = true;
                PARSE_DEBUG("No ID found for device\r\n");
                return;
            }
            if (!parser.m_typeFound)
            {
                parser.m_parseError = true;
                PARSE_DEBUG("No type found for device\r\n");
                return;
            }
            
            parser.m_configurations.back().deviceNodes.push_back(parser.m_currentNodeIdentifier);
        }
    }
}

void ConfigParser::deepTokenParser(lwjson_stream_parser_t* jsp, lwjson_stream_type_t type)
{
    ConfigParser& parser = ConfigParser::getInstance();

    // Handle config id
    // config object start
    if (jsp->stack_pos == 2 && type == LWJSON_STREAM_TYPE_OBJECT)
    {
        PARSE_DEBUG("Config[%d]:\r\n", parser.m_currentConfigId);
        if (parser.m_currentConfigId != parser.m_configId)
        {
            PARSE_DEBUG(" - ignore\r\n");
        }
    }
    if (jsp->stack_pos == 1 && type == LWJSON_STREAM_TYPE_OBJECT_END)
    {
        // new config ended, increase config id
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
        PARSE_DEBUG("\r\n Load %s\r\n", jsp->stack[2].meta.name);
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
        PARSE_DEBUG("\r\n Load Link\r\n");
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
            parser.m_currentNodeData.type = std::string{jsp->data.str.buff};
            PARSE_DEBUG( "  %s\r\n", parser.m_currentNodeData.type.c_str());
        }

        // scalar string field
        else if (type == LWJSON_STREAM_TYPE_STRING)
        {
            parser.m_currentNodeData.scalarParams[jsp->stack[5].meta.name] = jsp->data.str.buff;
            PARSE_DEBUG( "   %s: \"%s\"\r\n", jsp->stack[5].meta.name, jsp->data.str.buff);
        }

        // scalar number field
        else if (type == LWJSON_STREAM_TYPE_NUMBER)
        {
            parser.m_currentNodeData.scalarParams[jsp->stack[5].meta.name] = jsp->data.str.buff;
            PARSE_DEBUG( "   %s: %s\r\n", jsp->stack[5].meta.name, jsp->data.str.buff);
        }

        // scalar boolean field
        else if (type == LWJSON_STREAM_TYPE_TRUE || type == LWJSON_STREAM_TYPE_FALSE)
        {
            parser.m_currentNodeData.scalarParams[jsp->stack[5].meta.name] = type == LWJSON_STREAM_TYPE_TRUE?"1":"0";
            PARSE_DEBUG( "   %s: %s\r\n", jsp->stack[5].meta.name, type == LWJSON_STREAM_TYPE_TRUE?"true":"false");
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
            PARSE_DEBUG( "   %s: [", lastArrayName.c_str());
        }

        // array string field
        if (type == LWJSON_STREAM_TYPE_STRING)
        {
            parser.m_currentNodeData.arrayParams[lastArrayName].push_back(jsp->data.str.buff);
            PARSE_DEBUG( "\"%s\", ", jsp->data.str.buff);
        }

        // array number field
        else if (type == LWJSON_STREAM_TYPE_NUMBER)
        {
            parser.m_currentNodeData.arrayParams[lastArrayName].push_back(jsp->data.str.buff);
            PARSE_DEBUG( "%s, ", jsp->data.str.buff);
        }

        // array boolean field
        else if (type == LWJSON_STREAM_TYPE_TRUE || type == LWJSON_STREAM_TYPE_FALSE)
        {
            parser.m_currentNodeData.arrayParams[lastArrayName].push_back(type == LWJSON_STREAM_TYPE_TRUE?"1":"0");
            PARSE_DEBUG( "%s, ", jsp->data.str.buff);
        }
    }
    else if (lwjson_stack_seq_5(jsp, 0, ARRAY, OBJECT, KEY, ARRAY, OBJECT)
         && jsp->stack_pos == 5
         && type == LWJSON_STREAM_TYPE_ARRAY_END)
    {
        // close array
        PARSE_DEBUG( "]\r\n");
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
                PARSE_DEBUG( "Could not create device node\r\n");
            }
            break;
        case ConfigSection::AlgorithmicNodes:
            parser.m_algorithms.push_back(parser.createAlgorithmicNode(parser.m_currentNodeData));

            // check if creation was successful
            if (parser.m_algorithms.back() == nullptr)
            {
                parser.m_parseError = true;
                PARSE_DEBUG( "Could not create algorithmic node\r\n");
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
        PARSE_DEBUG( "  %s->%s", jsp->data.str.buff, jsp->stack[4].meta.name);
       
        PortDescriptor inputPort;
        PortDescriptor outputPort;
        if (createPortDescriptor(jsp->stack[4].meta.name, inputPort) && createPortDescriptor(jsp->data.str.buff, outputPort))
        {
            if (!parser.linkNodes(inputPort, outputPort))
            {
                parser.m_parseError = true;
                PARSE_DEBUG( " - error\r\n");
            }
            else
            {
                PARSE_DEBUG( " - linked\r\n");
            }
        }
    }
}

std::unique_ptr<DeviceNode> ConfigParser::createDeviceNode(const NodeData& nodeData)
{
    std::array<char, 10> ID;
    if (!getID(nodeData, ID))
    {
        return nullptr;
    }
    if (nodeData.type == "BarDisp7Ch")
    {
        return std::make_unique<BarDisplay>(ID);
    }
    /*else if (nodeData.type == "BltSink6Ch")
    {
        return std::make_unique<OutputNode>(ID);
    }
    else if (nodeData.type == "EmgSens6Ch")
    {
        return std::make_unique<FloatInputNode>(ID);
    }*/
    else
    {
        return nullptr;
    }
}

template <typename T>
std::unique_ptr<LinearFunctionNode<T>> createLinearFunctionNode(const NodeData& nodeData)
{
    T slope;
    T offset;
    if (!parseParameter("slope", nodeData, slope) || !parseParameter("offset", nodeData, offset))
    {
        return nullptr;
    }
    return std::make_unique<LinearFunctionNode<T>>(slope, offset);
}

std::unique_ptr<AlgorithmicNode> ConfigParser::createAlgorithmicNode(const NodeData& nodeData)
{
    if (nodeData.type == "TestAlgorithm")
    {
        return std::make_unique<TestAlgorithm>();
    }
    else if (nodeData.type == "LogNode")
    {
        std::string longId;
        std::string shortId;

        bool success = true;
        success &= nodeData.scalarParams.find("longId") != nodeData.scalarParams.end();
        success &= nodeData.scalarParams.find("shortId") != nodeData.scalarParams.end();
        if (!success)
        {
            return nullptr;
        }

        longId = nodeData.scalarParams.at("longId");
        shortId = nodeData.scalarParams.at("shortId");

        return std::make_unique<LogNode>(longId, shortId);
    }
    else if (nodeData.type == "LinearFuncNode")
    {
        // First get type
        std::string type;
        if (nodeData.scalarParams.find("dataType") == nodeData.scalarParams.end())
        {
            return nullptr;
        }
        type = nodeData.scalarParams.at("dataType");
        
        if (type == "float")
        {
            return createLinearFunctionNode<float>(nodeData);
        }
        else if (type == "int32")
        {
            return createLinearFunctionNode<int32_t>(nodeData);
        }
        else if (type == "uint32")
        {
            return createLinearFunctionNode<uint32_t>(nodeData);
        }
        else if (type == "int8")
        {
            return createLinearFunctionNode<int8_t>(nodeData);
        }
        else if (type == "uint8")
        {
            return createLinearFunctionNode<uint8_t>(nodeData);
        }
        else
        {
            return nullptr;
        }
    }
    /*else if (nodeData.type == "AdditionNode")
    {
        bool subtract;
        if (!parseParameter("subtract", nodeData, subtract))
        {
            return nullptr;
        }
        return std::make_unique<AdditonNode>(subtract);
    }
    else if (nodeData.type == "Float2IntNode")
    {
        return std::make_unique<CastNode<float, int>>();
    }
    else if (nodeData.type == "ArraySelectorNode")
    {
        uint32_t index;
        if (!parseParameter("index", nodeData, index))
        {
            return nullptr;
        }
        return std::make_unique<ArraySelectorNode>(index);
    }
    else if (nodeData.type == "AdvancedTestNode")
    {
        std::array<int, 3> numericalParameters;
        std::array<bool, 3> booleanParameters;
        if (!parseParameter("numericalParameters", nodeData, numericalParameters) 
            || !parseParameter("booleanParameters", nodeData, booleanParameters))
        {
            return nullptr;
        }
        return std::make_unique<AdvancedTestNode>(numericalParameters, booleanParameters);
    }*/
    else
    {
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

template <typename T>
static bool parseParameter(const std::string& parameterName, const NodeData& nodeData, T& parameter)
{
    if (nodeData.scalarParams.find(parameterName) == nodeData.scalarParams.end())
    {
        return false;
    }

    bool success = false;
    if (!std::numeric_limits<T>::is_integer)
    {
        // parse float parameter
        T value = stringToFloat(nodeData.scalarParams.at(parameterName), success);
        if (!success)
        {
            return false;
        }
        parameter = value;
        return true;
    }
    else{
        int64_t maxValue = std::numeric_limits<T>::max();
        int64_t minValue = std::numeric_limits<T>::min();
        if (std::is_signed<T>::value)
        {
            int32_t value = stringToInt(nodeData.scalarParams.at(parameterName), success);
            if (!success || value > maxValue || value < minValue)
            {
                return false;
            }
            parameter = value;
        }
        else
        {
            uint32_t value = stringToUint(nodeData.scalarParams.at(parameterName), success);
            if (!success || value > maxValue)
            {
                return false;
            }
            parameter = value;
        }
        return true;
    }
}

template <typename T, size_t N>
static bool parseParameter(const std::string& parameterName, const NodeData& nodeData, std::array<T, N>& parameter)
{
    if (nodeData.arrayParams.find(parameterName) == nodeData.arrayParams.end())
    {
        return false;
    }

    // check if array has the correct size
    auto& paramStringArray = nodeData.arrayParams.at(parameterName);
    if (paramStringArray.size() != N)
    {
        return false;
    }

    bool success = false;
    for (size_t i = 0; i < N; i++)
    {
        if (!std::numeric_limits<T>::is_integer)
        {
            // parse float parameter
            T value = stringToFloat(paramStringArray[i], success);
            if (!success)
            {
                return false;
            }
            parameter[i] = value;
        }
        else
        {
            // parse integer parameter
            uint32_t maxValue = std::numeric_limits<T>::max();
            int32_t minValue = std::numeric_limits<T>::min();
            if (std::is_signed<T>::value)
            {
                int32_t value = stringToInt(paramStringArray[i], success);
                if (!success || value > maxValue || value < minValue)
                {
                    return false;
                }
                parameter[i] = value;
            }
            else
            {
                uint32_t value = stringToUint(paramStringArray[i], success);
                if (!success || value > maxValue)
                {
                    return false;
                }
                parameter[i] = value;
            }
        }
    }


    return true;
}

static int32_t stringToInt(std::string_view str, bool& success)
{
    if (str.empty())
    {
        success = false;
        return 0;
    }

    // remove leading whitespace
    str.remove_prefix(std::min(str.find_first_not_of(" "), str.size()));

    bool negative = str[0] == '-';
    if (negative)
    {
        str.remove_prefix(1);
    }
    uint32_t value = stringToUint(str, success);
    if(value <= INT32_MAX)
    {
        return negative?-value:value;
    }
    else
    {
        success = false;
        return 0;
    }
}

/**
 * @brief Convert the given string to an unsigned integer.
 * 
 * @param str                           String to convert
 * @param success                       Success flag, set to false if conversion failed
 * @return uint32_t                     Converted value
 */
static uint32_t stringToUint(std::string_view str, bool& success)
{
    if (str.empty())
    {
        success = false;
        return 0;
    }

    // remove leading whitespace
    str.remove_prefix(std::min(str.find_first_not_of(" "), str.size()));

    // check if possible negative
    bool negative = str[0] == '-';
    if (negative)
    {
        success = false;
        return 0;
    }

    // Check if hexadecimal
    bool hex = str.size() > 2 && str[0] == '0' && (str[1] == 'x' || str[1] == 'X');
    if (hex)
    {
        str.remove_prefix(2);
    }

    uint32_t value = 0;
    uint32_t lastValue = 0;
    for (char c : str)
    {
        if (c >= '0' && c <= '9')
        {
            value = value * (hex?16:10) + c - '0';
        }
        else if (hex && c >= 'A' && c <= 'F')
        {
            value = value * 16 + c - 'A' + 10;
        }
        else if (hex && c >= 'a' && c <= 'f')
        {
            value = value * 16 + c - 'a' + 10;
        }
        else
        {
            // invalid character
            success = false;
            return 0;
        }
        if (value < lastValue)
        {
            // overflow
            success = false;
            return 0;
        }
        lastValue = value;
    }
    success = true;
    return value;
}

/**
 * @brief Convert the given string to a float.
 * 
 * @param str                           String to convert
 * @param success                       Success flag, set to false if conversion failed
 * @return float                        Converted value
 */
static float stringToFloat(std::string_view str, bool& success)
{
    if (str.empty())
    {
        success = false;
        return 0;
    }

    // remove leading whitespace
    str.remove_prefix(std::min(str.find_first_not_of(" "), str.size()));

    bool negative = str[0] == '-';
    if (negative)
    {
        str.remove_prefix(1);
    }

    float value = 0;
    float lastValue = 0;
    bool decimal = false;
    float decimalFactor = 1;
    for (char c : str)
    {
        if (c >= '0' && c <= '9')
        {
            if (decimal)
            {
                decimalFactor *= 0.1f;
                value += (c - '0') * decimalFactor;
            }
            else
            {
                value = value * 10 + c - '0';
            }
        }
        else if (c == '.')
        {
            if (decimal)
            {
                success = false;
                return 0;
            }
            decimal = true;
        }
        else
        {
            // invalid character
            success = false;
            return 0;
        }
        if (value < lastValue)
        {
            // overflow
            success = false;
            return 0;
        }
        lastValue = value;
    }
    success = true;
    return negative?-value:value;
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