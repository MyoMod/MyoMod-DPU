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
#include <limits>

#include "configParser.h"
#include "node.h"
// Data structures

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
    // config object start
    if (jsp->stack_pos == 2 && type == LWJSON_STREAM_TYPE_OBJECT)
    {
        std::cout << "Config[" << parser.m_currentConfigId << "]:" << std::endl;
        if (parser.m_currentConfigId != parser.m_configId)
        {
            std::cout << " - ignore" << std::endl;
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
        std::cout << std::endl << " Load " << jsp->stack[2].meta.name << std::endl;
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
        std::cout << std::endl << " Load Links" << std::endl;
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
            std::cout << "  " << parser.m_currentNodeData.type << std::endl;
        }

        // scalar string field
        else if (type == LWJSON_STREAM_TYPE_STRING)
        {
            parser.m_currentNodeData.scalarParams[jsp->stack[5].meta.name] = jsp->data.str.buff;
            std::cout << "   " << jsp->stack[5].meta.name << ": \"" << jsp->data.str.buff << "\"" << std::endl;
        }

        // scalar number field
        else if (type == LWJSON_STREAM_TYPE_NUMBER)
        {
            parser.m_currentNodeData.scalarParams[jsp->stack[5].meta.name] = jsp->data.str.buff;
            std::cout << "   " << jsp->stack[5].meta.name << ": " << jsp->data.str.buff << std::endl;
        }

        // scalar boolean field
        else if (type == LWJSON_STREAM_TYPE_TRUE || type == LWJSON_STREAM_TYPE_FALSE)
        {
            parser.m_currentNodeData.scalarParams[jsp->stack[5].meta.name] = type == LWJSON_STREAM_TYPE_TRUE?"1":"0";
            std::cout << "   " << jsp->stack[5].meta.name << ": " << (LWJSON_STREAM_TYPE_TRUE?"true":"false") << std::endl;
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
            std::cout << "   " << lastArrayName << ": [";
        }

        // array string field
        if (type == LWJSON_STREAM_TYPE_STRING)
        {
            parser.m_currentNodeData.arrayParams[lastArrayName].push_back(jsp->data.str.buff);
            std::cout << "\"" << jsp->data.str.buff << "\", ";
        }

        // array number field
        else if (type == LWJSON_STREAM_TYPE_NUMBER)
        {
            parser.m_currentNodeData.arrayParams[lastArrayName].push_back(jsp->data.str.buff);
            std::cout << jsp->data.str.buff << ", ";
        }

        // array boolean field
        else if (type == LWJSON_STREAM_TYPE_TRUE || type == LWJSON_STREAM_TYPE_FALSE)
        {
            parser.m_currentNodeData.arrayParams[lastArrayName].push_back(type == LWJSON_STREAM_TYPE_TRUE?"1":"0");
            std::cout << jsp->data.str.buff << ", ";
        }
    }
    else if (lwjson_stack_seq_5(jsp, 0, ARRAY, OBJECT, KEY, ARRAY, OBJECT)
         && jsp->stack_pos == 5
         && type == LWJSON_STREAM_TYPE_ARRAY_END)
    {
        // close array
        std::cout << "]" << std::endl;
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
        std::cout << "  " << jsp->data.str.buff << "->" << jsp->stack[4].meta.name;
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
    bool parseSuccess = false;
    std::array<char, 10> ID;
    if (!getID(nodeData, ID))
    {
        return nullptr;
    }
    if (nodeData.type == "InputNode")
    {
        return std::make_unique<InputNode>(ID);
    }
    else if (nodeData.type == "OutputNode")
    {
        return std::make_unique<OutputNode>(ID);
    }
    else if (nodeData.type == "FloatInputNode")
    {
        return std::make_unique<FloatInputNode>(ID);
    }
    else if (nodeData.type == "ArrayInputNode")
    {
        return std::make_unique<ArrayInputNode>(ID);
    }
    else
    {
        return nullptr;
    }
}

std::unique_ptr<AlgorithmicNode> ConfigParser::createAlgorithmicNode(const NodeData& nodeData)
{
    bool parseSuccess = false;
    if (nodeData.type == "AdditonNode")
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
    }
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
    uint32_t maxValue = std::numeric_limits<T>::max();
    uint32_t minValue = std::numeric_limits<T>::min();
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

    // Check if hexadeciaml
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
                decimalFactor *= 0.1;
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