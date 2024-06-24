/**
 * @file configParser.h
 * @author Leon Farchau (leon2225)
 * @brief   The ConfigParser is used to parse a configuration file and create the corresponding nodes as well as link them together.
 *        The configuration file is expected to be in JSON format and contain a list of nodes with their type and parameters.
 * @version 0.1
 * @date 20.06.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <map>
#include <memory>
#include <string_view>
#include <array>
#include <vector>
#include <tuple>
#include <string>

#include "lwjson/lwjson.h"

#include "node.h"

// Tuple that consists of input devices, algorithms and output devices
using NodesTuple = std::tuple<
        std::vector<std::unique_ptr<DeviceNode>>,       // devices
        std::vector<std::unique_ptr<AlgorithmicNode>>  // algorithms
    >;

struct NodeData
{
    std::string type;
    std::map<std::string, std::string> scalarParams;
    std::map<std::string, std::vector<std::string>> arrayParams;
};

enum class NodeCategory
{
    None,
    DeviceNodes,
    AlgorithmNodes
};

enum class ConfigSection
{
    None,
    DeviceNodes,
    AlgorithmicNodes,
    Links
};


struct PortDescriptor
{
    NodeCategory category;
    uint32_t nodeIndex;
    uint32_t nodePort;
};

struct NodeIdentifier
{
    std::array<char, 10> type;
    std::array<char, 10> id;
};

struct Configuration
{
    std::string name;
    std::vector<NodeIdentifier> deviceNodes;
};

class ConfigParser
{
public:
    static ConfigParser& getInstance()
    {
        static ConfigParser instance;
        return instance;
    }

    NodesTuple loadConfig(std::string_view configPath, uint32_t configId);
    std::vector<Configuration> scanConfigurations(std::string_view configPath);

    ConfigParser(const ConfigParser&) = delete;
    ConfigParser& operator=(const ConfigParser&) = delete;

private:
    static void deepTokenParser(lwjson_stream_parser_t* jsp, lwjson_stream_type_t type);
    static void shallowTokenParser(lwjson_stream_parser_t* jsp, lwjson_stream_type_t type);
    std::unique_ptr<DeviceNode> createDeviceNode(const NodeData& nodeData);
    std::unique_ptr<AlgorithmicNode> createAlgorithmicNode(const NodeData& nodeData);
    bool linkNodes(const PortDescriptor& inputPort, const PortDescriptor& outputPort);


    lwjson_stream_parser_t m_parser;
    NodeData m_currentNodeData;
    NodeIdentifier m_currentNodeIdentifier;
    uint32_t m_configId;
    uint32_t m_currentConfigId;
    ConfigSection m_currentConfigSection;
    bool m_parseError;
    bool m_configIdFound;
    bool m_typeFound;

    std::vector<std::unique_ptr<DeviceNode>> m_devices;
    std::vector<std::unique_ptr<AlgorithmicNode>> m_algorithms;

    std::vector<Configuration> m_configurations;

    ConfigParser();
};

        


        
