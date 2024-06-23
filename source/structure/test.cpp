#include <iostream>
#include "node.h"
#include "nodeFactory.h"
#include "configParser.h"
#include <filesystem>


int main()
{
    ConfigParser& parser = ConfigParser::getInstance();
    std::cout << std::filesystem::current_path() << std::endl;
    auto [devices, algorithms] = parser.parseConfig("../../../configData/config.json", 0);

    for (int i = 0; i < 10; i++)
    {
        // Handle input nodes
        for (auto& inputDevice : devices)
        {
            inputDevice->processInData();
        }

        // Handle algorithmic nodes
        for (auto& algorithm : algorithms)
        {
            algorithm->process();
        }

        // Handle output node
        for (auto& outputDevice : devices)
        {
            outputDevice->processOutData();
        }
    }
#if 0
    for (int j = 0; j < 10; j++)
    {
        std::cout << "Iteration: " << j << std::endl;
        // Create nodes
        auto inputNodeA = NodeFactory::createDeviceNode(NodeType::InputNode);
        auto inputNodeB = NodeFactory::createDeviceNode(NodeType::InputNode);
        auto inputNodeC = NodeFactory::createDeviceNode(NodeType::FloatInputNode);
        auto inputNodeD = NodeFactory::createDeviceNode(NodeType::ArrayInputNode);
        auto float2IntNode = NodeFactory::createAlgorithmicNode(NodeType::Float2IntNode);
        auto addNode1 = NodeFactory::createAlgorithmicNode(NodeType::AdditonNode);
        auto addNode2 = NodeFactory::createAlgorithmicNode(NodeType::AdditonNode);
        auto arraySelectorNode = NodeFactory::createAlgorithmicNode(NodeType::ArraySelectorNode, 5);
        auto outputNode = NodeFactory::createDeviceNode(NodeType::OutputNode);


        // Link nodes
        arraySelectorNode->linkInputPort(0, inputNodeD.get(), 0);
        addNode1->linkInputPort(0, inputNodeA.get(), 0);
        addNode1->linkInputPort(1, arraySelectorNode.get(), 0);
        float2IntNode->linkInputPort(0, inputNodeC.get(), 0);
        addNode2->linkInputPort(0, addNode1.get(), 0);
        addNode2->linkInputPort(1, float2IntNode.get(), 0);
        outputNode->linkInputPort(0, addNode2.get(), 0);


        // Move nodes to vectors
        std::vector<std::unique_ptr<DeviceNode>> inputDevices;
        std::vector<std::unique_ptr<DeviceNode>> outputDevices;
        std::vector<std::unique_ptr<AlgorithmicNode>> algorithms;

        inputDevices.push_back(std::move(inputNodeA));
        inputDevices.push_back(std::move(inputNodeB));
        inputDevices.push_back(std::move(inputNodeC));
        inputDevices.push_back(std::move(inputNodeD));

        algorithms.push_back(std::move(float2IntNode));
        algorithms.push_back(std::move(addNode1));
        algorithms.push_back(std::move(addNode2));
        algorithms.push_back(std::move(arraySelectorNode));

        outputDevices.push_back(std::move(outputNode));

        // Process nodes
        for (int i = 0; i < 10; i++)
        {
            // Handle input nodes
            for (auto& inputDevice : inputDevices)
            {
                inputDevice->processInData();
            }
            

            // Handle algorithmic nodes
            for (auto& algorithm : algorithms)
            {
                algorithm->process();
            }

            // Handle output node
            for (auto& outputDevice : outputDevices)
            {
                outputDevice->processOutData();
            }
        }
    }
#endif
}