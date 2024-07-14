#include <iostream>
#include "node.h"
#include "configParser.h"
#include <filesystem>


int main()
{
    ConfigParser& parser = ConfigParser::getInstance();
    std::cout << std::filesystem::current_path() << std::endl;

    std::cout << "Scan configurations..." << std::endl;
    auto configurations = parser.scanConfigurations("../../../configData/config.json");
    std::cout << "Configurations: " << configurations.size() << std::endl;

    uint32_t index = 0;
    std::cout << "Load configuration " << index << std::endl;
    auto [devices, algorithms] = parser.loadConfig("../../../configData/config.json", index);
    if (devices.empty() || algorithms.empty())
    {
        std::cout << "Failed to load configuration" << std::endl;
        return 1;
    }
    std::cout << "\nConfiguration loaded" << std::endl;


    std::cout << "\nProcess data..." << std::endl;    
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
}