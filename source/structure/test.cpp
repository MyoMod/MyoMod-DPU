#include <iostream>
#include "node.h"
#include "configParser.h"
#include <filesystem>


int main()
{
    ConfigParser& parser = ConfigParser::getInstance();
    std::cout << std::filesystem::current_path() << std::endl;
    auto [devices, algorithms] = parser.parseConfig("../../../configData/config.json", 1);

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