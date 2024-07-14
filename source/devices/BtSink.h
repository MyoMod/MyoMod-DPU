#pragma once

#include "Device.h"



class BtSink : public Device {
public:
    // Constructor
    BtSink(std::string_view name);

    // Destructor
    ~BtSink();

    std::array<char, 10> getDeviceType() override { return std::array<char, 10> { "BtSink6Ch" }; }
    // Other member functions and variables
};


