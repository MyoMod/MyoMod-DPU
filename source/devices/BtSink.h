#pragma once

#include "Device.h"

namespace freesthetics {

class BtSink : public Device {
public:
    // Constructor
    BtSink(std::string_view name);

    // Destructor
    ~BtSink();

    std::array<char, 10> getDeviceType() override { return std::array<char, 10> { "BtSink6Ch" }; }
    // Other member functions and variables
};

} // namespace freesthetics
