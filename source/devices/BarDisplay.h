#pragma once

#include "Device.h"

namespace freesthetics {

class BarDisplay : public Device {
public:
    // Constructor
    BarDisplay(std::string_view name);

    // Destructor
    ~BarDisplay();

    std::array<char, 10> getDeviceType() override { return std::array<char, 10> { "BarDis7Ch" }; }
    // Other member functions and variables
};

} // namespace freesthetics
