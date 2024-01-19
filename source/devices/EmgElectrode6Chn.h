#ifndef EMG_ELECTRODE_6_CHN_H
#define EMG_ELECTRODE_6_CHN_H

#include "Device.h"

namespace freesthetics {

class EmgElectrode6Chn : public Device {
public:
    // Constructor
    EmgElectrode6Chn(std::string_view name);

    // Destructor
    ~EmgElectrode6Chn();

    std::array<char, 10> getDeviceType() override { return std::array<char, 10> { "Elctr6Ch" }; }
    // Other member functions and variables
};

} // namespace freesthetics
#endif // EMG_ELECTRODE_6_CHN_H
