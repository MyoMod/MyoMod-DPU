/*
 * Configuration.h
 *
 *  Created on: Jan 9, 2024
 *      Author: leon
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_
#include "stdint.h"
#include <vector>
#include <string>
#include <array>
#include <set>

#include "Status.h"
#include "node.h"


struct PortDescriptor
{
    NodeCategory category;
    uint32_t nodeIndex;
    uint32_t nodePort;
};

struct Configuration
{
    std::string name;
    std::vector<DeviceIdentifier> deviceNodes;

	bool isCompatibleWith (const std::vector<DeviceIdentifier>& devices) const;
};

#endif /* CONFIGURATION_H_ */
