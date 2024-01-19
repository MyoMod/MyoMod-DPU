#pragma once
/*
 * PDSChannel.h
 *
 *  Created on: Jan 9, 2024
 *      Author: leon
 */

#include "stdint.h"
#include <string>
namespace freesthetics {

enum class PDSChannelTypes {
	EMG_32Bit,
	EMG_24Bit,
	EMG_16Bit,
	EMG_8Bit,
	GENERIC_32Bit,
	GENERIC_24Bit,
	GENERIC_16Bit,
	GENERIC_8Bit,
	BUTTON_8Bit,
};

/*
 *
 */
struct PDSChannel {
public:
	std::string name;
	PDSChannelTypes type;
	std::string unit;
	uint32_t length;
	uint32_t sampleSize;
	bool isInput;
};

} /* namespace freesthetics */
