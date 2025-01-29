#pragma once
#include <stdint.h>

struct MemoryRegion
{
	void* address;
	uint32_t size;
};