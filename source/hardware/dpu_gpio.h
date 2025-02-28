/**
 * @file dpu_gpio.h
 * @author Leon Farchau (leon2225)
 * @brief Interface for the GPIOs of the DPU 
 * @version 0.1
 * @date 19.07.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once
#include "fsl_gpio.h"
#include "fsl_common.h"
#include "board.h"
#include "pin_mux.h"

#define DEBUG_GPIO DEBUG_DEBUG0_GPIO
#define DEBUG_PINS {DEBUG_DEBUG0_PIN, DEBUG_DEBUG1_PIN, DEBUG_DEBUG2_PIN, DEBUG_DEBUG3_PIN}

#define DEBUG_PIN_0(x) GPIO_PinWrite(DEBUG_DEBUG0_GPIO, DEBUG_DEBUG0_PIN, x) // Pin 29 EMC_31
#define DEBUG_PIN_1(x) GPIO_PinWrite(DEBUG_DEBUG1_GPIO, DEBUG_DEBUG1_PIN, x) // Pin 30 EMC_37
#define DEBUG_PIN_2(x) GPIO_PinWrite(DEBUG_DEBUG2_GPIO, DEBUG_DEBUG2_PIN, x) // Pin 31 EMC_36
#define DEBUG_PIN_3(x) GPIO_PinWrite(DEBUG_DEBUG3_GPIO, DEBUG_DEBUG3_PIN, x) // Pin 32 B0_12
//#define DEBUG_SET_PINS(x) DEBUG_PIN_0(x&1); DEBUG_PIN_1((x>>1) & 1); DEBUG_PIN_2((x>>2) & 1); DEBUG_PIN_3((x>>3) & 1)
#define DEBUG_TOGGLE(x) GPIO_PortToggle(DEBUG_GPIO, 1u << DEBUG_PINS[x])

void debugSetValue(uint32_t value);