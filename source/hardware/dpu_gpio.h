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

#define DEBUG_PIN_1(x) GPIO_PinWrite(BOARD_INITPINS_DEBUG1_GPIO, BOARD_INITPINS_DEBUG1_PIN, x) // Pin 29 EMC_31
#define DEBUG_PIN_2(x) GPIO_PinWrite(BOARD_INITPINS_DEBUG2_GPIO, BOARD_INITPINS_DEBUG2_PIN, x) // Pin 30 EMC_37
#define DEBUG_PIN_3(x) GPIO_PinWrite(BOARD_INITPINS_DEBUG3_GPIO, BOARD_INITPINS_DEBUG3_PIN, x) // Pin 31 EMC_36
#define DEBUG_PIN_4(x) GPIO_PinWrite(BOARD_INITPINS_DEBUG4_GPIO, BOARD_INITPINS_DEBUG4_PIN, x) // Pin 32 B0_12
#define DEBUG_PINS(x) DEBUG_PIN_1(x&1); DEBUG_PIN_2((x>>1) & 1); DEBUG_PIN_3((x>>2) & 1); DEBUG_PIN_4((x>>3) & 1)

#define RGB_R(x) GPIO_PinWrite(BOARD_INITPINS_RGB_R_GPIO, BOARD_INITPINS_RGB_R_PIN, x) // Pin 6 B0_10
#define RGB_G(x) GPIO_PinWrite(BOARD_INITPINS_RGB_G_GPIO, BOARD_INITPINS_RGB_G_PIN, x) // Pin 7 B1_01
#define RGB_B(x) GPIO_PinWrite(BOARD_INITPINS_RGB_B_GPIO, BOARD_INITPINS_RGB_B_PIN, x) // Pin 9 B0_11
#define RGB(x) RGB_R(x&1); RGB_G((x>>1) & 1); RGB_B((x>>2) & 1)
#define RGB_GND(x) GPIO_PinWrite(BOARD_INITPINS_RGB_GND_GPIO, BOARD_INITPINS_RGB_GND_PIN, x) // Pin 8 B1_00