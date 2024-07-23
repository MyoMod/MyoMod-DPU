#pragma once
/*
 * comInterface.h
 *
 *  Created on: Jan 1, 2024
 *      Author: leon
 */




/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <stdio.h>
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CONTROL_BYTE_ADDR(x) ((x) << 2)
#define CONTROL_BYTE_HOSTIN 0b10
#define CONTROL_BYTE_HOSTOUT 0
#define CONTROL_BYTE_PDS 0b1
#define CONTROL_BYTE_REGISTER 0
/*******************************************************************************
 * Data Types
 * ****************************************************************************/
// Data types

/*** Fields common to all devices for the communication interface ***/

/**
 * @brief Control byte for the communication interface
 * 
 */
struct __attribute__((packed)) ControlByte_t
{    
    uint8_t ADDR : 6; // used only for register access, otherwise reserved
    uint8_t HostIn_nHostOut : 1;
    uint8_t PDS_nRegister : 1;
};


enum class DeviceRegisterType
{
	Status = 0,
	CommonDeviceStatus,
	CommonDeviceInformation,
	CommonDeviceConfiguration,
	DeviceSpecificStatus,
	DeviceSpecificInformation,
	DeviceSpecificConfiguration,
    n
};
#define NUM_REGISTERS ((int)DeviceRegisterType::n)

/**
 * @brief Status byte for the communication interface
 * 
 */
struct __attribute__((packed)) StatusByte_t
{
    bool reserved2          : 1;
    bool realtime_warning   : 1;
    bool specific_warning   : 1;
    bool common_warning     : 1;
    bool reserved1          : 1;
    bool alignment_error    : 1;
    bool specific_error     : 1;
    bool common_error       : 1;
};

/**
 * @brief Common Device Status
 * 
 */
struct __attribute__((packed)) CommonDeviceStatus_t
{
    bool reserved2              : 3;
    bool config_length_warning  : 1;
    bool reserved1              : 1;
    bool streamdirection_error  : 1;
    bool config_access_error    : 1;
    bool not_initialized_error  : 1;
};

/**
 * @brief Common Device Information
 * 
 */
struct __attribute__((packed)) CommonDeviceInformation_t
{
    uint8_t             device_version[2];
    uint16_t            hostIn_size;
    uint16_t            hostOut_size;
    std::array<char,10> identifier;
    std::array<char,10> device_type;
    uint8_t             protocol_version[2];
};


/**
 * @brief Common Device Configuration
 * 
 */
struct __attribute__((packed)) CommonDeviceConfiguration_t
{
    uint8_t reserved : 7;
    uint8_t initialized : 1;
};
