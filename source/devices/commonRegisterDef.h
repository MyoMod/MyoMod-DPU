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
    uint32_t ADDR : 6; // used only for register access, otherwise reserved
    uint32_t HostIn_nHostOut : 1;
    uint32_t PDS_nRegister : 1;
};


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
    uint8_t     device_version[2];
    uint16_t    hostIn_size;
    uint16_t    hostOut_size;
    char        identifier[10];
    char        device_type[10];
    uint8_t     protocol_version[2];
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
