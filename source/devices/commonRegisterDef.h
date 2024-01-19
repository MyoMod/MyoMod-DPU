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
 * @brief Error states for the onebyte status byte
 * 
 */
enum class CommErrorState_t
{
    Ok = 1,
    CommonError,
    DeviceSpecificError,
    AlignmentError,
    OverflowH_IN,
    UnderflowH_IN,
    OverflowH_OUT,
    UnderflowH_OUT
};

/**
 * @brief Warning states for the onebyte status byte
 * 
 */
enum class CommWarning_t
{
    Ok = 1,
    CommonWarning,
    DeviceSpecificWarning,
};

/**
 * @brief Status byte for the communication interface
 * 
 */
struct __attribute__((packed)) StatusByte_t
{
    CommErrorState_t errorState : 4;
    CommWarning_t warningState : 2;
    bool H_IN_FIFO_AVAIL : 1;
    bool H_OUT_FIFO_NFULL : 1;
};

/**
 * @brief Common Device Status
 * 
 */
struct __attribute__((packed)) CommonDeviceStatus_t
{
    uint8_t notInitialized : 1;
    uint8_t ill_HostInBurstSize : 1;
    uint8_t ill_HostOutBurstSize : 1;
    uint8_t ill_ConfigurationAccess : 1;
    uint8_t reserved : 4;
};

/**
 * @brief Direction of the stream supported by the device
 * 
 */
enum class StreamDir_t
{
    None = 0,
    HostIn = 1,
    HostOut = 2,
    HostInHostOut = 3
};

/**
 * @brief Common Device Information
 * 
 */
struct __attribute__((packed)) CommonDeviceInfo_t
{
    uint8_t H_In_PacketSize;
    uint8_t H_Out_PacketSize;
    char Identifier[10];
    uint8_t DeviceVersion[3];
    uint8_t ProtocolVersion[3];
    StreamDir_t SupportedStreamDirections;
};

/**
 * @brief Common Device Configuration
 * 
 */
struct __attribute__((packed)) CommonDeviceConfiguration_t
{
    uint8_t H_In_BurstSize;
    uint8_t H_Out_BurstSize;
    uint8_t DeviceIntialized : 1;
    uint8_t reserved : 7;
};
