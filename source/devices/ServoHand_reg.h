#pragma once
#include <stdint.h>

/*** Device Specific fields for the communication interface ***/

/**
 * @brief Device specific status
 * 
 */
struct __attribute__((packed)) DeviceSpecificStatus_t
{
    uint8_t displayDetected : 8;
};

/**
 * @brief Device specific information
 * 
 */
struct __attribute__((packed)) DeviceSpecificInfo_t
{
    uint8_t deviceSpecificInfo[10];
};

/**
 * @brief Device specific configuration
 * 
 */
struct DeviceSpecificConfiguration_t
{
    uint32_t barColors[7] =
    {
        0x00ff00,
        0xffff00,
        0xff0000,
        0xff00ff,
        0x0000ff,
        0x00ffff,
        0x000000
    };
};

/**
 * @brief Register Names
 * 
 */
enum RegisterName_t
{
    REG_StatusByte,
    REG_CommonDeviceStatus,
    REG_CommonDeviceInfo,
    REG_CommonDeviceConfiguration,
    REG_DeviceSpecificStatus,
    REG_DeviceSpecificInfo,
    REG_DeviceSpecificConfiguration,
    NUM_REGISTERS,
};

/**
 * @brief Accessrights for the registers
 * 
 */
enum class AccessRights_t
{
    NoAccess = 0,
    Read = 1,
    Write = 2,
    ReadWrite = 3
};