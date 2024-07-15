#pragma once
#include <stdint.h>

/*** Device Specific fields for the communication interface ***/

/**
 * @brief Device specific status
 * 
 */
struct __attribute__((packed)) DeviceSpecificStatus_t
{
    uint8_t Synchronized : 1;
    uint8_t Clipped : 6;
    uint8_t RangeExceeded : 6;
    uint8_t ADCError : 1;
    uint8_t NoADC : 1;
    uint8_t Simulated : 1;
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
    uint8_t SamplesPerCycle;
    uint8_t Gain;
    uint8_t UsePGA : 1;
    uint8_t UseFaultControl : 1;
    uint8_t reserved : 6;
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