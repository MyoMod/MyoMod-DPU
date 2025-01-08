/**
 * @file embeddedDevicesManager.cpp
 * @author Leon Farchau (leon2225)
 * @brief A manager that handles the embedded devices that are directly connected to the DPU 
 *         and is equivalent to the PeripheralHandler for the MyoMod interface.
 * @version 0.1
 * @date 08.01.2025
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/* -------------------------------- Includes -------------------------------- */
#include "embeddedDevicesManager.h"

/* ----------------------------- Implementation ----------------------------- */

/**
 * @brief Construct a new embedded Devices Manager::embedded Devices Manager object
 * 
 */
EmbeddedDevicesManager::EmbeddedDevicesManager() :
    m_connectedDevices{},
    m_installedDevices{},
    m_pingPongIndex{0}
{
}

/**
 * @brief Registers a device as availible on this module
 * 
 * @param device    The device to register
 */
void EmbeddedDevicesManager::registerDevice(DeviceIdentifier device)
{
    m_connectedDevices.push_back(device);
}

/**
 * @brief Lists all connected devices
 * 
 * @return std::vector<DeviceIdentifier>    The connected devices
 */
std::vector<DeviceIdentifier> EmbeddedDevicesManager::listConnectedDevices() const
{
    return m_connectedDevices;
}

/**
 * @brief Installs a device
 * 
 * @param device    The device to install
 * @return Status   Ok if the device was installed successfully
 *                  Error if the device could not be installed
 */
Status EmbeddedDevicesManager::installDevices(std::vector<std::unique_ptr<EmbeddedDeviceNode>> devices)
{
    for (auto& device : devices)
    {
        m_installedDevices.push_back(std::move(device));
    }
    return Status::Ok;
}

/**
 * @brief Uninstalls all devices
 * 
 */
void EmbeddedDevicesManager::uninstallAllDevices()
{
    m_installedDevices.clear();
}

/**
 * @brief Gets the current ping-pong index
 * 
 * @return uint32_t     The current ping-pong index
 */
uint32_t EmbeddedDevicesManager::getPingPongIndex() const
{
    return m_pingPongIndex;
}

/**
 * @brief Sends a sync signal
 * 
 * @return Status   Ok if the sync signal was sent successfully
 *                  Error if the sync signal could not be sent
 */
Status EmbeddedDevicesManager::sendSync()
{
    for (auto& device : m_installedDevices)
    {
        device->sync();
    }
    return Status::Ok;
}

/**
 * @brief Enters real-time mode
 * 
 * @return Status   Ok if real-time mode was entered successfully
 *                  Error if real-time mode could not be entered
 */
Status EmbeddedDevicesManager::enterRealTimeMode()
{
    for (auto& device : m_installedDevices)
    {
        device->enterRealTimeMode();
    }
    return Status::Ok;
}

/**
 * @brief Exits real-time mode
 * 
 * @return Status   Ok if real-time mode was exited successfully
 *                  Error if real-time mode could not be exited
 */
Status EmbeddedDevicesManager::exitRealTimeMode()
{
    for (auto& device : m_installedDevices)
    {
        device->exitRealTimeMode();
    }
    return Status::Ok;
}

/**
 * @brief Processes incoming data
 * 
 * @return Status   Ok if the data was processed successfully
 *                  Error if the data could not be processed
 */
Status EmbeddedDevicesManager::processInData()
{
    for (auto& device : m_installedDevices)
    {
        device->processInData();
    }
    return Status::Ok;
}

/**
 * @brief Processes outgoing data
 * 
 * @return Status   Ok if the data was processed successfully
 *                  Error if the data could not be processed
 */ 
Status EmbeddedDevicesManager::processOutData()
{
    for (auto& device : m_installedDevices)
    {
        device->processOutData();
    }
    return Status::Ok;
}

/**
 * @brief Checks if a device is connected
 * 
 * @param device    The device to check
 * @return true     The device is connected
 * @return false    The device is not connected
 */
bool EmbeddedDevicesManager::isConnected(DeviceIdentifier& device) const
{
    for (auto& connectedDevice : m_connectedDevices)
    {
        if (connectedDevice == device)
        {
            return true;
        }
    }
    return false;
}

// End of File