/**
 * @file embeddedDevicesManager.h
 * @author Leon Farchau (leon2225)
 * @brief A manager that handles the embedded devices that are directly connected to the DPU 
 *         and is equivalent to the PeripheralHandler for the MyoMod interface.
 * @version 0.1
 * @date 08.01.2025
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

/* -------------------------------- includes -------------------------------- */
// System includes
#include <vector>
#include <memory>

// Includes from project
#include "embeddedDeviceNode.h"
#include "Status.h"

/* ------------------------------- Constants ------------------------------- */

/* ---------------------------- Type Definitions ---------------------------- */

class EmbeddedDevicesManager
{
public:
    EmbeddedDevicesManager();
    ~EmbeddedDevicesManager() = default;

    void registerDevice(DeviceIdentifier device);
    std::vector<DeviceIdentifier> listConnectedDevices() const;

    Status installDevices(std::vector<std::unique_ptr<EmbeddedDeviceNode>> devices);
	void uninstallAllDevices();

	uint32_t getPingPongIndex() const;

	Status sendSync();

	Status enterRealTimeMode();
	Status exitRealTimeMode();

    Status processInData();
    Status processOutData();

    bool hasInstalledDevices() const { return !m_installedDevices.empty(); }
	bool isConnected(DeviceIdentifier& device) const;
private:
    std::vector<DeviceIdentifier> m_connectedDevices;
    std::vector<std::unique_ptr<EmbeddedDeviceNode>> m_installedDevices;

    uint32_t m_pingPongIndex;
};
