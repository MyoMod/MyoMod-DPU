/*
 * PeripheralHandler.cpp
 *
 *  Created on: Jan 9, 2024
 *      Author: leon
 */

#include "PeripheralHandler.h"
#include "cmsis_gcc.h"

#define LPIIC_INTERRUPTS (kLPI2C_MasterFifoErrFlag | kLPI2C_MasterNackDetectFlag | kLPI2C_MasterStopDetectFlag \
							| kLPI2C_MasterArbitrationLostFlag)

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_DIVIDER (1U)
/* Get frequency of lpi2c clock */
#define LPI2C_CLOCK_FREQUENCY ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

#define I2C_FIFO_DEPTH 4
#define DMA_LOWEST_CHANNEL 4

std::array<PeripheralHandler*, I2C_UNITS> PeripheralHandler::handlers;

/**
 * @brief Constructa a new PeripheralHandler object and initializes the hardware.
 * 			It also registers the interrupt handlers for the dma and i2c hardware.
 * 
 * @param dma 						Pointer to the dma hardware
 * @param i2cIndex 					Index of the i2c hardware (1-4)
 * @param processDataCallback 		Callback function that is called when new input data is available
 * @param highSpeed 				True if the i2c should be configured for high speed mode (1MBit/s), false for normal speed (400kBit/s)
 */
PeripheralHandler::PeripheralHandler(DMA_Type* dma, uint32_t i2cIndex, void (*processDataCallback)(void), bool highSpeed):
	m_dmaBase{ dma }, 
	m_commState{CommState:: Stopped }, 
	m_dmaChannel{ DMA_LOWEST_CHANNEL + (i2cIndex - 1) }, // i2cIndex is 1 based, but the array is 0 based
	m_tcdIndex{ 0 },
	m_pingPongIndex{ 0 },
	m_connectedDevicesChanged{ true }, // Set to true to trigger the first listNewDevices call
	m_deviceExpected{ false },
	m_processDataCallback{ processDataCallback },
	m_stalled{ false }
{
	i2cIndex--; // i2cIndex is 1 based, but the array is 0 based
	/* Init dma hardware */

	// Enable DMA clock 
    CLOCK_EnableClock(kCLOCK_Dma);

    DMAMUX_Init(DMAMUX_BASE);
    // Procedure from manual 5.5.1
    // 2. Clear the CHCFG[ENBL] and CHCFG[TRIG] fields of the DMA channel.
    DMAMUX_DisableChannel(DMAMUX_BASE, m_dmaChannel);

    // 3. Ensure that the DMA channel is properly configured in the DMA. The DMA channel
    //  may be enabled at this point.

    // init edma module
    edma_config_t dmaConfig = {0};
    /*
     *   config.enableContinuousLinkMode = false;
     *   config.enableHaltOnError = true;
     *   config.enableRoundRobinArbitration = false;
     *   config.enableDebugMode = false;
     * */
    EDMA_GetDefaultConfig(&dmaConfig);
    dmaConfig.enableRoundRobinArbitration = true;

    EDMA_Init(m_dmaBase, &dmaConfig);

	// Enable interrupts
    // individual interrupts are enabled in the corresponding tcd
	IRQn interrupt{ static_cast<IRQn>((static_cast<uint32_t>(DMA0_DMA16_IRQn) + (m_dmaChannel%16)))};
    EnableIRQ(interrupt);

    // 4. Configure the corresponding timer.
    // 5. Select the source to be routed to the DMA channel. Write to the corresponding
    //      CHCFG register, ensuring that the CHCFG[ENBL] and CHCFG[TRIG] fields are
    //      set
	std::array<_dma_request_source,4> dmaSources{ 
		kDmaRequestMuxLPI2C1, kDmaRequestMuxLPI2C2, 
		kDmaRequestMuxLPI2C3, kDmaRequestMuxLPI2C4 };
	assert(i2cIndex < 4);
    DMAMUX_SetSource(DMAMUX_BASE, m_dmaChannel, dmaSources[i2cIndex]);
    DMAMUX_EnableChannel(DMAMUX_BASE, m_dmaChannel);

	/* Init LPI2C Hardware */
	std::array<LPI2C_Type*,4> i2cBases{ LPI2C1, LPI2C2, LPI2C3, LPI2C4 };
	m_i2cBase = i2cBases[i2cIndex];

	lpi2c_master_config_t i2cConfig = {0};

	/*Clock setting for LPI2C*/
	CLOCK_SetMux(kCLOCK_Lpi2cMux, LPI2C_CLOCK_SOURCE_SELECT);
    CLOCK_SetDiv(kCLOCK_Lpi2cDiv, LPI2C_CLOCK_SOURCE_DIVIDER);


    /*
     * masterConfig.debugEnable = false;
     * masterConfig.ignoreAck = false;
     * masterConfig.pinConfig = kLPI2C_2PinOpenDrain;
     * masterConfig.baudRate_Hz = 100000U;
     * masterConfig.busIdleTimeout_ns = 0;
     * masterConfig.pinLowTimeout_ns = 0;
     * masterConfig.sdaGlitchFilterWidth_ns = 0;
     * masterConfig.sclGlitchFilterWidth_ns = 0;
     */
    LPI2C_MasterGetDefaultConfig(&i2cConfig);
	i2cConfig.busIdleTimeout_ns = 10'000U; // 10us

	// Change baudrate
	i2cConfig.baudRate_Hz = highSpeed ? 1'000'000U : 400'000U;

	// Init i2c hardware
	LPI2C_MasterInit(m_i2cBase, &i2cConfig, LPI2C_CLOCK_FREQUENCY);

	// Activate IRQ
	interrupt = static_cast<IRQn>((static_cast<uint32_t>(LPI2C1_IRQn) + i2cIndex));
    EnableIRQ(interrupt);

	// Set tx and rx fifo watermark
	LPI2C_MasterSetWatermarks(m_i2cBase, 0, 0);

	// register handler
	PeripheralHandler::handlers[i2cIndex] = this;

	m_i2cIndex = i2cIndex;

	// Scan for devices
	m_connectedDevicesChanged = true;

}

/**
 * @brief Destroy the PeripheralHandler object, deinitializes the hardware and
 * 			 removes the interrupt handlers
 * 
 */
PeripheralHandler::~PeripheralHandler() {
	LPI2C_MasterDeinit(m_i2cBase);
	IRQn interrupt{ static_cast<IRQn>((static_cast<uint32_t>(LPI2C1_IRQn) + (m_i2cBase - LPI2C1)))};
	DisableIRQ(interrupt);

	interrupt = static_cast<IRQn>((static_cast<uint32_t>(DMA0_DMA16_IRQn) + (m_dmaChannel%16)));
    DisableIRQ(interrupt);

	DMAMUX_DisableChannel(DMAMUX_BASE, m_dmaChannel);
	EDMA_ResetChannel(m_dmaBase, m_dmaChannel);

	uint32_t i2cIndex = m_dmaChannel - DMA_LOWEST_CHANNEL;

	handlers[i2cIndex] = nullptr;
}

/**
 * @brief Returns a list of DeviceDescriptors for all connected devices
 * 
 * @param status 	Status::Ok if the function was successful
 * 		   			Status::Error if the function was called while the handler was not in stopped state
 * 		   			Status::Warning if the function was called, but there were no new devices
 * @return 			List of DeviceDescriptors for all connected devices
 */
std::vector<DeviceIdentifier> PeripheralHandler::listConnectedDevices(Status& status) {
	if(m_commState != CommState::Stopped)
	{
		// It is not allowed to list devices while in realTime mode
		status = Status::Error;
		return std::vector<DeviceIdentifier>();
	}

	if(m_stalled)
	{
		// we are stalled, no normal operation possible
		// just return an empty list, as we can't communicate with the devices
		status = Status::busStalled;
		m_connectedDevices.clear();
		m_connectedDevicesChanged = false;
		return std::vector<DeviceIdentifier>();
	}

	if(m_connectedDevicesChanged )
	{
		// TODO: scan for new devices and get their identifiers
		m_connectedDevices.clear();

#if SIMULATE_DEVICE_SCAN == 1
		switch (m_i2cIndex)
		{
		case 0:
			// Add devices that are connected to the Port 1
			m_connectedDevices[0x08] = DeviceIdentifier{ "Elctr6Ch", "Elctrode1" };
			//m_connectedDevices[0x18] = DeviceIdentifier{ "BarDis7Ch", "BDisplay1" };
			m_connectedDevices[0x28] = DeviceIdentifier{ "Elctr6Ch", "SDSource1" };
			m_connectedDevices[0x48] = DeviceIdentifier{ {'S','e','r','v','o',' ','H','a','n','d'}, {'S','e','r','v','o','H','a','n','d','1'} };
			//m_connectedDevices[0x38] = DeviceIdentifier{ "BtSink6Ch", "Blt_Sink1" };
			break;
		case 2:
			// Add devices that are connected to the Port 2
			//m_connectedDevices[0x08] = DeviceIdentifier{ "Elctr6Ch", "Elctrode2" };
			m_connectedDevices[0x18] = DeviceIdentifier{ {'B','a','r','D','i','s','p','7','C','h'}, {'B','a','r','D','i','s','p','l','a','y'} };
			//m_connectedDevices[0x28] = DeviceIdentifier{ "Elctr6Ch", "SDSource2" };
			m_connectedDevices[0x38] = DeviceIdentifier{ "BtSink6Ch", "Blt_Sink2" };
			break;
		case 3:
			// Add devices that are connected to the LowSpeed Port 
			//m_connectedDevices[0x08] = DeviceIdentifier{ "Elctr6Ch", "Elctrode3" };
			//m_connectedDevices[0x18] = DeviceIdentifier{ "BarDis7Ch", "BDisplay3" };
			//m_connectedDevices[0x28] = DeviceIdentifier{ "Elctr6Ch", "SDSource3" };
			//m_connectedDevices[0x38] = DeviceIdentifier{ "BtSink6Ch", "Blt_Sink3" };
			break;
		
		default:
			break;
		}
#else

		// Scan for addresses
		std::vector<uint8_t> connectedAddresses;

		for (uint8_t i = 1; i < 128; i++)
		{
			// Check if device is connected
			if(probeAddress(i) == Status::Ok)
			{
				connectedAddresses.push_back(i);
			}
		}

		// Get device identifiers
		for(auto address : connectedAddresses)
		{
			CommonDeviceInformation_t devInfo;
			std::span<std::byte> data{reinterpret_cast<std::byte*>(&devInfo), sizeof(devInfo)};
			readRegister(address, DeviceRegisterType::CommonDeviceInformation, data);

			DeviceIdentifier identifier = { devInfo.device_type, devInfo.identifier };
			m_connectedDevices[address] = identifier;
		}

#endif
		m_connectedDevicesChanged = false;
		status = Status::Ok;
	}
	else
	{
		// Function was called, but there were no new devices return a warning
		status = Status::Warning;
	}
	
	std::vector<DeviceIdentifier> devices(m_connectedDevices.size());
	int i = 0;
	for(auto& device : m_connectedDevices)
	{
		devices[i++] = device.second;
	}
	return devices;
}

/**
 * @brief Returns true if new devices were detected since the last call to listConnectedDevices
 * 
 * @return true if new devices were detected
 * @return false if no new devices were detected
 */
bool PeripheralHandler::connectedDevicesChanged() {
	return m_connectedDevicesChanged;
}

/**
 * @brief Installs a device. This means that the handler will setup the 
 * 			dma to communicate with the device 
 * 
 * @param device 	Pointer to the device that should be added
 * @return Status::Ok if the function was successful
 * 		   Status::Error if the function was called, but the device was already added
 */
Status PeripheralHandler::installDevice(DeviceNode* device) {
	
	assert(m_commState == CommState::Stopped);

	// Get address
	DeviceIdentifier identifier = device->getDeviceIdentifier();
	int32_t address = getDeviceAdress(identifier);
	assert(address != -1);

	// Get pointer to storage in deviceNode
	auto nodeStoarge = device->getNodeStorage();

	// Install tcds+commands for data in and out
	if(nodeStoarge.inSize > 0)
	{
		// Check if address is already in use
		if(m_installedHostInStorages.find(address) != m_installedHostInStorages.end())
		{
			return Status::Error;
		}
		m_installedHostInStorages[address] = nodeStoarge.inStorage;

		// Create memory region for dataIn
		MemoryRegion dataInStorage{ nodeStoarge.inStorage.get(), nodeStoarge.inSize };

		//Install 
		appendTcdIn(dataInStorage, address);
	}
	if(nodeStoarge.outSize > 0)
	{
		// Check if address is already in use
		if(m_installedHostOutStorages.find(address) != m_installedHostOutStorages.end())
		{
			return Status::Error;
		}
		m_installedHostOutStorages[address] = nodeStoarge.outStorage;

		// Create memory region for dataOut
		std::array<MemoryRegion,2> dataOutBuffer{ 
			MemoryRegion{ nodeStoarge.outStorage[0].get(), nodeStoarge.outSize },
			MemoryRegion{ nodeStoarge.outStorage[1].get(), nodeStoarge.outSize }
		};

		//Install tcds for data out
		appendTcdOut(dataOutBuffer, address);
	}

	// Update all links
	linkTcds();

	// Configure the device
	Status status;
	auto specificConf = device->getRegisterRawData(DeviceRegisterType::DeviceSpecificConfiguration, status);
	if (status == Status::Ok)
	{
		writeRegister(address, DeviceRegisterType::DeviceSpecificConfiguration, specificConf);
	}

	// there is at least one installed device
	// -> we are not in the state of having no installed devices
	m_deviceExpected = true;

	return Status::Ok;
}

/**
 * @brief Return the address of a device if it is connected, otherwise -1
 * 
 * @param device 	DeviceIdentifier of the device
 * @return int32_t 	Address of the device if it is connected, otherwise -1
 */
int32_t PeripheralHandler::getDeviceAdress(const DeviceIdentifier& device) const {
	for(auto& connectedDevice : m_connectedDevices)
	{
		if(connectedDevice.second == device)
		{
			return connectedDevice.first;
		}
	}
	return -1;
}

/**
 * @brief Uninstalls all devices from the handler
 * 
 */
void PeripheralHandler::uninstallAllDevices() {
	assert(m_commState == CommState::Stopped);

	m_hInTcdhandles.clear();
	m_hOutTcdhandles.clear();
	m_installedHostInStorages.clear();
	m_installedHostOutStorages.clear();

	m_gotNack.reset();
	m_deviceExpected = false;
	m_tcdIndex = 0;
	m_pingPongIndex = 0;
	m_commandQueue = std::queue<uint16_t>();
}

/**
 * @brief Returns the index of the ping pong buffer to which new data should be written
 * 
 * @return uint32_t 	Index of the ping pong buffer to which new data should be written
 */
uint32_t PeripheralHandler::getPingPongIndex() {
	return m_pingPongIndex;
}

/**
 * @brief Sends a sync command to all connected devices
 * 
 * @note The function will return Status::Warning if a new device was detected 
 * 			and won't send the sync command
 * 
 * @return Status Returns Status::Ok if the function was successful
 * 				  Returns Status::Timeout if the a timeout occured
 * 				  Returns Status::Warning if a new device was detected
 * 				  Returns Status::busStalled if the bus is stalled
 */
Status PeripheralHandler::sendSync() {
	// Make a general call with a sync command to all devices
	// The command may not be one of the following: 0b0000 0110, 0b0000 0100, 0b0000 0000

	if (m_stalled)
	{
		// we are stalled, no normal operation possible
		// just check if we still are stalled
		m_commState = CommState::Stopped;
		if (isUnstalled())
		{
			m_connectedDevicesChanged = true;
			SEGGER_RTT_printf(0, "I2C%d is unstalled!\n", m_i2cIndex + 1);
			return Status::Warning;
		}
		return Status::busStalled;
	}

	if(!m_deviceExpected && m_gotNack.has_value() && !*m_gotNack)
	{
		// We expected that there was no devices, but we got an ack (because we got no nack)
		// -> a new device is present
		m_connectedDevicesChanged = true;
		m_commState = CommState::Stopped;
		m_deviceExpected = true;
		SEGGER_RTT_printf(0, "I2C%d detected a device\n", m_i2cIndex + 1);
		return Status::Warning;
	}
	m_gotNack = false;

	// We do allways expect a nack, as for some reason expecting nacks doesn't work
	static std::array<uint16_t, 2> syncCommand = {
		LPI2C_MTDR_CMD(4) | 0x00u | kLPI2C_Write, //General Call 
		LPI2C_MTDR_CMD(2)};//Stop

	m_commState = CommState::Sync;

	// Send the command to all devices
	return sendCommand(std::span{ syncCommand });
}

/**
 * @brief Writes data to a register of a device
 * 
 * @param deviceAddress 		Address of the device
 * @param registerType 		Address of the register
 * @param data 					Data to write to the register
 * @return Status 				Returns Status::Ok if the function was successful
 * 				  				Returns Status::DeviceNotConnected if the device is not connected
 */
Status PeripheralHandler::writeRegister(uint8_t deviceAddress, DeviceRegisterType registerType, std::span<const std::byte> data) {
	assert(registerType == DeviceRegisterType::CommonDeviceConfiguration || 
			registerType == DeviceRegisterType::DeviceSpecificConfiguration);

	// Start transmission
	status_t i2cStatus = LPI2C_MasterStart(m_i2cBase, deviceAddress, kLPI2C_Write);
	if(i2cStatus != kStatus_Success)
	{
		return Status::DeviceNotConnected;
	}
	
	// Send control byte
	uint8_t ctrlByte = 	CONTROL_BYTE_ADDR(static_cast<uint8_t>(registerType)) | 
						CONTROL_BYTE_HOSTOUT | 
						CONTROL_BYTE_REGISTER;
	i2cStatus = LPI2C_MasterSend(m_i2cBase, &ctrlByte, 1);
	if(i2cStatus != kStatus_Success)
	{
		return Status::Error;
	}

	// Send data (LPI2C_MasterSend doesn't alter the data, so we can cast away the const)
	i2cStatus = LPI2C_MasterSend(m_i2cBase, 
		const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data.data())), 
		data.size());
	if(i2cStatus != kStatus_Success)
	{
		return Status::Error;
	}

	// Stop transmission
	i2cStatus = LPI2C_MasterStop(m_i2cBase);
	if(i2cStatus != kStatus_Success)
	{
		return Status::Error;
	}
	return Status::Ok;
}

/**
 * @brief Reads data from a register of a device
 * 
 * @param deviceAddress 		Address of the device
 * @param registerType 		Address of the register
 * @param data 					Data to write to the register
 * @return Status 				Returns Status::Ok if the function was successful
 * 				  				Returns Status::DeviceNotConnected if the device is not connected
 * 				  				Returns Status::Error it another error occured
 */
Status PeripheralHandler::readRegister(uint8_t deviceAddress, DeviceRegisterType registerType, std::span<std::byte> data) {

	// Start transmission
	status_t i2cStatus = LPI2C_MasterStart(m_i2cBase, deviceAddress, kLPI2C_Write);
	if(i2cStatus != kStatus_Success)
	{
		return Status::DeviceNotConnected;
	}
	
	// Send control byte
	uint8_t ctrlByte = 	CONTROL_BYTE_ADDR(static_cast<uint8_t>(registerType)) | 
						CONTROL_BYTE_HOSTIN | 
						CONTROL_BYTE_REGISTER;
	i2cStatus = LPI2C_MasterSend(m_i2cBase, &ctrlByte, 1);
	if(i2cStatus != kStatus_Success)
	{
		return i2cStatus==kStatus_LPI2C_Nak?Status::DeviceNotConnected:Status::Error;
	}

	// Switch to receiving mode
	i2cStatus = LPI2C_MasterRepeatedStart(m_i2cBase, deviceAddress, kLPI2C_Read);
	if(i2cStatus != kStatus_Success)
	{
		return Status::Error;
	}

	// Receive data
	i2cStatus = LPI2C_MasterReceive(m_i2cBase, reinterpret_cast<uint8_t*>(data.data()), data.size());
	if(i2cStatus != kStatus_Success)
	{
		return Status::Error;
	}

	// Stop transmission
	i2cStatus = LPI2C_MasterStop(m_i2cBase);
	if(i2cStatus != kStatus_Success)
	{
		return Status::Error;
	}
	return Status::Ok;
}

Status PeripheralHandler::probeAddress(uint8_t deviceAddress) {
	uint8_t status;
	return readRegister(deviceAddress, DeviceRegisterType::Status, std::span{reinterpret_cast<std::byte*>(&status), 1});
}


/**
 * @brief Starts a new cycle
 * 
 * @return Status Returns Status::Ok if the function was successful
 * 				  Returns Status::Error if the function was called while the handler was not in idle state
 * 				  Returns Status::Timeout when a timeout occured
 */
Status PeripheralHandler::startCycle() {
	if(m_commState != CommState::Sync)
	{
		// It is not allowed to start a cycle when not in idle state
		return Status::Error;
	}

	// Don't do anything if there are no devices
	// Note each device has at least one inTcd
	if(m_hInTcdhandles.size() == 0)
	{
		return Status::Ok;
	}

	m_tcdIndex = 0;

	// Enable rx dma
	LPI2C_MasterEnableDMA(m_i2cBase, false, true);
	EDMA_EnableChannelRequest(m_dmaBase, m_dmaChannel);

	// TODO: Maybe send command at i2c callback (So we don't have to wait for the completion of the sync transfer)
	// Write the first receive command to the fifo
	// Note: This will trigger the dma to start
	return sendCommand(std::span{ m_hInTcdhandles[0].command });
}

/**
 * @brief Enters real time mode
 * 
 * @return Status Returns Status::Ok if the function was successful
 * 				  Returns Status::Error if the function was called while the handler was not in stopped state
 */
Status PeripheralHandler::enterRealTimeMode() {
	if(m_commState != CommState::Stopped)
	{
		// It is not allowed to enter realTime mode when not in stopped state
		return Status::Error;
	}
	LPI2C_MasterEnableInterrupts(m_i2cBase, LPIIC_INTERRUPTS);
	m_commState = CommState::Idle;
	return Status::Ok;
}

/**
 * @brief Exits real time mode
 * 
 * @return Status Returns Status::Ok if the function was successful
 * 				  Returns Status::Error if the function was called while the handler was not in idle state
 */
Status PeripheralHandler::exitRealTimeMode() {
	while(m_commState > CommState::Idle)
	{
		// Wait until the sync command is done
		// TODO: Add a timeout
	}
	if(m_commState > CommState::Idle)
	{
		// It is not allowed to exit realTime mode when not in idle state
		return Status::Error;
	}
	LPI2C_MasterDisableInterrupts(m_i2cBase, LPIIC_INTERRUPTS);
	m_commState = CommState::Stopped;
	return Status::Ok;
}

/**
 * @brief Interrupt handler for the dma hardware
 * 
 */
void PeripheralHandler::dmaInterruptHandler() {
	uint32_t status = EDMA_GetChannelStatusFlags(m_dmaBase, m_dmaChannel);
    EDMA_ClearChannelStatusFlags(m_dmaBase, m_dmaChannel, status);

	// TODO: Check which interrupt occured and handle it
	
	m_tcdIndex++;

	//Start next rx tcd if there is one
	if(m_tcdIndex < m_hInTcdhandles.size())
	{
		Status cmdStatus = sendCommand(std::span{ m_hInTcdhandles[m_tcdIndex].command });
		if (cmdStatus != Status::Ok)
		{
			SEGGER_RTT_printf(0, "I2C%d had a timeout while sending a command\n", (uint32_t(m_i2cBase) - LPI2C1_BASE)/0x4000 + 1);
			return;
		}
	}
	else if(m_tcdIndex == m_hInTcdhandles.size())
	{
		if(m_hOutTcdhandles.size() > 0)
		{
			// Change direction to tx
			m_commState = CommState::Transmitting;
			LPI2C_MasterEnableDMA(m_i2cBase, true, false);
			EDMA_EnableChannelRequest(m_dmaBase, m_dmaChannel);
		}
		else
		{
			// All tcds are done
			m_commState = CommState::Idle;
		}
		

		//swap ping pong buffer
		m_pingPongIndex ^= 1;
		
		// Call functions for handling the data
		m_processDataCallback();
	}
	else
	{
		// All tx-tcds are done
		m_commState = CommState::Idle;
	}
	
}

/**
 * @brief Interrupt handler for the i2c hardware
 * 
 */
void PeripheralHandler::i2cInterruptHandler() {
	uint32_t i2cIndex = (uint32_t(m_i2cBase) - LPI2C1_BASE)/0x4000 + 1;
	// If we receive a nack, no device are present anymore -> stop transmission
	
	if(LPI2C_MasterGetStatusFlags(m_i2cBase) & kLPI2C_MasterNackDetectFlag)
	{
		// Clear flag
		LPI2C_MasterClearStatusFlags(m_i2cBase, kLPI2C_MasterNackDetectFlag);
		m_gotNack = true;

		if(m_deviceExpected)
		{
			// first stop the transfer
			auto commState = m_commState;
			hardStop();

			// We expected an ack (i.e. a device) but it wasn't there
			// -> connected devices changed
			m_connectedDevicesChanged = true;


			if (commState == CommState::Sync)
			{
				// Sync command was not acknowledged -> there are no devices present
				m_deviceExpected = false;
			}

			SEGGER_RTT_printf(0, "I2C%d %s\n", 
								i2cIndex, 
								(commState == CommState::Sync) ? "detected no devices" : "lost device");
		}
	}

	if(LPI2C_MasterGetStatusFlags(m_i2cBase) & kLPI2C_MasterStopDetectFlag)
	{
		// Clear flag
		LPI2C_MasterClearStatusFlags(m_i2cBase, kLPI2C_MasterStopDetectFlag);

		// Stop condition was detected -> if we where in sync mode 
		//  and there are devices present we can start receiving
		if(hasInstalledDevices() && m_commState == CommState::Sync)
		{
			m_commState = CommState::Receiving;
		}
		//  and there are no devices present we can stop the transmission
		else if(!hasInstalledDevices() && m_commState == CommState::Sync)
		{
			m_commState = CommState::Idle;
		}
	}

	if(LPI2C_MasterGetStatusFlags(m_i2cBase) & kLPI2C_MasterTxReadyFlag)
	{
		// Clear flag
		LPI2C_MasterClearStatusFlags(m_i2cBase, kLPI2C_MasterTxReadyFlag);

		uint32_t fifoAvailable = I2C_FIFO_DEPTH - 
					((m_i2cBase->MSR & LPI2C_MSR_TDF_MASK) >> LPI2C_MSR_TDF_SHIFT);

		// The tx fifo is ready for new commands from the queue
		while(!m_commandQueue.empty() && fifoAvailable > 0)
		{
			uint16_t command = m_commandQueue.front();
			m_commandQueue.pop();
			m_i2cBase->MTDR = command;
			fifoAvailable--;
		}

		// If there are no more commands, disable the tx interrupt
		if(m_commandQueue.empty())
		{
			LPI2C_MasterDisableInterrupts(m_i2cBase, kLPI2C_MasterTxReadyFlag);
		}
	}

	if(LPI2C_MasterGetStatusFlags(m_i2cBase) & kLPI2C_MasterFifoErrFlag)
	{
		/* Reset fifos. */
        m_i2cBase->MCR |= LPI2C_MCR_RRF_MASK | LPI2C_MCR_RTF_MASK;


		SEGGER_RTT_printf(0, "I2C%d had a fifo error\n", i2cIndex);

		// clear flag
		LPI2C_MasterClearStatusFlags(m_i2cBase, kLPI2C_MasterFifoErrFlag);
	}

	if(LPI2C_MasterGetStatusFlags(m_i2cBase) & kLPI2C_MasterArbitrationLostFlag)
	{
		SEGGER_RTT_printf(0, "I2C%d had an arbitration lost\n", i2cIndex);

		// clear flag
		LPI2C_MasterClearStatusFlags(m_i2cBase, kLPI2C_MasterArbitrationLostFlag);
	}
}

/**
 * @brief Fills a tcd for a In transfer and appends it to the tcd list
 * 
 * @param data 			MemoryRegion where the data should be stored
 * @param deviceAddress Address of the device
 */
Status PeripheralHandler::appendTcdIn(MemoryRegion data, uint32_t deviceAddress) {
	TcdInHandle tcdHandle;
	// build command
	tcdHandle.command.push_back(LPI2C_MTDR_CMD(4) | (deviceAddress & 0x7F) << 1 | kLPI2C_Read);
	int32_t totalLen = data.size;
	while(totalLen > 0)
	{
		uint32_t len = totalLen > 256 ? 255 : (totalLen - 1); // -1 because the actual length is len + 1
		tcdHandle.command.push_back(LPI2C_MTDR_CMD(1) | (len & 0xFF));
		totalLen -= 256;
	}
	tcdHandle.command.push_back(LPI2C_MTDR_CMD(2));

	// build tcd
	for(auto& tcd : tcdHandle.tcd)
	{
		EDMA_TcdReset(&tcd);
		tcd.ATTR = DMA_ATTR_SSIZE(0) | DMA_ATTR_DSIZE(0);

		tcd.SADDR = (uint32_t)&m_i2cBase->MRDR;
		tcd.SOFF = 0;

		tcd.DADDR = (uint32_t)data.address;
		tcd.DOFF = 1;

		tcd.NBYTES = 1;
		tcd.CITER = data.size;
		tcd.BITER = data.size;
		tcd.SLAST = -data.size * 1;
		tcd.DLAST_SGA = 0; // Leave empty for now, will be filled by linkTcds

		tcd.CSR = DMA_CSR_ESG(1) | DMA_CSR_INTMAJOR(1);

		// Disable auto stop request as the channels may always be active
		//  because there will only be data in the fifo when the i2c is active
		EDMA_TcdEnableAutoStopRequest(&tcd, false);
	}

	tcdHandle.deviceAddress = deviceAddress;
	m_hInTcdhandles.push_back(tcdHandle);
	return Status::Ok;
}

/**
 * @brief Fills a tcd for a Out transfer
 * 
 * @param tcd 			Pointer to the tcd that should be filled
 * @param data 			Pointer to the data that should be send
 * @param length 		Length of the data in bytes
 * @param nextTcd 		Pointer to the next tcd in the chain
 * @param i2cHardware 	Pointer to the i2c hardware
 * @param size          Size of an element in 2^n bytes
 */
void PeripheralHandler::fillOutTcds(edma_tcd_t* tcd, uint8_t* data, uint32_t length, uint32_t size)
{
    assert (size <= 4);
    uint32_t sizeInBytes = 1 << size;
    EDMA_TcdReset(tcd);
    tcd->ATTR = DMA_ATTR_SSIZE(size) | DMA_ATTR_DSIZE(size);

    tcd->SADDR = (uint32_t)data;
    tcd->SOFF = sizeInBytes;

    tcd->DADDR = (uint32_t)&m_i2cBase->MTDR;
    tcd->DOFF = 0;

    tcd->NBYTES = sizeInBytes;
    tcd->CITER = length;
    tcd->BITER = length;
    tcd->SLAST = -length * sizeInBytes;
    tcd->DLAST_SGA = 0; // Leave empty for now, will be filled by linkTcds

    // Disable interrupts because tx/out can be linked without software interaction
    tcd->CSR = DMA_CSR_ESG(1) | DMA_CSR_INTMAJOR(0);

    // Disable auto stop request as the channels may always be active
    //  because there will only be data in the fifo when the i2c is active
    EDMA_TcdEnableAutoStopRequest(tcd, false);
}

/**
 * @brief Fills a tcd-set for a Out transfer and appends it to the tcd list
 * 
 * @param data 			MemoryRegion where the data should be stored
 * @param deviceAddress Address of the device
 */
Status PeripheralHandler::appendTcdOut(std::array<MemoryRegion,2> data, uint32_t deviceAddress)  {
	m_hOutTcdhandles.push_back({});
	TcdOutHandle& tcdHandle = m_hOutTcdhandles[m_hOutTcdhandles.size() - 1];
	// build command
	uint8_t controlWord = 0x81; //Command for HOut PDS transfer
	tcdHandle.command[0] = LPI2C_MTDR_CMD(4) | (deviceAddress & 0x7F) << 1 | kLPI2C_Write; //start
	tcdHandle.command[1] = LPI2C_MTDR_CMD(0) | controlWord; //command phase
	/*
		Here comes the data phase, which is handled by dma
	*/
	tcdHandle.command[2] = LPI2C_MTDR_CMD(2); //stop

	// build tcd
	uint32_t pingPong = 0;
	for(auto& tcd : tcdHandle.tcds)
	{
		fillOutTcds(&tcd[0], (uint8_t*)&tcdHandle.command[0], 2, 1); // start phase (start + command)
		fillOutTcds(&tcd[1], (uint8_t*)data[pingPong].address, data[pingPong].size, 0); // data phase
		fillOutTcds(&tcd[2], (uint8_t*)&tcdHandle.command[2], 1, 1); // stop phase (stop)
		pingPong ^= 1;
	}

	tcdHandle.deviceAddress = deviceAddress;
	return Status::Ok;
}

/**
 * @brief Links all tcds together and handles the pingpong buffer switching
 * 
 * @return Status Returns Status::Ok if the function was successful
 */
Status PeripheralHandler::linkTcds() {
	// Link all tcds together
	// Note2: Due to pipelining the tcdOut of the prior cycle (pong) will be send directly after
	//        the tcdIn of the current cycle(ping). Therefore the (ping) tcdIn links to the tcdOut of
	// 		  the opposed ping-pong buffer (pong). The (pong)tcdOut links to the tcdIn of the (pong) buffer 
	//        as the tcdIn of the (ping) buffer will start in the next cycle.

	for(uint32_t pingPong = 0; pingPong < 2; pingPong++)
	{
		// Link tcds for data in
		uint32_t nextTcd = 0;
		uint32_t i = m_hInTcdhandles.size();
		while(i--)
		{
			EDMA_TcdEnableAutoStopRequest(&m_hInTcdhandles[i].tcd[pingPong], false);
			m_hInTcdhandles[i].tcd[pingPong].DLAST_SGA = nextTcd;
			nextTcd = (uint32_t)&m_hInTcdhandles[i].tcd[pingPong];
		}

		// Link tcds for data out
		nextTcd = 0;
		uint32_t handleIndex = m_hOutTcdhandles.size();
		while(handleIndex--)
		{
			// Link the tcds for the 3 phases
			for(int32_t phaseIndex = 2; phaseIndex >= 0; phaseIndex--)
			{
				m_hOutTcdhandles[handleIndex].tcds[pingPong][phaseIndex].DLAST_SGA = nextTcd;
				nextTcd = (uint32_t)&m_hOutTcdhandles[handleIndex].tcds[pingPong][phaseIndex];
				EDMA_TcdEnableAutoStopRequest(&m_hOutTcdhandles[handleIndex].tcds[pingPong][phaseIndex], false);
			}
			// Update the command addresses to the commands
			m_hOutTcdhandles[handleIndex].tcds[pingPong][0].SADDR = (uint32_t)&m_hOutTcdhandles[handleIndex].command[0];
			m_hOutTcdhandles[handleIndex].tcds[pingPong][2].SADDR = (uint32_t)&m_hOutTcdhandles[handleIndex].command[2];

			// Disable interrupts that may have been enabled for a prior last tcd
			m_hOutTcdhandles[handleIndex].tcds[pingPong][2].CSR &= ~DMA_CSR_INTMAJOR_MASK;
		}
	}

	// now all but the last tcds are linked together, as per Note2 
	//  there is special care needed for the last tcds, as we need cross ping-pong buffer linking
	for(uint32_t pingPong = 0; pingPong < 2; pingPong++)
	{
		// Link last tcd in to opposing tcdOut
		edma_tcd_t* lastTcdIn = &m_hInTcdhandles[m_hInTcdhandles.size() - 1].tcd[pingPong];
		edma_tcd_t* lastTcdInOpposed = &m_hInTcdhandles[m_hInTcdhandles.size() - 1].tcd[pingPong ^ 1];
		edma_tcd_t* lastTcdOutOpposed = &m_hOutTcdhandles[m_hOutTcdhandles.size() - 1].tcds[pingPong ^ 1][2];
		edma_tcd_t* firstTcdIn = &m_hInTcdhandles[0].tcd[pingPong];
		edma_tcd_t* firstTcdInOpposed = &m_hInTcdhandles[0].tcd[pingPong ^ 1];
		edma_tcd_t* firstTcdOutOpposed = &m_hOutTcdhandles[0].tcds[pingPong ^ 1][0];

		// When there is no output data, the last tcd in points to the first tcd in
		if(m_hOutTcdhandles.size() == 0)
		{
			lastTcdInOpposed->DLAST_SGA = (uint32_t)firstTcdIn;
			EDMA_TcdEnableAutoStopRequest(lastTcdIn, true);
			continue;
		}

		lastTcdIn->DLAST_SGA = (uint32_t)firstTcdOutOpposed;
		lastTcdOutOpposed->DLAST_SGA = (uint32_t)firstTcdInOpposed;
		lastTcdOutOpposed->CSR |= DMA_CSR_INTMAJOR(1);

		// Enable auto stop request for direction changes
		EDMA_TcdEnableAutoStopRequest(lastTcdIn, true);
		EDMA_TcdEnableAutoStopRequest(lastTcdOutOpposed, true);
	}

	// Install the first tcd in the dma channel
	EDMA_InstallTCD(m_dmaBase, m_dmaChannel, &m_hInTcdhandles[0].tcd[0]);

	return Status::Ok;
} 

/**
 * @brief Sends a command to the i2c hardware, currently commands longer than the fifo depth 
 * 			are supported but busy wait until there is space in the fifo
 * 
 * @param command 	Pointer to the command that should be send
 * @param timeout 	Timeout in ms
 * 
 * @return Status::Ok if the command was send successfully
 * 		   Status::Timeout if there was a timeout
 */
Status PeripheralHandler::sendCommand(std::span<const uint16_t> command, uint32_t timeout) {
	// Check if the command fits into the fifo
	uint32_t fifoCount = ((m_i2cBase->MFSR & LPI2C_MFSR_TXCOUNT_MASK) >> LPI2C_MFSR_TXCOUNT_SHIFT);
	uint32_t fifoAvailable = I2C_FIFO_DEPTH - fifoCount;
	if(fifoAvailable > command.size())
	{
		// Write the entire command into the fifo
		for(size_t i = 0; i < command.size(); i++)
		{
			// Recheck if the fifo is still available
			if(((m_i2cBase->MFSR & LPI2C_MFSR_TXCOUNT_MASK) >> LPI2C_MFSR_TXCOUNT_SHIFT) < I2C_FIFO_DEPTH)
			{
				m_i2cBase->MTDR = command[i];
			}
			else
			{
				// The fifo is not available anymore, return an error
				return Status::Error;
			}
		}
	}
	else
	{
		// Write as much as possible into the fifo
		for(size_t i = 0; i < fifoAvailable; i++)
		{
			m_i2cBase->MTDR = command[i];
		}

		// The rest of the command will be added to a 
		//  buffer and send when the fifo has space again
		for (size_t i = fifoAvailable; i < command.size(); i++)
		{
			
			if (m_commandQueue.size() < 100)
			{
				m_commandQueue.push(command[i]);
			}
			else
			{
				m_stalled = true;
				SEGGER_RTT_printf(0, "I2C%d is stalled\n", (uint32_t(m_i2cBase) - LPI2C1_BASE)/0x4000 + 1);
				m_connectedDevicesChanged = true;
				hardStop();
			}
		}
		
		// Activate the interrupt for the fifo empty flag
		LPI2C_MasterEnableInterrupts(m_i2cBase, kLPI2C_MasterTxReadyFlag);
	}

	return Status::Ok;
}

/**
 * @brief Stops all i2c and dma communication and resets the states. 
 * 			Use this for stopping the communication mid cycle
 * 
 * @return Status::Ok if the function was successful
 */
Status PeripheralHandler::hardStop()
{
	// Disable dma requests	
	LPI2C_MasterEnableDMA(m_i2cBase, false, false);

	// Reset fifos
	m_i2cBase->MCR |= LPI2C_MCR_RRF_MASK | LPI2C_MCR_RTF_MASK;

	// If master is still busy and has not send out stop signal yet.
	if (LPI2C_MasterGetStatusFlags(m_i2cBase) & ((uint32_t)kLPI2C_MasterStopDetectFlag))
	{
		// Send a stop command to finalize the transfer.
		m_i2cBase->MTDR = LPI2C_MTDR_CMD(2);//Stop
	}

	// Disable dma
	while (m_dmaBase->HRS & (1 << m_dmaChannel))
	{
		// Wait until the dma channel is not active anymore
	}
	EDMA_DisableChannelRequest(m_dmaBase, m_dmaChannel);

	// Cancel the edma transfer
	m_dmaBase->CR |= DMA_CR_CX(1 << m_dmaChannel);

	// Reset the state
	m_commState = CommState::Stopped;
	uninstallAllDevices();

	return Status::Ok;
}

/**
 * @brief Checks if the bus is still stalled. 
 * 			If the tx fifo is not empty, the bus is still stalled 
 * 					(as we are the only place that writes to it right now)
 * 			If the tx fifo is empty, we try to write
 * 
 * @return true 
 * @return false 
 */
bool PeripheralHandler::isUnstalled() {
	// try trigger master busy via fifo reset
	m_i2cBase->MCR |= LPI2C_MCR_RRF_MASK | LPI2C_MCR_RTF_MASK;

	// write a dummy command to trigger a reevaluation of the bus state
	m_i2cBase->MTDR = LPI2C_MTDR_CMD(4);
	m_i2cBase->MTDR = LPI2C_MTDR_CMD(2);


	//volatile uint32_t fifoCount = ((m_i2cBase->MFSR & LPI2C_MFSR_TXCOUNT_MASK) >> LPI2C_MFSR_TXCOUNT_SHIFT);
	volatile uint32_t busBusy = m_i2cBase->MSR & LPI2C_MSR_BBF_MASK;
	m_stalled = (busBusy);
	return !m_stalled;
}

// Interrupt handlers
extern "C" {
void DMA4_DMA20_IRQHandler(void)
{
	PeripheralHandler::DMA4_DMA20_IRQHandler();
}

void DMA5_DMA21_IRQHandler(void)
{
	PeripheralHandler::DMA5_DMA21_IRQHandler();
}

void DMA6_DMA22_IRQHandler(void)
{
	PeripheralHandler::DMA6_DMA22_IRQHandler();
}

void DMA7_DMA23_IRQHandler(void)
{
	PeripheralHandler::DMA7_DMA23_IRQHandler();
}

void LPI2C1_IRQHandler()
{
	PeripheralHandler::LPI2C1_IRQHandler();
}

void LPI2C2_IRQHandler()
{
	PeripheralHandler::LPI2C2_IRQHandler();
}

void LPI2C3_IRQHandler()
{
	PeripheralHandler::LPI2C3_IRQHandler();
}

void LPI2C4_IRQHandler()
{
	PeripheralHandler::LPI2C4_IRQHandler();
}
} /* extern "C" */

void PeripheralHandler::DMA4_DMA20_IRQHandler() {
	if(handlers[0] != nullptr){
		handlers[0]->dmaInterruptHandler();
	}
}

void PeripheralHandler::DMA5_DMA21_IRQHandler() {
	if(handlers[1] != nullptr){
		handlers[1]->dmaInterruptHandler();
	}
}

void PeripheralHandler::DMA6_DMA22_IRQHandler() {
	if(handlers[2] != nullptr){
		handlers[2]->dmaInterruptHandler();
	}
}

void PeripheralHandler::DMA7_DMA23_IRQHandler() {
	if(handlers[3] != nullptr){
		handlers[3]->dmaInterruptHandler();
	}
}

void PeripheralHandler::LPI2C1_IRQHandler() {
	if(handlers[0] != nullptr){
		handlers[0]->i2cInterruptHandler();
	}
}

void PeripheralHandler::LPI2C2_IRQHandler() {
	if(handlers[1] != nullptr){
		handlers[1]->i2cInterruptHandler();
	}
}

void PeripheralHandler::LPI2C3_IRQHandler() {
	if(handlers[2] != nullptr){
		handlers[2]->i2cInterruptHandler();
	}
}

void PeripheralHandler::LPI2C4_IRQHandler() {
	if(handlers[3] != nullptr){
		handlers[3]->i2cInterruptHandler();
	}
}



