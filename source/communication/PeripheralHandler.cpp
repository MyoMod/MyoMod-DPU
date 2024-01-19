/*
 * PeripheralHandler.cpp
 *
 *  Created on: Jan 9, 2024
 *      Author: leon
 */

#include "PeripheralHandler.h"

namespace freesthetics {

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
PeripheralHandler::PeripheralHandler(DMA_Type* dma, uint32_t i2cIndex, std::function<void(void)> processDataCallback, bool highSpeed):
	dmaBase{ dma }, 
	commState{CommState:: Stopped }, 
	dmaChannel{ DMA_LOWEST_CHANNEL + (i2cIndex - 1) }, // i2cIndex is 1 based, but the array is 0 based
	tcdIndex{ 0 },
	pingPongIndex{ 0 },
	devicesChanged{ true }, // Set to true to trigger the first listNewDevices call
	processDataCallback{ processDataCallback }
{
	i2cIndex--; // i2cIndex is 1 based, but the array is 0 based
	/* Init dma hardware */

	// Enable DMA clock 
    CLOCK_EnableClock(kCLOCK_Dma);

    DMAMUX_Init(DMAMUX_BASE);
    // Procedure from manual 5.5.1
    // 2. Clear the CHCFG[ENBL] and CHCFG[TRIG] fields of the DMA channel.
    DMAMUX_DisableChannel(DMAMUX_BASE, dmaChannel);

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

    EDMA_Init(dmaBase, &dmaConfig);

	// Enable interrupts
    // individual interrupts are enabled in the corresponding tcd
	IRQn interrupt{ static_cast<IRQn>((static_cast<uint32_t>(DMA0_DMA16_IRQn) + (dmaChannel%16)))};
    EnableIRQ(interrupt);

    // 4. Configure the corresponding timer.
    // 5. Select the source to be routed to the DMA channel. Write to the corresponding
    //      CHCFG register, ensuring that the CHCFG[ENBL] and CHCFG[TRIG] fields are
    //      set
	std::array<_dma_request_source,4> dmaSources{ 
		kDmaRequestMuxLPI2C1, kDmaRequestMuxLPI2C2, 
		kDmaRequestMuxLPI2C3, kDmaRequestMuxLPI2C4 };
	assert(i2cIndex < 4);
    DMAMUX_SetSource(DMAMUX_BASE, dmaChannel, dmaSources[i2cIndex]);
    DMAMUX_EnableChannel(DMAMUX_BASE, dmaChannel);

	/* Init LPI2C Hardware */
	std::array<LPI2C_Type*,4> i2cBases{ LPI2C1, LPI2C2, LPI2C3, LPI2C4 };
	i2cBase = i2cBases[i2cIndex];

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

	// Change baudrate
	i2cConfig.baudRate_Hz = highSpeed ? 1'000'000U : 400'000U;

	// Init i2c hardware
	LPI2C_MasterInit(i2cBase, &i2cConfig, LPI2C_CLOCK_FREQUENCY);

	// Activate IRQ
	LPI2C_MasterEnableInterrupts(i2cBase, kLPI2C_MasterFifoErrFlag | kLPI2C_MasterNackDetectFlag);
	interrupt = static_cast<IRQn>((static_cast<uint32_t>(LPI2C1_IRQn) + i2cIndex));
    EnableIRQ(interrupt);

	// Set tx and rx fifo watermark
	LPI2C_MasterSetWatermarks(i2cBase, 0, 0);

	// register handler
	PeripheralHandler::handlers[i2cIndex] = this;
}

/**
 * @brief Destroy the PeripheralHandler object and deinitializes the hardware and
 * 			 removes the interrupt handlers
 * 
 */
PeripheralHandler::~PeripheralHandler() {
	LPI2C_MasterDeinit(i2cBase);
	IRQn interrupt{ static_cast<IRQn>((static_cast<uint32_t>(LPI2C1_IRQn) + (i2cBase - LPI2C1)))};
	DisableIRQ(interrupt);

	interrupt = static_cast<IRQn>((static_cast<uint32_t>(DMA0_DMA16_IRQn) + (dmaChannel%16)));
    DisableIRQ(interrupt);

	DMAMUX_DisableChannel(DMAMUX_BASE, dmaChannel);
	EDMA_ResetChannel(dmaBase, dmaChannel);

	uint32_t i2cIndex = dmaChannel - DMA_LOWEST_CHANNEL;

	handlers[i2cIndex] = nullptr;
}

/**
 * @brief Returns a list of DeviceDescriptors for all devices that are found
 * 			but are not added to the handler yet
 * 
 * @param devices 	Pointer to a vector where the devices should be stored
 * @return Status::Ok if the function was successful
 * 		   Status::Error if the function was called while the handler was not in stopped state
 * 		   Status::Warning if the function was called, but there were no new devices
 */
Status PeripheralHandler::listNewDevices(std::vector<DeviceDescriptor>* devices) {
	if(commState != CommState::Stopped)
	{
		// It is not allowed to list devices while in realTime mode
		return Status::Error;
	}

	if(devicesChanged)
	{
		// TODO: scan for new devices and get their descriptors
		devices->push_back(DeviceDescriptor{
			.deviceType = "Elctr6Ch",
			.deviceIdentifier = "Elctr1",
			.peripheralIndex = -1,
			.deviceAddress = 0x08,
			.name = ""
		});
		devices->push_back(DeviceDescriptor{
			.deviceType = "BarDis6Ch",
			.deviceIdentifier = "Display1",
			.peripheralIndex = -1,
			.deviceAddress = 0x18,
			.name = ""
		});
		devicesChanged = false;
	}
	else
	{
		// Function was called, but there were no new devices, return the empty vector
		// and return a warning
		return Status::Warning;
	}
	return Status::Ok;
}

/**
 * @brief Returns true if new devices were detected since the last call to listNewDevices
 * 
 * @return true if new devices were detected
 * @return false if no new devices were detected
 */
bool PeripheralHandler::detectedNewDevices() {
	return devicesChanged;
}

/**
 * @brief Adds a device to the handler. 
 * 
 * @param address 		Address of the device
 * @param dataInBuffer 	MemoryRegion where the data from the device should be stored
 * @param dataOutBuffer MemoryRegion where the data for the device should be read from
 * @return Status::Ok if the function was successful
 * 		   Status::Error if the function was called, but the device was already added
 */
Status PeripheralHandler::addDevice(uint32_t address, MemoryRegion dataInBuffer, std::array<MemoryRegion,2> dataOutBuffer) {
	
	assert(commState == CommState::Stopped);

	// Install tcds+commands for data in and out
	if(dataInBuffer.size > 0)
	{
		// Check if address is already in use
		for(auto& handle : hInTcdhandles)
		{
			if(handle.deviceAddress == address)
			{
				return Status::Error;
			}
		}

		//Install 
		appendTcdIn(dataInBuffer, address);

	}
	if(dataOutBuffer[0].size > 0)
	{
		// Check if address is already in use
		for(auto& handle : hOutTcdhandles)
		{
			if(handle.deviceAddress == address)
			{
				return Status::Error;
			}
		}

		//Install tcds for data out
		appendTcdOut(dataOutBuffer, address);
	}

	// Update all links
	// Note: This will always be done for all devices, as the addition of a device 
	//        may trigger the vector to reallocate, which will invalidate all pointers
	//        to the tcds
	linkTcds();

	return Status::Ok;
}

/**
 * @brief Removes a device from the handler
 * 
 * @param address 	Address of the device
 * @return Status::Ok if the function was successful
 * 		   Status::Error if the function was called, but the device was not added
 */
Status PeripheralHandler::removeDevice(uint32_t address) {
	bool foundDevice = false;

	assert(commState == CommState::Stopped);

	// Remove tcds for data in
	for(auto it = hInTcdhandles.begin(); it != hInTcdhandles.end(); it++)
	{
		if(it->deviceAddress == address)
		{
			hInTcdhandles.erase(it);
			foundDevice = true;
			break;
		}
	}

	// Remove tcds for data out
	for(auto it = hOutTcdhandles.begin(); it != hOutTcdhandles.end(); it++)
	{
		if(it->deviceAddress == address)
		{
			hOutTcdhandles.erase(it);
			foundDevice = true;
			break;
		}
	}

	if(foundDevice)
	{
		// Update all links
		// Note: This will always be done for all devices, as the addition of a device 
		//        may trigger the vector to reallocate, which will invalidate all pointers
		//        to the tcds
		linkTcds();
	}
	else
	{
		// Device was not found
		return Status::Error;
	}

	return Status::Ok;
}

/**
 * @brief Returns the index of the ping pong buffer to which new data should be written
 * 
 * @return uint32_t 	Index of the ping pong buffer to which new data should be written
 */
uint32_t PeripheralHandler::getPingPongIndex() {
	return pingPongIndex;
}

/**
 * @brief Sends a sync command to all devices
 * 
 * @return Status Returns Status::Ok if the function was successful
 */
Status PeripheralHandler::sendSync() {
	// Make a general call with a sync command to all devices
	// The command may not be one of the following: 0b0000 0110, 0b0000 0100, 0b0000 0000


	static std::array<uint16_t, 3> syncCommand = {
		LPI2C_MTDR_CMD(4) | 0x00 | kLPI2C_Write, //General Call 
		LPI2C_MTDR_CMD(0) | 0xA5, //Sync command
		LPI2C_MTDR_CMD(2)};//Stop

	// Send the command to all devices
	sendCommand(std::span{ syncCommand });
	return Status::Ok;
}

/**
 * @brief Starts a new cycle
 * 
 * @return Status Returns Status::Ok if the function was successful
 * 				  Returns Status::Error if the function was called while the handler was not in idle state
 */
Status PeripheralHandler::startCycle() {
	if(commState != CommState::Idle)
	{
		// It is not allowed to start a cycle when not in idle state
		return Status::Error;
	}

	// Don't do anything if there are no devices
	// Note each device has at least one inTcd
	if(hInTcdhandles.size() == 0)
	{
		return Status::Ok;
	}

	tcdIndex = 0;
	commState = CommState::Receiving;

	// Enable rx dma
	LPI2C_MasterEnableDMA(i2cBase, false, true);
	EDMA_EnableChannelRequest(dmaBase, dmaChannel);

	// TODO: Maybe send command at i2c callback (So we don't have to wait for the completion of the sync transfer)
	// Write the first receive command to the fifo
	// Note: This will trigger the dma to start
	sendCommand(std::span{ hInTcdhandles[0].command });
	return Status::Ok;
}

/**
 * @brief Enters real time mode
 * 
 * @return Status Returns Status::Ok if the function was successful
 * 				  Returns Status::Error if the function was called while the handler was not in stopped state
 */
Status PeripheralHandler::enterRealTimeMode() {
	if(commState != CommState::Stopped)
	{
		// It is not allowed to enter realTime mode when not in stopped state
		return Status::Error;
	}
	commState = CommState::Idle;
	return Status::Ok;
}

/**
 * @brief Exits real time mode
 * 
 * @return Status Returns Status::Ok if the function was successful
 * 				  Returns Status::Error if the function was called while the handler was not in idle state
 */
Status PeripheralHandler::exitRealTimeMode() {
	if(commState > CommState::Idle)
	{
		// It is not allowed to exit realTime mode when not in idle state
		return Status::Error;
	}
	commState = CommState::Stopped;
	return Status::Ok;
}

/**
 * @brief Interrupt handler for the dma hardware
 * 
 */
void PeripheralHandler::dmaInterruptHandler() {
	uint32_t status = EDMA_GetChannelStatusFlags(dmaBase, dmaChannel);
    EDMA_ClearChannelStatusFlags(dmaBase, dmaChannel, status);

	// TODO: Check which interrupt occured and handle it
	
	tcdIndex++;

	//Start next rx tcd if there is one
	if(tcdIndex < hInTcdhandles.size())
	{
		// TODO: Handle tcdIns with length > 256
		sendCommand(std::span{ hInTcdhandles[tcdIndex].command });
	}
	else if(tcdIndex == hInTcdhandles.size())
	{
		// Change direction to tx
		commState = CommState::Transmitting;
		LPI2C_MasterEnableDMA(i2cBase, true, false);
		EDMA_EnableChannelRequest(dmaBase, dmaChannel);

		//swap ping pong buffer
		pingPongIndex ^= 1;
		
		// Call functions for handling the data
		processDataCallback();
	}
	else if(tcdIndex == hInTcdhandles.size() + 1)
	{
		// All tcds are done, send sync command
		commState = CommState::Idle;
	}
	
}

/**
 * @brief Interrupt handler for the i2c hardware
 * 
 */
void PeripheralHandler::i2cInterruptHandler() {
	uint32_t i2cIndex = (uint32_t(i2cBase) - LPI2C1_BASE)/0x4000 + 1;
	// If we receive a nack, the device is not present anymore -> stop transmission
	if(LPI2C_MasterGetStatusFlags(i2cBase) & kLPI2C_MasterNackDetectFlag)
	{
		/* Reset fifos. */
        i2cBase->MCR |= LPI2C_MCR_RRF_MASK | LPI2C_MCR_RTF_MASK;

        /* If master is still busy and has not send out stop signal yet. */
        if (LPI2C_MasterGetStatusFlags(i2cBase) & ((uint32_t)kLPI2C_MasterStopDetectFlag))
        {
            /* Send a stop command to finalize the transfer. */
            i2cBase->MTDR = LPI2C_MTDR_CMD(2);//Stop
        }

		SEGGER_RTT_printf(0, "I2C%d detected a NAck\n", i2cIndex);

		// Clear flag
		LPI2C_MasterClearStatusFlags(i2cBase, kLPI2C_MasterNackDetectFlag);
	}

	if(LPI2C_MasterGetStatusFlags(i2cBase) & kLPI2C_MasterFifoErrFlag)
	{
		/* Reset fifos. */
        i2cBase->MCR |= LPI2C_MCR_RRF_MASK | LPI2C_MCR_RTF_MASK;


		SEGGER_RTT_printf(0, "I2C%d had a fifo error\n", i2cIndex);

		// clear flag
		LPI2C_MasterClearStatusFlags(i2cBase, kLPI2C_MasterFifoErrFlag);
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

		tcd.SADDR = (uint32_t)&i2cBase->MRDR;
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
	hInTcdhandles.push_back(tcdHandle);
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

    tcd->DADDR = (uint32_t)&i2cBase->MTDR;
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
	hOutTcdhandles.push_back({});
	TcdOutHandle& tcdHandle = hOutTcdhandles[hOutTcdhandles.size() - 1];
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
	// Note: This will always be done for all devices, as the addition of a device
	//        may trigger the vector to reallocate, which will invalidate all pointers
	//        to the tcds
	// Note2: Due to pipelining the tcdOut of the prior cycle (pong) will be send directly after
	//        the tcdIn of the current cycle(ping). Therefore the (ping) tcdIn links to the tcdOut of
	// 		  the opposed ping-pong buffer (pong). The (pong)tcdOut links to the tcdIn of the (pong) buffer 
	//        as the tcdIn of the (ping) buffer will start in the next cycle.

	for(uint32_t pingPong = 0; pingPong < 2; pingPong++)
	{
		// Link tcds for data in
		uint32_t nextTcd = 0;
		uint32_t i = hInTcdhandles.size();
		while(i--)
		{
			EDMA_TcdEnableAutoStopRequest(&hInTcdhandles[i].tcd[pingPong], false);
			hInTcdhandles[i].tcd[pingPong].DLAST_SGA = nextTcd;
			nextTcd = (uint32_t)&hInTcdhandles[i].tcd[pingPong];
		}

		// Link tcds for data out
		nextTcd = 0;
		uint32_t handleIndex = hOutTcdhandles.size();
		while(handleIndex--)
		{
			// Link the tcds for the 3 phases
			for(int32_t phaseIndex = 2; phaseIndex >= 0; phaseIndex--)
			{
				hOutTcdhandles[handleIndex].tcds[pingPong][phaseIndex].DLAST_SGA = nextTcd;
				nextTcd = (uint32_t)&hOutTcdhandles[handleIndex].tcds[pingPong][phaseIndex];
				EDMA_TcdEnableAutoStopRequest(&hOutTcdhandles[handleIndex].tcds[pingPong][phaseIndex], false);
			}
			// Update the command addresses to the commands
			hOutTcdhandles[handleIndex].tcds[pingPong][0].SADDR = (uint32_t)&hOutTcdhandles[handleIndex].command[0];
			hOutTcdhandles[handleIndex].tcds[pingPong][2].SADDR = (uint32_t)&hOutTcdhandles[handleIndex].command[2];

			// Disable interrupts that may have been enabled for a prior last tcd
			hOutTcdhandles[handleIndex].tcds[pingPong][2].CSR &= ~DMA_CSR_INTMAJOR_MASK;
		}
	}

	// now all but the last tcds are linked together, as per Note2 
	//  there is special care needed for the last tcds, as we need cross ping-pong buffer linking
	for(uint32_t pingPong = 0; pingPong < 2; pingPong++)
	{
		// Link last tcd in to opposing tcdOut
		edma_tcd_t* lastTcdIn = &hInTcdhandles[hInTcdhandles.size() - 1].tcd[pingPong];
		edma_tcd_t* lastTcdOutOpposed = &hOutTcdhandles[hOutTcdhandles.size() - 1].tcds[pingPong ^ 1][2];
		edma_tcd_t* firstTcdIn = &hInTcdhandles[0].tcd[pingPong];
		edma_tcd_t* firstTcdOutOpposed = &hOutTcdhandles[0].tcds[pingPong ^ 1][0];

		// When there is no putput data, the last tcd in points to the first tcd in
		if(hOutTcdhandles.size() == 0)
		{
			lastTcdIn->DLAST_SGA = (uint32_t)firstTcdIn;
			continue;
		}

		lastTcdIn->DLAST_SGA = (uint32_t)firstTcdOutOpposed;
		lastTcdOutOpposed->DLAST_SGA = (uint32_t)firstTcdIn;
		lastTcdOutOpposed->CSR |= DMA_CSR_INTMAJOR(1);

		// Enable auto stop request for direction changes
		EDMA_TcdEnableAutoStopRequest(lastTcdIn, true);
		EDMA_TcdEnableAutoStopRequest(lastTcdOutOpposed, true);
	}

	// Install the first tcd in the dma channel
	EDMA_InstallTCD(dmaBase, dmaChannel, &hInTcdhandles[0].tcd[0]);

	return Status::Ok;
} 

/**
 * @brief Sends a command to the i2c hardware, currently commands longer than the fifo depth 
 * 			are supported but busy wait until there is space in the fifo
 * 
 * @param command 	Pointer to the command that should be send
 */
void PeripheralHandler::sendCommand(std::span<const uint16_t> command) {
	// TODO: Write a send command that can handle commands longer than the fifo

	// Write the command into the fifo as long as there is space
	for(auto& cmd : command)
	{
		// Wait for space in the fifo
		while(((i2cBase->MFSR & LPI2C_MFSR_TXCOUNT_MASK) >> LPI2C_MFSR_TXCOUNT_SHIFT) == I2C_FIFO_DEPTH)
		{
			__NOP();
		}
		i2cBase->MTDR = cmd;
	}

}

/**
 * @brief Checks if the address is available
 * 
 * @param address 	Address that should be checked
 * @return Status::Ok if the address is available
 * 		   Status::Error if the address is not available
 */
Status PeripheralHandler::addrAvailable(uint32_t address) {
	return Status::Ok;
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


} /* namespace freesthetics */
