/*
 * PeripheralHandler.h
 *
 *  Created on: Jan 9, 2024
 *      Author: leon
 */

#ifndef PERIPHERALHANDLER_H_
#define PERIPHERALHANDLER_H_
#include "stdint.h"
#include <vector>
#include <array>
#include <span>
#include <map>
#include <functional>
#include "peripherals.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MIMXRT1062.h"
#include "MIMXRT1062_features.h"
#include "fsl_common.h"
#include "fsl_lpi2c.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "SEGGER_RTT.h"

#include "Status.h"
#include "MemoryRegion.h"
#include "Configuration.h"

namespace freesthetics {
#define DMA_CHANNELS 32
#define I2C_UNITS 4

enum class CommState
{
	Stopped,
	Idle,
	Receiving,
	Transmitting,
	Configuring
};

struct TcdInHandle
{
	std::array<edma_tcd_t,2> tcd __ALIGNED(32); //ping-pong buffer
	std::vector<uint16_t> command; // gets longer than 3 elements when datalen > 255 Byte
	uint32_t deviceAddress;
};

struct TcdOutHandle
{
	//ping-pong buffer that consits of 3 phases (command, data, stop)
	std::array<std::array<edma_tcd_t, 3>, 2> tcds __ALIGNED(32);
	std::array<uint16_t, 3> command;
	uint32_t deviceAddress;
};

/*
 *
 */
class PeripheralHandler {
public:
	PeripheralHandler(DMA_Type* dma, uint32_t i2cIndex, std::function<void(void)> processDataCallback, bool highSpeed);
	virtual ~PeripheralHandler();

	Status listNewDevices(std::vector<DeviceDescriptor>* devices);
	bool detectedNewDevices();

	Status addDevice(uint32_t address, MemoryRegion dataInBuffer, std::array<MemoryRegion,2> dataOutBuffer);
	Status removeDevice(uint32_t address);
	uint32_t getPingPongIndex();

	Status sendSync();
	Status startCycle();

	Status enterRealTimeMode();
	Status exitRealTimeMode();

	void dmaInterruptHandler();
	void i2cInterruptHandler();

	bool hasDevices() { return !hInTcdhandles.empty(); }

	static void DMA4_DMA20_IRQHandler();
	static void DMA5_DMA21_IRQHandler();
	static void DMA6_DMA22_IRQHandler();
	static void DMA7_DMA23_IRQHandler();
	static void LPI2C1_IRQHandler();
	static void LPI2C2_IRQHandler();
	static void LPI2C3_IRQHandler();
	static void LPI2C4_IRQHandler();
	
	static std::array<PeripheralHandler*, I2C_UNITS> handlers;
private:
	Status appendTcdIn(MemoryRegion data, uint32_t deviceAddress) ;
	void fillOutTcds(edma_tcd_t* tcd, uint8_t* data, uint32_t length, uint32_t size = 0);
	Status appendTcdOut(std::array<MemoryRegion,2> data, uint32_t deviceAddress);
	Status linkTcds();

	void sendCommand(std::span<const uint16_t> command);

	Status addrAvailable(uint32_t address);

	std::vector<TcdOutHandle> hOutTcdhandles;
	std::vector<TcdInHandle> hInTcdhandles;
	std::map<uint32_t, uint8_t> cyclesSineLastSeen;

	DMA_Type* dmaBase;
	LPI2C_Type* i2cBase;

	CommState commState;
	uint32_t dmaChannel;
	uint32_t tcdIndex;
	uint32_t pingPongIndex;
	bool devicesChanged;

	std::function<void(void)> processDataCallback;
	//void (*processDataCallback)(uint8_t callbackParam, bool pingPongIndex);

};



} /* namespace freesthetics */

#endif /* PERIPHERALHANDLER_H_ */
