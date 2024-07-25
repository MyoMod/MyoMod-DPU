/*
 * RuntimeHandler.cpp
 *
 *  Created on: Jan 9, 2024
 *      Author: leon
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "RuntimeHandler.h"

#include <vector>
#include <string>
#include <span>
#include <array>
#include <memory>

#include "board.h"
#include "pin_mux.h"
#include "fsl_pit.h"
#include "fsl_gpio.h"
#include "fsl_common.h"
#include "fsl_clock.h"
#include "SEGGER_RTT.h"

#include "ConfigurationManager.h"
#include "Configuration.h"
#include "PeripheralHandler.h"
#include "Status.h"
#include "dpu_gpio.h"


/*******************************************************************************
 * Defines
 ******************************************************************************/

/*******************************************************************************
 * Data Types
 * ****************************************************************************/

/*******************************************************************************
 * Private Variables
 * ****************************************************************************/

ConfigurationManager* g_configManager = nullptr;
std::array<PeripheralHandler*, 3> g_peripheralHandlers;
std::vector<std::unique_ptr<DeviceNode>> g_deviceNodes;
std::vector<std::unique_ptr<AlgorithmicNode>> g_algorithmicNodes;

volatile bool g_isRunning = false;

uint8_t g_debugBuffer[1024];
/*******************************************************************************
 * Function Prototypes
 * ****************************************************************************/
void renumrateDevices();
void updateConfiguration();
void start();
void stop();
void inputHandlingDone();
void startCycle();

// init
void initHardware();
void initPerpheralHandler();
void gpio_init();
void init_debug();
/*******************************************************************************
 * Functions
 * ****************************************************************************/
int main()
{
	/** Initialization **/
	initHardware();
	initPerpheralHandler();
	init_debug();

	RGB_B(1);

	/** fill globals **/
	g_configManager = new ConfigurationManager();
	assert(g_configManager->readConfigurations() == Status::Ok);

	/** wait 500 ms for the devices to start up **/
	SDK_DelayAtLeastUs(500000, CLOCK_GetFreq(kCLOCK_CoreSysClk));

	/** Initialzation done -> Enumerate the devices **/
	renumrateDevices();

	updateConfiguration();

	//Main Loop
	while(1)
	{
		if(!g_isRunning)
		{
			SEGGER_RTT_printf(0, "runtime is not running, this shouldn't happen\n");
		}
	}
	return 0;
}


void renumrateDevices()
{
	//Stop the cycle
	stop();

	// Update devices
	std::vector<DeviceIdentifier> foundDevices;
	for (auto& peripheralHandler : g_peripheralHandlers)
	{
		Status status;
		auto devices = peripheralHandler->listConnectedDevices(status);
		foundDevices.insert(foundDevices.end(), devices.begin(), devices.end());
		assert(status != Status::Error);
	}

	// Update the valid configurations
	if(g_configManager->updateValidConfigurations(foundDevices) == Status::Ok)
	{
		// Start the cycle if the current configuration still is valid
		start();
	};	
}

/**
 * @brief Updates the configuration if needed
 * 
 */
void updateConfiguration()
{
	// Stop the cycle
	stop();

	if ((!g_configManager->isValid()) || 
		(g_configManager->getActiveConfigurationIndex() == 0 && g_configManager->getNumberOfValidConfigurations() > 0))
	{
		// Active configuration is not valid anymore or the NOP configuration is active and there are valid configurations
		// -> Select a valid, non-trivial configuration
		Status status = g_configManager->incrementActiveConfiguration();
		assert(status != Status::Error);
		
		// Get the active configuration
		auto nodes = g_configManager->createActiveConfiguration();
		g_deviceNodes = std::move(std::get<0>(nodes));
		g_algorithmicNodes = std::move(std::get<1>(nodes));

		// remove all old devices from the peripheral handlers
		for (auto& peripheralHandler : g_peripheralHandlers)
		{
			peripheralHandler->uninstallAllDevices();
		}

		// Add the new devices to the peripheral handlers
		for (auto&& deviceNode : g_deviceNodes)
		{
			auto deviceIdentifier = deviceNode->getDeviceIdentifier();
			for (auto& peripheralHandler : g_peripheralHandlers)
			{
				if(peripheralHandler->getDeviceAdress(deviceIdentifier) != -1)
				{
					// Device is connected to this peripheral handler
					// -> Install the device and don't try to install it in another peripheral handler

					status = peripheralHandler->installDevice(deviceNode.get());
					assert(status != Status::Error);
				}
				
			}
		}
	}

	// Start the cycle
	start();
}

/**
 * @brief Starts the realtime processing cycle
 * 
 */
void start()
{
	if (g_isRunning)
	{
		// System is already running
		// -> Nothing to do
		return;
	}

	SEGGER_RTT_printf(0, "Start realtime processing\n");

	// Start peripheral handlers
	Status status;
	for (auto& peripheralHandler : g_peripheralHandlers)
	{
		status = peripheralHandler->enterRealTimeMode();
		assert(status == Status::Ok);
	}

	// Set the running flag
	g_isRunning = true;
}

/**
 * @brief Stops the realtime processing cycle
 * 
 */
void stop()
{
	if (!g_isRunning)
	{
		// System is not running
		// -> Nothing to do
		return;
	}

	SEGGER_RTT_printf(0, "Stop realtime processing\n");

	// Stop the peripheral handlers
	Status status;
	for (auto& peripheralHandler : g_peripheralHandlers)
	{
		status = peripheralHandler->exitRealTimeMode();
		assert(status == Status::Ok);
	}

	// Clear the running flag
	g_isRunning = false;
}

void peripheralHandlerCallback(uint32_t index)
{
	static std::array<bool, 3> dataAvailable = {false, false, false};

	dataAvailable[index] = true;

	// Check if all peripherals have data available
	bool allDataAvailable = true;
	for (size_t i = 0; i < dataAvailable.size(); i++)
	{
		allDataAvailable &= (dataAvailable[i] || !g_peripheralHandlers[i]->hasInstalledDevices());
	}

	if (allDataAvailable)
	{
		if(g_isRunning)
		{
			// All peripherals have data available and the system is running
			// -> Start the input handling
			inputHandlingDone();
		}
		dataAvailable = {false, false, false};
	}
}

/**
 * @brief Callback that is called when the input handling of the ComInterface is done
 * 			and the data should be processed
 * 
 */
void inputHandlingDone()
{
	RGB_B(1);

	for (auto&& deviceNode : g_deviceNodes)
	{
		deviceNode->processInData();
	}

	DEBUG_PINS(1);
	for (auto&& algorithmicNode : g_algorithmicNodes)
	{
		algorithmicNode->process();
	}
		DEBUG_PINS(0);

	for (auto&& deviceNode : g_deviceNodes)
	{
		deviceNode->processOutData();
	}

	RGB_B(0);
}

/*******************************************************************************
 * Interrupt Handlers
 * ****************************************************************************/
extern "C"
{
void PIT_IRQHandler(void)
{
	/* Clear interrupt flag.*/
	PIT_ClearStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0, kPIT_TimerFlag);

	static int i;
    i++;
    GPIO_PinWrite(BOARD_INITPINS_Sync_GPIO, BOARD_INITPINS_Sync_GPIO_PIN, i & 1);


    // start i2c transfer after 100 cycles
    if (i & 1 && i > 100 && g_isRunning)
	{
		startCycle();
	}
}
}

/**
 * @brief Starts a new cycle
 * 
 */
void startCycle()
{
	// check if devices have changed
	for (auto& peripheralHandler : g_peripheralHandlers)
	{
		if (peripheralHandler->connectedDevicesChanged())
		{
			// Devices have changed
			// -> Reenumerate the devices
			renumrateDevices();
			updateConfiguration();
			return;
		}
	}

	// Send the sync signal
	Status status;
	for (auto& peripheralHandler : g_peripheralHandlers)
	{
		status = peripheralHandler->sendSync();
		if (status == Status::Error)
		{
			// Error sending the sync signal
			// -> Stop the cycle
			stop();
			SEGGER_RTT_printf(0, "Error sending sync signal\n");
			return;
		}
	}

	// Start the cycles
	for (auto& peripheralHandler : g_peripheralHandlers)
	{
		if (!peripheralHandler->hasInstalledDevices())
		{
			// No devices connected to this peripheral handler
			// -> Skip this peripheral handler
			continue;
		}
		status = peripheralHandler->startCycle();
		if (status != Status::Ok)
		{
			// Error starting the cycle
			// -> Stop the cycle
			stop();
			SEGGER_RTT_printf(0, "Error starting cycle\n");
			return;
		}
	}
}

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/


void initHardware()
{
    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    // Set frequency to 200Hz
    PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, USEC_TO_COUNT(5000, CLOCK_GetFreq(kCLOCK_PerClk)));

    gpio_init();
}

void initPerpheralHandler()
{
	g_peripheralHandlers[0] = new PeripheralHandler(DMA0, 1, [](){peripheralHandlerCallback(0);}, true);
	g_peripheralHandlers[1] = new PeripheralHandler(DMA0, 3, [](){peripheralHandlerCallback(1);}, true);
	g_peripheralHandlers[2] = new PeripheralHandler(DMA0, 4, [](){peripheralHandlerCallback(2);}, false);
}


void gpio_init()
{
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        0,
    };

    /* Init output LED GPIO. */
    GPIO_PinInit(BOARD_INITPINS_USR_LED_GPIO, BOARD_INITPINS_USR_LED_GPIO_PIN, &led_config);
    GPIO_PinInit(BOARD_INITPINS_Sync_GPIO, BOARD_INITPINS_Sync_GPIO_PIN, &led_config);

	GPIO_PinWrite(BOARD_INITPINS_USR_LED_GPIO, BOARD_INITPINS_USR_LED_GPIO_PIN, 1);
}

void init_debug()
{

    SEGGER_RTT_Init();
    SEGGER_RTT_ConfigUpBuffer(1, "DataOut", &g_debugBuffer[0], sizeof(g_debugBuffer),
                              SEGGER_RTT_MODE_NO_BLOCK_SKIP);

    SEGGER_RTT_WriteString(0, "MyoMod DPU\r\n\r\n");
    SEGGER_RTT_WriteString(0, BOARD_NAME);
	SEGGER_RTT_WriteString(0, "\r\n\r\n");

}
