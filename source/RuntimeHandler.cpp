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

#include "ComInterface.h"
#include "ConfigurationManager.h"
#include "Configuration.h"
#include "AnalysisAlgorithm.h"
#include "PassAlgorithm.h"
#include "FftAlgorithm.h"
#include "Status.h"


/*******************************************************************************
 * Defines
 ******************************************************************************/
#define USE_ELECTRODE 1
#define USE_SDCARD 0
#define USE_DISPLAY 1
#define USE_BTSINK 1

/*******************************************************************************
 * Data Types
 * ****************************************************************************/

/*******************************************************************************
 * Private Variables
 * ****************************************************************************/
freesthetics::ComInterface* comInterface = nullptr;
freesthetics::ConfigurationManager* configManager = nullptr;
freesthetics::AnalysisAlgorithm* activeAnalysisAlgorithm = nullptr;

volatile bool isRunning = false;

uint8_t g_debugBuffer[32];
/*******************************************************************************
 * Function Prototypes
 * ****************************************************************************/
void renumrateDevices();
void start();
void stop();
void inputHandlingDone();

// init
void initHardware();
void gpio_init();
void init_debug();
/*******************************************************************************
 * Functions
 * ****************************************************************************/
void main()
{
	freesthetics::Status status;

	/** Initialization **/
	initHardware();
	init_debug();

	/** fill globals **/
	comInterface = new freesthetics::ComInterface(inputHandlingDone);
	configManager = new freesthetics::ConfigurationManager(comInterface);
	activeAnalysisAlgorithm = configManager->getActiveConfiguration().algorithm;

	// Add a configuration
	freesthetics::AnalysisAlgorithm* fftAlgorithm = new freesthetics::FFTAlgorithm("FFT Algorithm");
	freesthetics::Configuration fftConfiguration {
		"FFT Filter Configuration",
		fftAlgorithm
	};
	// Devices
	freesthetics::DeviceDescriptor electrode {
		.deviceType = "Elctr6Ch",
		.deviceIdentifier = "Elctrode1",
		.peripheralIndex = -1,
		.deviceAddress = -1,
		.name = "Electrode"
	};
	freesthetics::DeviceDescriptor sdCard {
		.deviceType = "Elctr6Ch",
		.deviceIdentifier = "SDSource1",
		.peripheralIndex = -1,
		.deviceAddress = -1,
		.name = "SD Card"
	};
	freesthetics::DeviceDescriptor display {
		.deviceType = "BarDis7Ch",
		.deviceIdentifier = "BDisplay2",
		.peripheralIndex = -1,
		.deviceAddress = -1,
		.name = "Display"
	};
	freesthetics::DeviceDescriptor btSink {
		.deviceType = "BtSink6Ch",
		.deviceIdentifier = "Blt_Sink2",
		.peripheralIndex = -1,
		.deviceAddress = -1,
		.name = "Bt Sink"
	};	

	uint32_t pdsIndex = 0;
#if USE_ELECTRODE
	fftConfiguration.PDSs.push_back({
		.name = "Electrode",
		.channels = {},
		.isInput = true
	});
	const uint32_t electrodePdsIndex = pdsIndex++;
#endif
#if USE_SDCARD
	fftConfiguration.PDSs.push_back({
		.name = "SDCard",
		.channels = {},
		.isInput = true
	});
	const uint32_t sdCardPdsIndex = pdsIndex++;
#endif
#if USE_DISPLAY
	fftConfiguration.PDSs.push_back({
		.name = "Display",
		.channels = {},
		.isInput = false
	});
	const uint32_t displayPdsIndex = pdsIndex++;
	fftConfiguration.PDSs.push_back({
		.name = "DisplayButtons",
		.channels = {},
		.isInput = true
	});
	const uint32_t displayButtonsPdsIndex = pdsIndex++;
#endif
#if USE_BTSINK
	fftConfiguration.PDSs.push_back({
		.name = "BtSink",
		.channels = {},
		.isInput = false
	});
	const uint32_t btSinkPdsIndex = pdsIndex++;
#endif

	// Add channels
	for(uint32_t i = 0; i < 6; i++)
	{
		#if USE_ELECTRODE
		freesthetics::ChannelDescriptor electrodeChannel {
			.device = &electrode,
			.channelIndex = i,
			.name = "Channel " + std::to_string(i + 1)
		};
		fftConfiguration.PDSs[electrodePdsIndex].channels.push_back(electrodeChannel);
		#endif
		#if USE_SDCARD
		freesthetics::ChannelDescriptor sdChannel {
			.device = &sdCard,
			.channelIndex = i,
			.name = "Channel " + std::to_string(i + 1)
		};
		fftConfiguration.PDSs[sdCardPdsIndex].channels.push_back(sdChannel);
		#endif
		#if USE_DISPLAY
		freesthetics::ChannelDescriptor displayChannel {
			.device = &display,
			.channelIndex = i,
			.name = "Channel " + std::to_string(i + 1)
		};
		fftConfiguration.PDSs[displayPdsIndex].channels.push_back(displayChannel);
		#endif
		#if USE_BTSINK
		freesthetics::ChannelDescriptor btChannel {
			.device = &btSink,
			.channelIndex = i,
			.name = "Channel " + std::to_string(i + 1)
		};
		fftConfiguration.PDSs[btSinkPdsIndex].channels.push_back(btChannel);
		#endif
	}
	#if USE_DISPLAY
	// add 7nth Channel
	freesthetics::ChannelDescriptor displayChannel {
		.device = &display,
		.channelIndex = 6,
		.name = "Channel 7"
	};
	fftConfiguration.PDSs[displayPdsIndex].channels.push_back(displayChannel);

	// add buttons
	std::string buttonNames[4] = {"A", "B", "X", "Y"};
	for (size_t i = 0; i < 4; i++)
	{
		freesthetics::ChannelDescriptor buttonChannel {
			.device = &display,
			.channelIndex = i,
			.name = buttonNames[i]
		};
		fftConfiguration.PDSs[displayButtonsPdsIndex].channels.push_back(buttonChannel);
	}
	#endif
	
	configManager->addConfiguration(fftConfiguration);

	/** Initialzation done -> Enumerate the devices **/
	renumrateDevices();

	//Main Loop
	while(1)
	{
		if(!isRunning)
		{
			// Check if the configuration is valid
			status = comInterface->configurationValid();
			if(status != freesthetics::Status::Ok)
			{
				// Configuration is not valid, stop the cycle and setup new configuration
				renumrateDevices();
			}
			else
			{
				// Start the cycle
				start();
			}
		}
	}
}


void renumrateDevices()
{
	//Stop the cycle
	stop();

	// Renumerate the devices
	freesthetics::Status status = configManager->renumrateDevices();
	assert(status == freesthetics::Status::Ok);

	status = configManager->incrementActiveConfiguration();
	assert(status == freesthetics::Status::Ok);

	// Update the active configuration
	static freesthetics::Configuration activeConfiguration = configManager->getActiveConfiguration();

	// Update the analysis algorithm
	activeAnalysisAlgorithm = activeConfiguration.algorithm;
	activeAnalysisAlgorithm->setComInterface(comInterface);

	// Start the cycle
	start();	
}

/**
 * @brief Starts the realtime processing cycle
 * 
 */
void start()
{
	SEGGER_RTT_printf(0, "Start realtime processing\n");

	// Start the ComInterface
	freesthetics::Status status = comInterface->enterRealTimeMode();
	assert(status == freesthetics::Status::Ok);

	// Start the analysis algorithm
	activeAnalysisAlgorithm->start();

	// Set the running flag
	isRunning = true;
}

/**
 * @brief Stops the realtime processing cycle
 * 
 */
void stop()
{
	SEGGER_RTT_printf(0, "Stop realtime processing\n");

	// Stop the analysis algorithm
	activeAnalysisAlgorithm->stop();

	// Stop the ComInterface
	freesthetics::Status status = comInterface->exitRealTimeMode();
	assert(status == freesthetics::Status::Ok);

	// Clear the running flag
	isRunning = false;
}

/**
 * @brief Callback that is called when the input handling of the ComInterface is done
 * 			and the data should be processed
 * 
 */
void inputHandlingDone()
{
	//Check if the configuration is valid
	freesthetics::Status status = comInterface->configurationValid();
	if(status != freesthetics::Status::Ok)
	{
		// Stop realtime processing
		stop();

		// Configuration is not valid, stop the cycle and setup new configuration
		renumrateDevices();

		return;
	}

	activeAnalysisAlgorithm->run();

	status = comInterface->processOutgoingData();
	assert(status == freesthetics::Status::Ok);
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
    if (i & 1 && i > 100 && isRunning)
	{
		comInterface->CyclicHandler();
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
}

void init_debug()
{

    SEGGER_RTT_Init();
    SEGGER_RTT_ConfigUpBuffer(1, "DataOut", &g_debugBuffer[0], sizeof(g_debugBuffer),
                              SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

    SEGGER_RTT_WriteString(0, "SEGGER Real-Time-Terminal Sample\r\n\r\n");
    SEGGER_RTT_WriteString(0, BOARD_NAME);
	SEGGER_RTT_WriteString(0, "\r\n\r\n");

}
