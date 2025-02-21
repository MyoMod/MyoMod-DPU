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
#include "math.h"

#include "board.h"
#include "pin_mux.h"
#include "fsl_pit.h"
#include "fsl_gpio.h"
#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_qtmr.h"
#include "fsl_src.h"
#include "fsl_pwm.h"
#include "fsl_adc.h"
#include "fsl_romapi.h"
#include "fsl_cache.h"

#include "SEGGER_RTT.h"
#include "etl/queue_spsc_atomic.h"

#include "myomodCommon.h"
#include "ConfigurationManager.h"
#include "Configuration.h"
#include "PeripheralHandler.h"
#include "embeddedDevicesManager.h"
#include "Status.h"
#include "dpu_gpio.h"

#include "ui/button.h"
#include "ICM42670P.h"
#include "max11254.h"

#include "embeddedIMU.h"

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
std::array<PeripheralHandler*, 2> g_peripheralHandlers;
EmbeddedDevicesManager g_embeddedDevicesManager;
std::vector<std::unique_ptr<DeviceNode>> g_deviceNodes;
std::vector<std::shared_ptr<EmbeddedDeviceNode>> g_embeddedDeviceNodes;
std::vector<std::unique_ptr<AlgorithmicNode>> g_algorithmicNodes;

volatile bool g_isRunning = false;
// volatile float g_vBat = 0;
// volatile uint32_t g_vBatRaw = 0;

// static volatile int32_t accelRaw[3] = {0,0,0};
// volatile int32_t adcRaw[6] = {0,0,0,0,0,0};
// // volatile uint64_t g_time;
// etl::queue_spsc_atomic<std::array<int32_t, 6>, 20> g_adcQueue;
// volatile uint32_t g_adcCounter = 0;

//ICM42670 g_imu = ICM42670(SPI_IMU_PERIPHERAL);
//MAX11254 *g_max11254 = nullptr;

int myvar[1] __attribute__((section(".mySection"))) = {0};

uint8_t g_debugBuffer[1024];
/*******************************************************************************
 * Function Prototypes
 * ****************************************************************************/
void renumrateDevices();
void updateConfiguration();
void incrementConfiguration();
void decrementConfiguration();
void installActiveConfiguration();
void start();
void stop();
void handleNodes();
void startCycle();
//void newAdcData(std::array<int32_t, MAX11254_NUM_CHANNELS> &measurements, bool clipped, bool rangeExceeded, bool error);

// init
void initHardware();
void initPeripheralHandler();
void initEmbeddedDevices();
void gpio_init();
void init_debug();
void testFlashAccess();
bool writeConfiguration(uint32_t offset, uint32_t* data, uint32_t length);

/*******************************************************************************
 * Functions
 * ****************************************************************************/
int main()
{
	/** Initialization **/

	initHardware();
	// uint64_t lastTime = time_us_64();
	// float ledVal = 50;
	
	// while(0)
	// {
	// 	uint64_t now = time_us_64();
	// 	if(now - lastTime > 10000)
	// 	{
	// 		lastTime = now;
	// 		// 1ms passed
	// 		g_time = now;

	// 		GPIO_PinWrite(DEBUG_DEBUG2_GPIO, DEBUG_DEBUG2_PIN, 1);
	// 		// Get latest imu data
	// 		inv_imu_sensor_event_t imu_event;
	// 		g_imu.getDataFromRegisters(imu_event);
	// 		GPIO_PinWrite(DEBUG_DEBUG2_GPIO, DEBUG_DEBUG2_PIN, 0);

	// 		ledVal += 0.005f;

	// 		accelRaw[0] = imu_event.accel[0];
	// 		accelRaw[1] = imu_event.accel[1];
	// 		accelRaw[2] = imu_event.accel[2];

	// 		// 1g at about 2048
	// 		float colors[3] = {0,0,0};
	// 		for (int i = 0; i < 3; i++)
	// 		{
	// 			//scale from -2048 to 2048 to -100 to 100
	// 			colors[i] = (float)accelRaw[i] / 2048.0f *100.0f;
				 
	// 			//abs and limit to 100
	// 			colors[i] = fminf(100.0f, fabsf(colors[i]));
	// 		}

			
	// 		PWM_UpdatePwmDutycycle(PWM1, PWM1_SM1, PWM1_SM1_LED_G, kPWM_EdgeAligned, colors[0]);
	// 		PWM_UpdatePwmDutycycle(PWM1, PWM1_SM1, PWM1_SM1_LED_B, kPWM_EdgeAligned, colors[1]);
	// 		PWM_UpdatePwmDutycycle(PWM1, PWM1_SM2, PWM1_SM2_LED_R, kPWM_EdgeAligned, colors[2]);
	// 		PWM_SetPwmLdok(PWM1, 2 | 4, true);

			
	// 		//retrieve adc data
	// 		std::array<std::array<int32_t, 6>, 15> adcData;
	// 		for (size_t i = 0; i < 15; i++)
	// 		{
	// 			if(!g_adcQueue.pop(adcData[i]))
	// 			{
	// 				// queue is empty
	// 				break;
	// 			}
	// 		}
	// 		g_adcCounter = g_adcQueue.size();

	// 		// read battery voltage
	// 		const float a = 0.13872f;
	// 		const float b = 0.01362;
	// 		g_vBatRaw = ADC_GetChannelConversionValue(ADC1, 0);
	// 		g_vBat = (float)g_vBatRaw * b + a;
	// 	}
	// }

	initPeripheralHandler();
	initEmbeddedDevices();
	init_debug();

	//Button<1> button(BOARD_INITPINS_SW_UP_PORT, BOARD_INITPINS_SW_UP_PIN, true);

	/** fill globals **/
	g_configManager = new ConfigurationManager();
	assert(g_configManager->readConfigurations() == Status::Ok);

	/** wait 500 ms for the devices to start up **/
	SDK_DelayAtLeastUs(500000, CLOCK_GetFreq(kCLOCK_CoreSysClk));

	/** Initialzation done -> Enumerate the devices **/
	renumrateDevices();

	updateConfiguration();

	// uint32_t currentTime = QTMR_GetCurrentTimerCount(TMR1_PERIPHERAL, TMR1_MS_COUNTER_CHANNEL);
	// uint32_t ledVal = 50;
	// bool direction = true;	

	//Main Loop
	while(1)
	{


		if(!g_isRunning)
		{
			SEGGER_RTT_printf(0, "runtime is not running, this shouldn't happen\n");
		}

		// uint16_t tmrVal = QTMR_GetCurrentTimerCount(TMR1_PERIPHERAL, TMR1_MS_COUNTER_CHANNEL);
		// if((currentTime & 0xFFFF) != tmrVal)
		// {
		// 	// 1ms passed
		// 	if(tmrVal < (currentTime & 0xFFFF))
		// 	{
		// 		// timer overflow -> increment the high word
		// 		currentTime += 0x10000;
		// 	}
		// 	currentTime = (currentTime & 0xFFFF0000) | tmrVal;


		// 	ledVal += direction? 1 : -1;
		// 	if (ledVal == 0 || ledVal == 100)
		// 	{
		// 		direction = !direction;
		// 	}
		// 	PWM_UpdatePwmDutycycle(PWM1_PERIPHERAL, PWM1_SM2, PWM1_SM2_LED_R, kPWM_SignedCenterAligned, ledVal);


		// 	// if(button_up.update())
		// 	// {
		// 	// 	if(!button_up.isSet())
		// 	// 	{
		// 	// 		incrementConfiguration();
		// 	// 	}
		// 	// }
		// 	// if(button_down.update())
		// 	// {
		// 	// 	if(!button_down.isSet())
		// 	// 	{
		// 	// 		decrementConfiguration();
		// 	// 	}
		// 	// }
		// }

	}
	return 0;
}


void renumrateDevices()
{
	//Stop the cycle
	stop();

	SEGGER_RTT_printf(0, "Renumerate devices\n");

	// Update devices
	std::vector<DeviceIdentifier> foundDevices;
	for (auto& peripheralHandler : g_peripheralHandlers)
	{
		Status status;
		auto devices = peripheralHandler->listConnectedDevices(status);
		foundDevices.insert(foundDevices.end(), devices.begin(), devices.end());
		assert(status != Status::Error);
	}

	// Add the embedded devices
	auto embeddedDevices = g_embeddedDevicesManager.listConnectedDevices();
	foundDevices.insert(foundDevices.end(), embeddedDevices.begin(), embeddedDevices.end());

	// print found devices
	SEGGER_RTT_printf(0, "Found device(s): \"");
	for (size_t i = 0; i < foundDevices.size(); i++)
	{
		if (i != 0)
		{
			SEGGER_RTT_printf(0, ", ");
		}
		foundDevices[i].print();
	}
	SEGGER_RTT_printf(0, "\"\n");

	// Update the valid configurations
	g_configManager->updateValidConfigurations(foundDevices);
}

/**
 * @brief Updates the active configuration if necessary
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

		SEGGER_RTT_printf(0, "Update configuration to %s\n", g_configManager->getActiveConfigurationName().c_str());
		installActiveConfiguration();
	}
	else
	{
		// Active configuration is still valid
		// -> Nothing to do
		SEGGER_RTT_printf(0, "Keep active Configuration \"%s\".\n", g_configManager->getActiveConfigurationName().c_str());
	}

	// Start the cycle
	start();
}

/**
 * @brief Increments the active configuration
 * 
 */
void incrementConfiguration()
{
	// Stop the cycle
	stop();

	// Increment the active configuration
	Status status = g_configManager->incrementActiveConfiguration();
	assert(status != Status::Error);

	SEGGER_RTT_printf(0, "Update configuration to %s\n", g_configManager->getActiveConfigurationName().c_str());

	installActiveConfiguration();

	// Start the cycle
	start();
}

/**
 * @brief Decrements the active configuration
 * 
 */
void decrementConfiguration()
{
	// Stop the cycle
	stop();

	// Decrement the active configuration
	Status status = g_configManager->decrementActiveConfiguration();
	assert(status != Status::Error);

	SEGGER_RTT_printf(0, "Update configuration to %s\n", g_configManager->getActiveConfigurationName().c_str());

	installActiveConfiguration();

	// Start the cycle
	start();
}

void installActiveConfiguration()
{
	Status status;

	// Get the active configuration
	auto nodes = g_configManager->createActiveConfiguration();
	g_deviceNodes = std::move(std::get<0>(nodes));
	auto embeddedDeviceNodes = std::move(std::get<1>(nodes));
	g_algorithmicNodes = std::move(std::get<2>(nodes));

	// remove all old devices from the peripheral handlers
	for (auto& peripheralHandler : g_peripheralHandlers)
	{
		peripheralHandler->uninstallAllDevices();
	}
	g_embeddedDevicesManager.uninstallAllDevices();

	// Add the new devices to the peripheral handlers
	for (auto&& deviceNode : g_deviceNodes)
	{
		auto deviceIdentifier = deviceNode->getDeviceIdentifier();
		// Install the device, if it is connected to a peripheral handler
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

	// Add the new devices to the embedded devices manager
	g_embeddedDevicesManager.installDevices(std::move(embeddedDeviceNodes));

	// Update the color	
	//uint32_t color = g_configManager->getActiveConfigurationColor();
	//uint8_t shortColor = !!(color & 0xFF0000) << 2 | !!(color & 0xFF00) << 1 | !!(color & 0xFF);
	//RGB(shortColor);
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

	g_embeddedDevicesManager.enterRealTimeMode();

	//?: TODO: Start the algorithmic nodes

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

	// TODO: Stop the virtual devices
	g_embeddedDevicesManager.exitRealTimeMode();

	//?: TODO: Stop the algorithmic nodes

	// Clear the running flag
	g_isRunning = false;
}

void peripheralHandlerCallback(uint32_t index)
{
	static std::array<bool, 2> dataAvailable = {false, false};

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
			handleNodes();
		}
		dataAvailable = {false, false};
	}
}

/**
 * @brief Callback that is called when the input handling of the ComInterface is done
 * 			and the data should be processed
 * 
 */
void handleNodes()
{
	for (auto&& deviceNode : g_deviceNodes)
	{
		deviceNode->processInData();
	}
	g_embeddedDevicesManager.processInData();

	DEBUG_PIN_0(1);
	for (auto&& algorithmicNode : g_algorithmicNodes)
	{
		algorithmicNode->process();
	}
	DEBUG_PIN_0(0);

	g_embeddedDevicesManager.processOutData();
	for (auto&& deviceNode : g_deviceNodes)
	{
		deviceNode->processOutData();
	}
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

	static uint32_t cycle_index;
    cycle_index++;
    //GPIO_PinWrite(BOARD_INITPINS_Sync_GPIO, BOARD_INITPINS_Sync_GPIO_PIN, i & 1);


    // start i2c transfer after 100 cycles
    if (cycle_index & 1 && cycle_index > 100 && g_isRunning)
	{
		
		startCycle();
	}
	DEBUG_PIN_2(cycle_index & 1);
}

/* GPIO4_Combined_0_15_IRQn interrupt handler */
void GPIO4_GPIO_COMB_0_15_IRQHANDLER(void) {
  /* Get pins flags */ 
  uint32_t pins_flags = GPIO_GetPinsInterruptFlags(GPIO4); 

  /* Place your interrupt code here */ 
//   if (GPIO_PinRead(BOARD_INITPINS_SW_RESET_GPIO, BOARD_INITPINS_SW_RESET_PIN) == 0)
//   {
// 	  SEGGER_RTT_printf(0, "Reset button pressed\n");
// 	  SRC_DoSoftwareResetARMCore0(SRC);
//   }

  /* Clear ins flags */ 
  GPIO_ClearPinsInterruptFlags(GPIO4, pins_flags); 
}

/* EXTERNAL_CONNECTIONS_GPIO1_14_handle callback function */
// void EXTERNAL_CONNECTIONS_GPIO1_14_callback(void *param) {
//   /* Place your code here */
//   g_max11254->IRQ_handler();
//   g_max11254->async_handler();
// }
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
	bool deviceInstalled = false;
	for (auto& peripheralHandler : g_peripheralHandlers)
	{
		if (!peripheralHandler->hasInstalledDevices())
		{
			// No devices connected to this peripheral handler
			// -> Skip this peripheral handler
			continue;
		}
		deviceInstalled = true;
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

	// Call handleNodes directly if no devices are installed (this may be the case
	//  for systems with embedded devices only), as the peripheral handler callback
	//  will not be called in this case	
	if (!deviceInstalled)
	{
		handleNodes();
	}
}

/*******************************************************************************
 * Initialization Functions
 ******************************************************************************/

void writeRAM(uint32_t* ptr, uint32_t* data, uint32_t length)
{
	for (size_t i = 0; i < length; i++)
	{
		*ptr = data[i];
		ptr++;
	}
}

void readRAM(uint32_t* ptr, uint32_t* data, uint32_t length)
{
	for (size_t i = 0; i < length; i++)
	{
		data[i] = *ptr;
		ptr++;
	}
}

void initAndTestRAM()
{

	//unlock LUT
	FLEXSPI_RAM_PERIPHERAL->LUTKEY = 0x5AF05AF0UL;
	FLEXSPI_RAM_PERIPHERAL->LUTCR = FLEXSPI_LUTCR_UNLOCK_MASK;

	// Write LUT
	for (size_t i = 0; i < 64; i++)
	{
		FLEXSPI_RAM_PERIPHERAL->LUT[i] = FLEXSPI_RAM_LUT[i];
	}

    FLEXSPI_SoftwareReset(FLEXSPI_RAM_PERIPHERAL);

	#define TEST_SIZE 1024
	volatile uint32_t* AHBptr = (uint32_t*)0x70002000;

	// test if the hyper ram is working
	volatile uint32_t outData[TEST_SIZE] = {0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
	volatile uint32_t inData[TEST_SIZE] = {5};



	writeRAM((uint32_t*)AHBptr, (uint32_t*)outData, 1);

	readRAM((uint32_t*)AHBptr, (uint32_t*)inData, 1);


	for (size_t i = 10; i < TEST_SIZE; i+=10)
	{
		writeRAM((uint32_t*)AHBptr, (uint32_t*)outData, i);

		readRAM((uint32_t*)AHBptr, (uint32_t*)inData, i);

		SDK_DelayAtLeastUs(5, CLOCK_GetFreq(kCLOCK_CoreSysClk));
	}

	// flexspi_transfer_t flashXferRead;
	// flashXferRead.deviceAddress = 0;
	// flashXferRead.port = kFLEXSPI_PortA1;
	// flashXferRead.cmdType = kFLEXSPI_Read;
	// flashXferRead.seqIndex = 1;
	// flashXferRead.SeqNumber = 1;
	// flashXferRead.data = (uint32_t*)inData;
	// flashXferRead.dataSize = TEST_SIZE * 4;

	// flexspi_transfer_t flashXferWrite;
	// flashXferWrite.deviceAddress = 0;
	// flashXferWrite.port = kFLEXSPI_PortA1;
	// flashXferWrite.cmdType = kFLEXSPI_Write;
	// flashXferWrite.seqIndex = 0;
	// flashXferWrite.SeqNumber = 1;
	// flashXferWrite.data = (uint32_t*)outData;
	// flashXferWrite.dataSize = TEST_SIZE * 4;

	// FLEXSPI_TransferBlocking(FLEXSPI_RAM_PERIPHERAL, &flashXferWrite);

	// FLEXSPI_TransferBlocking(FLEXSPI_RAM_PERIPHERAL, &flashXferRead);
	// FLEXSPI_TransferBlocking(FLEXSPI_RAM_PERIPHERAL, &flashXferRead);

	bool equal = true;
	for (size_t i = 0; i < TEST_SIZE; i++)
	{
		if(inData[i] != outData[i])
		{
			equal = false;
			break;
		}
	}

	if(!equal)
	{
		SEGGER_RTT_printf(0, "Hyper ram test failed\n");
	}
	else
	{
		SEGGER_RTT_printf(0, "Hyper ram test passed\n");
	} 
}

// void newAdcData(std::array<int32_t, MAX11254_NUM_CHANNELS> &measurements, bool clipped, bool rangeExceeded, bool error)
// {
// 	//SEGGER_RTT_printf(0, "New ADC data: %d\n", measurements[0]);
// 	for (size_t i = 0; i < 6; i++)
// 	{
// 		adcRaw[i] = measurements[i];
// 	}
// 	g_adcQueue.push(measurements);
// 	g_adcCounter = g_adcQueue.size();
// }

void initHardware()
{
    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    // Set frequency to 200Hz
    PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, USEC_TO_COUNT(5000, CLOCK_GetFreq(kCLOCK_PerClk)));

	// Start system timer
	QTMR_StartTimer(TMR1_PERIPHERAL, TMR1_MS_COUNTER0_CHANNEL, kQTMR_CascadeCount);
	QTMR_StartTimer(TMR1_PERIPHERAL, TMR1_MS_COUNTER1_CHANNEL, kQTMR_CascadeCount);
	QTMR_StartTimer(TMR1_PERIPHERAL, TMR1_MS_COUNTER2_CHANNEL, kQTMR_CascadeCount);
	QTMR_StartTimer(TMR1_PERIPHERAL, TMR1_TIMEPRESACLER_CHANNEL, kQTMR_PriSrcRiseEdge);

    gpio_init();	
	LPI2C_MasterEnable(I2C_EXT_PERIPHERAL, true);

	char testString[] = "Hello World!Hello World!Hello World!Hello World!Hello World!Hello World!Hello World!";
	writeConfiguration(0, (uint32_t*)testString, sizeof(testString));
}

void initPeripheralHandler()
{
	g_peripheralHandlers[0] = new PeripheralHandler(DMA0, 1, [](){peripheralHandlerCallback(0);}, true); // MM_EXT
	g_peripheralHandlers[1] = new PeripheralHandler(DMA0, 3, [](){peripheralHandlerCallback(1);}, false); // MM_ESP
}

void initEmbeddedDevices()
{
	g_embeddedDevicesManager.registerDevice(DeviceIdentifier(idArr("Embed' IMU"), idArr("OnboardIMU")));
	g_embeddedDevicesManager.registerDevice(DeviceIdentifier(idArr("Embed' LED"), idArr("OnboardLED")));
	g_embeddedDevicesManager.registerDevice(DeviceIdentifier(idArr("Embed' EMG"), idArr("OnboardEMG")));
}

/* LPUART1 signal event callback function */
void LPUART1_SignalEvent(uint32_t event) {
  /* Send completed (USART may still transmit data) */
  if (event & ARM_USART_EVENT_SEND_COMPLETE) {
    /* Place your code here */
  }
  /* Receive completed */
  if (event & ARM_USART_EVENT_RECEIVE_COMPLETE) {
    /* Place your code here */
  }
  /* Receive data overflow */
  if (event & ARM_USART_EVENT_RX_OVERFLOW) {
    /* Place your code here */
  }
}

/* EXTERNAL_CONNECTIONS_IMU_INT1_handle callback function */
void EXTERNAL_CONNECTIONS_IMU_INT1_callback(void *param) {
  /* Place your code here */
}

/* EXTERNAL_CONNECTIONS_IMU_INT2_handle callback function */
void EXTERNAL_CONNECTIONS_IMU_INT2_callback(void *param) {
  /* Place your code here */
}
extern "C"
{
// Handler for user button
void ONOFF_PRESSED_IRQHANDLER(void) {
	// Clear interrupt flag
	SNVS->LPSR |= SNVS_LPSR_SPOF_MASK;

	SEGGER_RTT_printf(0, "User/ONOFF button pressed\n");
}
}


void gpio_init()
{
    /* Define the init structure for the output LED pin*/
    gpio_pin_config_t debug_config = {
        kGPIO_DigitalOutput,
        0,
    };

    /* Init debug pins. */

	GPIO_PinInit(DEBUG_DEBUG0_GPIO, DEBUG_DEBUG0_PIN, &debug_config);
	GPIO_PinInit(DEBUG_DEBUG1_GPIO, DEBUG_DEBUG1_PIN, &debug_config);
	GPIO_PinInit(DEBUG_DEBUG2_GPIO, DEBUG_DEBUG2_PIN, &debug_config);
	GPIO_PinInit(DEBUG_DEBUG3_GPIO, DEBUG_DEBUG3_PIN, &debug_config);

	GPIO_PinWrite(DEBUG_DEBUG0_GPIO, DEBUG_DEBUG0_PIN, 0);
	GPIO_PinWrite(DEBUG_DEBUG1_GPIO, DEBUG_DEBUG1_PIN, 0);
	GPIO_PinWrite(DEBUG_DEBUG2_GPIO, DEBUG_DEBUG2_PIN, 0);
	GPIO_PinWrite(DEBUG_DEBUG3_GPIO, DEBUG_DEBUG3_PIN, 0);
    // GPIO_PinInit(BOARD_INITPINS_USR_LED_GPIO, BOARD_INITPINS_USR_LED_GPIO_PIN, &led_config);
    // GPIO_PinInit(BOARD_INITPINS_Sync_GPIO, BOARD_INITPINS_Sync_GPIO_PIN, &led_config);

	// GPIO_PinWrite(BOARD_INITPINS_USR_LED_GPIO, BOARD_INITPINS_USR_LED_GPIO_PIN, 1);
	// debug_config.outputLogic = 1;
	// GPIO_PinInit(EXTERNAL_CONNECTIONS_EXT_SDA_GPIO, EXTERNAL_CONNECTIONS_EXT_SDA_PIN, &debug_config);
	// // GPIO_PinInit(EXTERNAL_CONNECTIONS_EXT_SCL_GPIO, EXTERNAL_CONNECTIONS_EXT_SCL_PIN, &debug_config);

	// GPIO_PinWrite(EXTERNAL_CONNECTIONS_EXT_SDA_GPIO, EXTERNAL_CONNECTIONS_EXT_SDA_PIN, 0);
	// SDK_DelayAtLeastUs(10000, CLOCK_GetFreq(kCLOCK_CoreSysClk));
	// GPIO_PinWrite(EXTERNAL_CONNECTIONS_EXT_SDA_GPIO, EXTERNAL_CONNECTIONS_EXT_SDA_PIN, 1);

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

__attribute__((section(".ramfunc.$SRAM_ITC")))
bool writeConfiguration(uint32_t offset, uint32_t* data, uint32_t length)
{
	const uint32_t SECTOR_SIZE = 0x1000; // 4KB
	const uint32_t CONFIG_REGION_START = 15 * 1024 * 1024;
	const uint32_t CONFIG_REGION_SIZE = 1 * 1024 * 1024;

	assert(offset % SECTOR_SIZE == 0);
	assert((offset + length) <= CONFIG_REGION_SIZE);

	// Disable all interrupts, because Flash may not be accessed while it is 
	// cleared or programmed and interrupts may resides in the Flash memory
	//  
	__disable_irq();

	flexspi_nor_config_t config;
	serial_nor_config_option_t option;
	status_t status;
	uint32_t page_buffer[256 / sizeof(uint32_t)];
	uint32_t instance = 0;
	option.option0.U = 0xC0000003; // QuadSPI NOR, Frequency: 133MHz

	uint32_t address = CONFIG_REGION_START + offset;
	uint32_t totalLength = length;
	uint32_t *ptr = data;

	status = ROM_FLEXSPI_NorFlash_GetConfig(instance, &config, &option);
	if (status != kStatus_Success)
	{
		SEGGER_RTT_printf(0, "Error: FLEXSPI_NorFlash_GetConfig failed\n");
	}

	// Without this delay, the init function may freeze
	SDK_DelayAtLeastUs(1000, CLOCK_GetFreq(kCLOCK_CoreSysClk));
	status = ROM_FLEXSPI_NorFlash_Init(instance, &config);
	if (status != kStatus_Success)
	{
		SEGGER_RTT_printf(0, "Error: FLEXSPI_NorFlash_Init failed\n");
	}

	status = ROM_FLEXSPI_NorFlash_Erase(instance, &config, address, length); // Erase 1 sector
	if (status != kStatus_Success)
	{
		SEGGER_RTT_printf(0, "Error: FLEXSPI_NorFlash_Erase failed\n");
	}

	// Program data to destination
	while(length > 0)
	{
		uint32_t bytesToWrite = length>sizeof(page_buffer)? sizeof(page_buffer): length;

		// Fill data into the page_buffer;
		memcpy(page_buffer, ptr, bytesToWrite);

		status = ROM_FLEXSPI_NorFlash_ProgramPage(instance, &config, address, page_buffer); // program 1 page
		if (status != kStatus_Success)
		{
			SEGGER_RTT_printf(0, "Error: FLEXSPI_NorFlash_ProgramPage failed\n");
		}

		address += sizeof(page_buffer);
		length -= bytesToWrite;
		ptr += (bytesToWrite / 4);
	}
	
	// Flush the cache
	DCACHE_CleanInvalidateByRange(CONFIG_REGION_START + offset, totalLength);

	// Use memory mapped access to verify whether data are programmed into Flash correctly
	uint32_t mem_address = 0x60000000 + CONFIG_REGION_START + offset;
	if (0 == memcmp((void *)mem_address, data, totalLength))
	{
		SEGGER_RTT_printf(0, "Flash access test passed\n");
	}
	else
	{
		SEGGER_RTT_printf(0, "Flash access test failed\n");
	}

	//reenable interrupts
	__enable_irq();
}

void testFlashAccess()
{
	// Disable all interrupts, because Flash may not be accessed while it is 
	// cleared or programmed and interrupts may resides in the Flash memory
	//  
	__disable_irq();

	flexspi_nor_config_t config;
	serial_nor_config_option_t option;
	status_t status;
	uint32_t sector_size = 0x1000; // 4KB
	uint32_t address = 15 * 1024 * 1024 + sector_size;	   // 1MB
	uint32_t page_buffer[256 / sizeof(uint32_t)];
	uint32_t instance = 0;
	option.option0.U = 0xC0000008; // QuadSPI NOR, Frequency: 133MHz
	status = ROM_FLEXSPI_NorFlash_GetConfig(instance, &config, &option);
	if (status != kStatus_Success)
	{
		SEGGER_RTT_printf(0, "Error: FLEXSPI_NorFlash_GetConfig failed\n");
	}

	// Without this delay, the init function may freeze
	SDK_DelayAtLeastUs(1000, CLOCK_GetFreq(kCLOCK_CoreSysClk));
	status = ROM_FLEXSPI_NorFlash_Init(instance, &config);
	if (status != kStatus_Success)
	{
		SEGGER_RTT_printf(0, "Error: FLEXSPI_NorFlash_Init failed\n");
	}
	status = ROM_FLEXSPI_NorFlash_Erase(instance, &config, address, sector_size * 5); // Erase 1 sector
	if (status != kStatus_Success)
	{
		SEGGER_RTT_printf(0, "Error: FLEXSPI_NorFlash_Erase failed\n");
	}
	// Fill data into the page_buffer;
	for (uint32_t i = 0; i < sizeof(page_buffer) / sizeof(page_buffer[0]); i++)
	{
		page_buffer[i] = (i << 24) | (i << 16) | (i << 8) | i;
	}
	// Program data to destination
	for (size_t i = 0; i < 5; i++)
	{

		status = ROM_FLEXSPI_NorFlash_ProgramPage(instance, &config, address, page_buffer); // program 1 page
		if (status != kStatus_Success)
		{
			SEGGER_RTT_printf(0, "Error: FLEXSPI_NorFlash_ProgramPage failed\n");
		}
		address += sizeof(page_buffer);
	}
	// Flush the cache
	DCACHE_CleanInvalidateByRange(address, sizeof(page_buffer));

	// Use memory mapped access to verify whether data are programmed into Flash correctly
	uint32_t mem_address = 0x60000000 + address;
	if (0 == memcmp((void *)mem_address, page_buffer, sizeof(page_buffer)))
	{
		SEGGER_RTT_printf(0, "Flash access test passed\n");
	}
	else
	{
		SEGGER_RTT_printf(0, "Flash access test failed\n");
	}

	//reenable interrupts
	__enable_irq();
}