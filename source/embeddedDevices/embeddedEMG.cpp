/**
 * @file EmbeddedEMG.cpp
 * @author Leon Farchau (leon2225)
 * @brief A driver for a MAX11254 6Ch diff. ADC that is used for EMG measurement interfaced directly with the DPU via SPI.
 * @version 0.1
 * @date 08.01.2025
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/* -------------------------------- Includes -------------------------------- */
#include "embeddedEMG.h"

#include "etl/singleton.h"
#include "etl/queue_spsc_atomic.h"

#include "peripherals.h"
#include "dpu_gpio.h"

#include "max11254.h"
/* ---------------------------- Private typedefs ---------------------------- */
using AdcSingleton = etl::singleton<MAX11254>;

/* --------------------------------- Globals -------------------------------- */
volatile int32_t adcRaw[6] = {0,0,0,0,0,0};
etl::queue_spsc_atomic<std::array<int32_t, 6>, 20> g_adcQueue;
volatile uint32_t g_adcCounter = 0;

/* ---------------------------- Private functions --------------------------- */
void newAdcData(std::array<int32_t, MAX11254_NUM_CHANNELS> &measurements, bool clipped, bool rangeExceeded, bool error);

/* ----------------------------- Implementation ----------------------------- */

/**
 * @brief Construct a new embeddedEMG Node
 * 
 * @param id                ID of the device
 * @param amplification     Selects the amplification of the input diff amp
 *                          Possible values:
 *                          0: 1x (amplifier bypassed)
 * |                        1: 1x (through amplifier)
 * |                        2: 2x
 * |                        4: 4x
 * |                        8: 8x
 * |                       16: 16x
 * |                       32: 32x
 * |                       64: 64x
 * |                      128: 128x
 *                          
 */
EmbeddedEMG::EmbeddedEMG(std::array<char, 10> id, uint8_t amplification) :
    EmbeddedDeviceNode{id, idArr("embed' EMG")},
    m_amplification(amplification)
{
    assert(amplification < 128);

    // Register the device
    AdcSingleton::create(SPI_ADC_PERIPHERAL, newAdcData);

    //Link the output ports
    for(size_t i = 0; i < m_emgPorts.size(); i++)
    {
        m_emgPorts[i] = std::make_shared<OutputPort<std::array<float, 15>>>();
        m_outputPorts.push_back(m_emgPorts[i]);
    }
}

/**
 * @brief Process the incoming data and write it to 
 *         the output Port
 * 
 */
void EmbeddedEMG::processInData()
{
    // Read all data, as each output port contain all EMBEDDED_EMG_OVERSAMPLING samples
    std::array<int32_t, MAX11254_NUM_CHANNELS> measurement;
    std::array<std::array<float, EMBEDDED_EMG_OVERSAMPLING>, MAX11254_NUM_CHANNELS> outputVectors;

    size_t subIndex = 0;
    for (; subIndex < EMBEDDED_EMG_OVERSAMPLING; subIndex++)
    {
        // pop latest adc data from queue
        if(!g_adcQueue.pop(measurement))
        {
            // queue is empty
            break;
        }

        // fill output vectors and transform to mV
        for (size_t j = 0; j < MAX11254_NUM_CHANNELS; j++)
        {
            float outVal = ((float)measurement[j] * 3.0f) / 8388607.0f;
            outVal /= m_amplification > 0 ? m_amplification : 1;
            outputVectors[j][subIndex] = outVal;
        }
    }

    if (subIndex != EMBEDDED_EMG_OVERSAMPLING)
    {
        // not enough data available, don't write to output ports
        return;
    }

    // Write the data to the output Port
    for (size_t i = 0; i < MAX11254_NUM_CHANNELS; i++)
    {
        m_emgPorts[i]->setValue(outputVectors[i]);
        m_emgPorts[i]->setValid(true);
    }
}

/**
 * @brief Process the data from the input Ports and
 *          write them to the device
 * 
 */
void EmbeddedEMG::processOutData()
{
    // Do nothing
}

/**
 * @brief Handler called during device synchronization phase
 * 
 */
void EmbeddedEMG::sync()
{
    // the max11254 runs in a cyclic mode and does not need
    // to be triggered by the DPU
}

/**
 * @brief Enter the real-time mode
 * 
 */
void EmbeddedEMG::enterRealTimeMode()
{
	// Init
	LPSPI_Enable(SPI_ADC_PERIPHERAL, true);

    MAX11254& adc = AdcSingleton::instance();

    volatile int returnVal = adc.begin();
	if (returnVal != 0)
	{
		SEGGER_RTT_printf(0, "ADC init failed\n");
	}

    adc.setGain(m_amplification);
	// Start the cyclic conversion - from now on all is done via interrupts
	adc.startCyclicConversion();
}

/**
 * @brief Exit the real-time mode
 * 
 */
void EmbeddedEMG::exitRealTimeMode()
{
    MAX11254& adc = AdcSingleton::instance();
    adc.stopCyclicConversion();
}

void newAdcData(std::array<int32_t, MAX11254_NUM_CHANNELS> &measurements, bool clipped, bool rangeExceeded, bool error)
{
	//SEGGER_RTT_printf(0, "New ADC data: %d\n", measurements[0]);
	for (size_t i = 0; i < 6; i++)
	{
		adcRaw[i] = measurements[i];
	}
	g_adcQueue.push(measurements);
	g_adcCounter = g_adcQueue.size();
}

extern "C"
{
 /* EXTERNAL_CONNECTIONS_GPIO1_14_handle callback function */
void EXTERNAL_CONNECTIONS_GPIO1_14_callback(void *param) {
    /* Place your code here */
    MAX11254& adc = AdcSingleton::instance();
    adc.IRQ_handler();
    adc.async_handler();
}
}