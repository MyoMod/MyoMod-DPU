/**
 * @file embeddedIMU.cpp
 * @author Leon Farchau (leon2225)
 * @brief A driver for a ICM-42670-P 6-axis IMU interfaced directly with the DPU via SPI.
 * @version 0.1
 * @date 08.01.2025
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/* -------------------------------- Includes -------------------------------- */
#include "embeddedIMU.h"

#include "etl/singleton.h"

#include "peripherals.h"

#include "ICM42670P.h"
/* ---------------------------- Private typedefs ---------------------------- */
using ImuSingleton = etl::singleton<ICM42670>;


/* --------------------------------- Globals -------------------------------- */


/* ----------------------------- Implementation ----------------------------- */

/**
 * @brief Construct a new Embedded IMU:: Embedded IMU object
 * 
 * @param id    The id of the device
 */
EmbeddedIMU::EmbeddedIMU(std::array<char, 10> id) :
    EmbeddedDeviceNode{id, idArr("embed' IMU")}
{
    // Register the device
    ImuSingleton::create(SPI_IMU_PERIPHERAL);

    //Link the output ports
    for(size_t i = 0; i < m_accelPorts.size(); i++)
    {
        m_accelPorts[i] = std::make_shared<OutputPort<float>>();
        m_outputPorts.push_back(m_accelPorts[i]);
    }
    for(size_t i = 0; i < m_gyroPorts.size(); i++)
    {
        m_gyroPorts[i] = std::make_shared<OutputPort<float>>();
        m_outputPorts.push_back(m_gyroPorts[i]);
    }
}

/**
 * @brief Process the incoming data and write it to 
 *         the output Port
 * 
 */
void EmbeddedIMU::processInData()
{
    // Get latest imu data
    inv_imu_sensor_event_t imu_event;
    ICM42670& imu = ImuSingleton::instance();

    imu.getDataFromRegisters(imu_event);

    // Write the data to the output Port
    for (size_t i = 0; i < 3; i++)
    {
        //transform to m/s^2
        m_accelPorts[i]->setValue((float)imu_event.accel[i] / 2048.0f * 9.81f);
        m_accelPorts[i]->setValid(true);
    }
    for (size_t i = 0; i < 3; i++)
    {
        //transform to dps
        m_gyroPorts[i]->setValue(imu_event.gyro[i] / 16.4);
        m_gyroPorts[i]->setValid(true);
    }
}

/**
 * @brief Process the data from the input Ports and
 *          write them to the device
 * 
 */
void EmbeddedIMU::processOutData()
{
    // Do nothing
}

/**
 * @brief Handler called during device synchronization phase
 * 
 */
void EmbeddedIMU::sync()
{
    // the icm42670p runs in a cyclic mode and does not need
    // to be triggered by the DPU
}

/**
 * @brief Enter the real-time mode
 * 
 */
void EmbeddedIMU::enterRealTimeMode()
{
	// Init IMU (ICM42670P)
	LPSPI_Enable(SPI_IMU_PERIPHERAL, true);

    ICM42670& imu = ImuSingleton::instance();

    volatile int returnVal = imu.begin();
	if (returnVal != 0)
	{
		SEGGER_RTT_printf(0, "IMU init failed\n");
	}

	// Accel ODR = 100 Hz and Full Scale Range = 16G
	imu.startAccel(100,16);
	// Gyro ODR = 100 Hz and Full Scale Range = 2000 dps
	imu.startGyro(100,2000);
}

/**
 * @brief Exit the real-time mode
 * 
 */
void EmbeddedIMU::exitRealTimeMode()
{

}