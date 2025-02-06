/**
  ******************************************************************************
  MAX11254
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include <bit>
#include <vector>
#include "max11254_hal.h"

// Defines

// global variables
static LPSPI_Type *g_spi;

/* Private function prototypes -----------------------------------------------*/
static inline uint32_t getRegLength(uint8_t reg);

// public functions

void max11254_hal_init(LPSPI_Type *spi)
{
	g_spi = spi;
}

/*
	Read 8 or 24 bit register.
	8bit reg:	STAT1, CTRL1, CTRL2, CTRL3
	24bit reg:	DATA, SOC, SGC, SCOC, SCGC
*/
uint32_t max11254_hal_read_reg(uint8_t reg, void *data)
{
	uint32_t buffer = 0;
	uint32_t byteLength = getRegLength(reg);

	buffer = 0xC1 | (reg << 1);

	lpspi_transfer_t transfer;
	transfer.txData = (uint8_t*) &buffer;
	transfer.rxData = (uint8_t*) &buffer;
	transfer.dataSize = byteLength; // registerlength + command byte
	transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;

	LPSPI_MasterTransferBlocking(g_spi, &transfer);
	
	// TODO: Check if byte swap is needed

	buffer >>= 8; // remove command byte
	
	if(byteLength == 4)
	{
		uint8_t temp = ((uint8_t*)&buffer)[0];
		((uint8_t*)&buffer)[0] = ((uint8_t*)&buffer)[2];
		((uint8_t*)&buffer)[2] = temp;
	}

	// return data if pointer is not NULL
	if (data != NULL)
	{
		memcpy(data, &buffer, byteLength);
	}

	return buffer;
}

/**
 * @brief Writes a register to the MAX11254. If validate is true, the value is read back and compared.
 * 
 * @note The length of the register is determined by the register address.
 * 
 * @param reg 			Register address
 * @param value 		Value to write
 * @param validate 		Validate the written value
 * @return true 		If the value was written successfully
 * @return false 		If the value was not written successfully or not validated
 */
bool max11254_hal_write_reg(uint8_t reg, void *value, bool validate)
{
	uint32_t buffer = (*(uint32_t *)value) << 8; // data without cmd
	buffer |= 0xC0 | (reg << 1); // add cmd
	uint32_t length = getRegLength(reg);

	//byte swapping
	if (length == 4)
	{
		uint8_t temp = ((uint8_t*)&buffer)[1];
		((uint8_t*)&buffer)[1] = ((uint8_t*)&buffer)[3];
		((uint8_t*)&buffer)[3] = temp;
	}

	lpspi_transfer_t transfer;
	transfer.txData = (uint8_t*) &buffer;
	transfer.rxData = (uint8_t*) &buffer;
	transfer.dataSize = length; // registerlength
	transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;

	LPSPI_MasterTransferBlocking(g_spi, &transfer);

	if (validate)
	{
		SDK_DelayAtLeastUs(100, CLOCK_GetFreq(kCLOCK_CoreSysClk));

		volatile uint32_t readValue = max11254_hal_read_reg(reg, NULL);
		// As the value can be 8 or 24 bit the upper bits need to be cleared
		uint32_t clearedValue = *(uint32_t *)value & (0xFFFFFFFF >> (32 - (length-1) * 8));
		return readValue == clearedValue;
	}
	else
	{
		return false;
	}
}

/*
	Send command to MAX11254.
*/
void max11254_hal_send_command(MAX11254_Command_Mode mode, MAX11254_Rate rate, bool blocking)
{
	uint8_t value = 0x80;
	assert((uint8_t)mode <= 0x03 && (uint8_t)mode >= 0x00);
	assert((uint8_t)rate <= 0x0F);
	value |= ((uint8_t)mode << 4);
	value |= (uint8_t)rate;

	lpspi_transfer_t transfer;
	transfer.txData = &value;
	transfer.rxData = &value;
	transfer.dataSize = 1;
	transfer.configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous;

	//TODO: Check if non-blocking is needed
	LPSPI_MasterTransferBlocking(g_spi, &transfer);
}

#if USE_CALIBRATION
/*
	Calibrate MAX11254
*/
void max11254_hal_calibration()
{
	// clear registers
	max11254_hal_write_reg(CTRL3, 0x00, 0, 0); //	NOSYSG NOSYSO NOSCG NOSCO = 0; GAIN = 1;
	max11254_hal_write_reg(SOC, 0x00, 0x00, 0x00);
	max11254_hal_write_reg(SGC, 0x00, 0x00, 0x00);
	max11254_hal_write_reg(SCOC, 0x00, 0x00, 0x00);
	max11254_hal_write_reg(SCGC, 0x00, 0x00, 0x00);
	/*
	//GAIN = 1
	// Enable self calibration registers
	max11254_hal_write_reg(CTRL3, 0x18, 0, 0);	//	NOSYSG=1, NOSYSO=1, NOSCG=0, NOSCO=0; GAIN = 1;
	max11254_hal_self_calib();	// Self-calibration

	// Enable system offset register
	max11254_hal_write_reg(CTRL3, 0x10, 0, 0);	//	NOSYSG=1, NOSYSO=0, NOSCG=0, NOSCO=0; GAIN = 1;
	max11254_hal_sys_offset_calib();	// System-calibration offset

	// Enable system gain register
	max11254_hal_write_reg(CTRL3, 0x00, 0, 0);	//	NOSYSG=0, NOSYSO=0, NOSCG=0, NOSCO=0; GAIN = 1;
	max11254_hal_sys_gain_calib();	// System-calibration gain
	*/

	// GAIN = 16
	//  Enable self calibration registers
	max11254_hal_write_reg(CTRL3, 0x98, 0, 0); //	NOSYSG=1, NOSYSO=1, NOSCG=0, NOSCO=0; GAIN = 16;
	max11254_hal_self_calib();				   // Self-calibration

	// Enable system offset register
	max11254_hal_write_reg(CTRL3, 0x90, 0, 0); //	NOSYSG=1, NOSYSO=0, NOSCG=0, NOSCO=0; GAIN = 16;
	max11254_hal_sys_offset_calib();		   // System-calibration offset

	// Enable system gain register
	max11254_hal_write_reg(CTRL3, 0x80, 0, 0); //	NOSYSG=0, NOSYSO=0, NOSCG=0, NOSCO=0; GAIN = 16;
	max11254_hal_sys_gain_calib();			   // System-calibration gain
}

/*
	System self calibration
		The first level of calibration is the self-calibration where the part performs the required connections to zero and
	full scale internally.
*/
void max11254_hal_self_calib(void)
{
	max11254_hal_send_command(SELF_CALIB, 1); // SCOC
	vTaskDelay(300 / portTICK_PERIOD_MS);
}

/*
	System offset calibration
		A second level of calibration is available where the user can calibrate a system zero scale and system full scale
	by presenting a zero-scale signal or a full-scale signal to the input pins and initiating a system zero scale or
	system gain calibration command.
*/
void max11254_hal_sys_offset_calib(void)
{
	// max11254_hal_send_command(SYS_OFFSET_CALIB,1);
	vTaskDelay(300 / portTICK_PERIOD_MS);
	//	delay(300);
}

/*
	System gain calibration
		A third level of calibration allows for the user to write to the internal calibration registers through the SPI interface
	to achieve any digital offset or scaling the user requires with the following restrictions. The range of digital offset
	correction is QVREF/4. The range of digital gain correction is from 0.5 to 1.5. The resolution of offset correction is 0.5 LSB.
*/
void max11254_hal_sys_gain_calib(void)
{
	// max11254_hal_send_command(SYS_GAIN_CALIB,1);
	vTaskDelay(300 / portTICK_PERIOD_MS);
	//	delay(300);
}
#endif

/*
	Check measure status
	Return masked status register STAT1
*/
uint8_t max11254_hal_meas_status(void)
{
	return max11254_hal_read_reg(MAX11254_STAT_OFFSET) & 0x000C;
}

void max11254_hal_flush_fifo(void)
{
	LPSPI_FlushFifo(g_spi, true, true);
}

/**
 * @brief Configures the SPI peripheral so that it handles the cyclic
 * 			conversions and data read-out without CPU intervention.
 * 
 * 			The only thing that is needed is to handle the irq when 
 * 			data is received.
 * 
 */
void max11254_hal_startCyclicConversion()
{
	// Prepare commands
	// for the conversion we don't want the received (dummy) data to end up in out rx fifo
	const uint32_t startConversionCmd = LPSPI_TCR_RXMSK_MASK | LPSPI_TCR_FRAMESZ(7); // 8bit
	const uint32_t startConversionData = 0xBF;

	const uint32_t readDataCmd = LPSPI_TCR_FRAMESZ(31); // 32bit
	const uint32_t readData[6] = {	0xC1000000 | ((MAX11254_DATA0_OFFSET + (0)) << (1+24)), 0xC1000000 | ((MAX11254_DATA0_OFFSET + 1) << (1+24)), 
									0xC1000000 | ((MAX11254_DATA0_OFFSET + (2)) << (1+24)), 0xC1000000 | ((MAX11254_DATA0_OFFSET + 3) << (1+24)), 
									0xC1000000 | ((MAX11254_DATA0_OFFSET + (4)) << (1+24)), 0xC1000000 | ((MAX11254_DATA0_OFFSET + 5) << (1+24))};

	// Prepare the transfer
	g_spi->TCR = startConversionCmd;
	g_spi->TDR = startConversionData;

	g_spi->TCR = readDataCmd;
	for (size_t i = 0; i < 6; i++)
	{
		g_spi->TDR = readData[i];
	}
}

void max11254_hal_readCyclicData(int32_t *data, uint32_t length)
{
	// Read the data
	for (size_t i = 0; i < length; i++)
	{
		int32_t rawVal = g_spi->RDR;
		data[i] = (rawVal << 8) >> 8; // sign extend 24bit value to 32bit
	}
}

void max11254_hal_stopCyclicConversion()
{
	LPSPI_Enable(g_spi, false);

	LPSPI_FlushFifo(g_spi, true, true);
    LPSPI_ClearStatusFlags(g_spi, (uint32_t)kLPSPI_AllStatusFlag);

	g_spi->CFGR0 = 0;

	// TODO: Disable interrupts

	LPSPI_Enable(g_spi, true);
}

/* Private functions ---------------------------------------------------------*/
static inline uint32_t getRegLength(uint8_t reg)
{
	bool is8bit = ((MAX11254_CTRL1_OFFSET <= reg && reg <= MAX11254_CTRL3_OFFSET) || 
					reg == MAX11254_GPO_DIR_OFFSET || reg == MAX11254_SEQ_OFFSET ||
					reg == MAX11254_GPIO_CTRL_OFFSET);
	return (is8bit) ? 2 : 4;
}

/*****	END OF FILE	****/