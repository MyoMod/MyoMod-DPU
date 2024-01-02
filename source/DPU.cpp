/*
 * Copyright 2016-2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    DPU.cpp
 * @brief   Application entry point.
 */


/*******************************************************************************
 * Includes
 ******************************************************************************/

extern "C"
{
    #include <stdio.h>
    #include "peripherals.h"
    #include "board.h"
    #include "pin_mux.h"
    #include "clock_config.h"
    #include "MIMXRT1062.h"
    #include "MIMXRT1062_features.h"
    #include "fsl_debug_console.h"
    #include "fsl_common.h"
    #include "fsl_lpi2c.h"
    #include "fsl_gpio.h"
    #include "fsl_pit.h"
    #include "fsl_edma.h"
    #include "fsl_dmamux.h"
}

#include "SEGGER_RTT.h"

#include "comInterface.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Select USB1 PLL (480 MHz) as master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_SELECT (0U)
/* Clock divider for master lpi2c clock source */
#define LPI2C_CLOCK_SOURCE_DIVIDER (1U)
/* Get frequency of lpi2c clock */
#define LPI2C_CLOCK_FREQUENCY ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 8) / (LPI2C_CLOCK_SOURCE_DIVIDER + 1U))

#define EXAMPLE_I2C_MASTER ((LPI2C_Type *)LPI2C3_BASE)

#define LPI2C_BAUDRATE 1000000U

#define PDS_SIZE 0xF0
#define STATUS_SIZE 0x01

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void i2c_init();
void gpio_init();
void init_hardware();
void init_debug();
void i2c_send();
#ifdef __MCUXPRESSO
// vscode linter can't call c++ functions from c functions
extern "C"
void PIT_IRQHandler(void);
extern "C"
void LPI2C3_IRQHandler(void);
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t g_debugBuffer[32];
uint16_t g_i2cData[3] = {0};
uint8_t g_pdsData[2][PDS_SIZE + STATUS_SIZE] = {0};
const uint16_t g_i2cDataLength = ARRAY_SIZE(g_i2cData);
const uint32_t g_transferChannel = 0;

/**** Register interface ****/
StatusByte_t g_statusByte;
CommonDeviceStatus_t g_commonDeviceStatus;
CommonDeviceInfo_t g_commonDeviceInfo;
CommonDeviceConfiguration_t g_commonDeviceConfiguration;
DeviceSpecificStatus_t g_deviceSpecificStatus;
DeviceSpecificInfo_t g_deviceSpecificInfo;
DeviceSpecificConfiguration_t g_deviceSpecificConfiguration;

uint32_t g_regLength[] = {sizeof(g_statusByte), sizeof(g_commonDeviceStatus), 
                            sizeof(g_commonDeviceInfo), sizeof(g_commonDeviceConfiguration), 
                            sizeof(g_deviceSpecificStatus), sizeof(g_deviceSpecificInfo), 
                            sizeof(g_deviceSpecificConfiguration)};
uint8_t* g_regPointers[] = {(uint8_t*)&g_statusByte, (uint8_t*)&g_commonDeviceStatus,
                            (uint8_t*)&g_commonDeviceInfo, (uint8_t*)&g_commonDeviceConfiguration,
                            (uint8_t*)&g_deviceSpecificStatus, (uint8_t*)&g_deviceSpecificInfo,
                            (uint8_t*)&g_deviceSpecificConfiguration};
AccessRights_t g_regAccessRights[] = {AccessRights_t::Read, AccessRights_t::Read, 
                                        AccessRights_t::Read, AccessRights_t::ReadWrite, 
                                        AccessRights_t::Read, AccessRights_t::Read, 
                                        AccessRights_t::ReadWrite};

/*
 * @brief   Application entry point.
 */

int main(void)
{
    init_hardware();
    init_debug();

    uint32_t testTime = USEC_TO_COUNT(2000, CLOCK_GetFreq(kCLOCK_PerClk));

    /* Enter an infinite loop, just incrementing a counter. */
    while (1)
    {
        /* 'Dummy' NOP to allow source level single stepping of
            tight while() loop */
        __asm volatile("nop");
        size_t txFifoLength;
        LPI2C_MasterGetFifoCounts(EXAMPLE_I2C_MASTER, NULL, &txFifoLength);
        uint32_t currentTime = PIT_GetCurrentTimerCount(PIT_PERIPHERAL, PIT_CHANNEL_0);
        GPIO_PinWrite(BOARD_INITPINS_USR_LED_GPIO, BOARD_INITPINS_USR_LED_GPIO_PIN, txFifoLength > 1);
    }
    return 0;
}

/* PIT_IRQn interrupt handler */
void PIT_IRQHandler(void)
{
    PIT_ClearStatusFlags(PIT_PERIPHERAL, PIT_CHANNEL_0, 1);
    static int i;
    i++;
    GPIO_PinWrite(BOARD_INITPINS_Sync_GPIO, BOARD_INITPINS_Sync_GPIO_PIN, i & 1);

    if (i & 1)
    {
        EDMA_EnableChannelRequest(DMA0, g_transferChannel);
    }

    // start i2c transfer after 100 cycles
    if (i == 100)
    {
        i2c_send();
    }
}

void LPI2C3_IRQHandler(void)
{
    uint32_t status = LPI2C_MasterGetStatusFlags(EXAMPLE_I2C_MASTER);

    LPI2C_MasterClearStatusFlags(EXAMPLE_I2C_MASTER, status);
}

void i2c_buildCommand_ReadStatus(uint16_t* data, uint8_t addr)
{
    uint16_t startCmd = LPI2C_MTDR_CMD(4) | addr << 1 | kLPI2C_Write;
    uint16_t restartCmd = LPI2C_MTDR_CMD(4) | addr << 1 | kLPI2C_Read;
    uint16_t stopCmd = LPI2C_MTDR_CMD(2);

    ControlByte_t controlByte;
    controlByte.ADDR = 0;
    controlByte.HostIn_nHostOut = 1;
    controlByte.PDS_nRegister = 0;

    // Set data to send
    data[0] = startCmd;
    data[1] = LPI2C_MTDR_CMD(0) | *((uint8_t*) &controlByte);
    data[2] = restartCmd;
    data[3] = LPI2C_MTDR_CMD(3) | 0x0;
    data[4] = stopCmd;
}

void i2c_buildCommand_HIn_PDS(uint16_t* data, uint8_t addr)
{
    uint16_t startCmd = LPI2C_MTDR_CMD(4) | addr << 1 | kLPI2C_Read;
    uint16_t stopCmd = LPI2C_MTDR_CMD(2);

    // Set data to send
    data[0] = startCmd;
    data[1] = LPI2C_MTDR_CMD(3) | (0xf0);
    data[2] = stopCmd;
}

void i2c_send()
{
    uint8_t addr = 0x08;
    
    //i2c_buildCommand_ReadStatus(&g_i2cData[0], addr);
    i2c_buildCommand_HIn_PDS(&g_i2cData[0], addr);

    // Enable DMA request
    LPI2C_MasterEnableDMA(EXAMPLE_I2C_MASTER, true, false);
}

void i2c_buildCommands()
{
    const uint8_t nDevices = 4;
    static uint16_t HInCommands[5*nDevices];
    static uint16_t readHInCommands[3*nDevices];
    static uint16_t readHOutCommands[3*nDevices];

    // Build commands for reading status
    for (uint8_t i = 0; i < nDevices; i++)
    {
    }
}

void dma_createHInTcd(edma_tcd_t* tcd, uint16_t* data, uint16_t length, edma_tcd_t* nextTcd, LPI2C_Type* i2cHardware)
{
    EDMA_TcdReset(tcd);
    tcd->ATTR = DMA_ATTR_SSIZE(1) | DMA_ATTR_DSIZE(1);

    tcd->SADDR = (uint32_t)data;
    tcd->SOFF = 2;

    tcd->DADDR = (uint32_t)&i2cHardware->MTDR;
    tcd->DOFF = 0;

    tcd->NBYTES = 2;
    tcd->CITER = length;
    tcd->BITER = length;
    tcd->SLAST = -length * 2;
    tcd->DLAST_SGA = (uint32_t)nextTcd;

    tcd->CSR = DMA_CSR_ESG(1);
}

void dma_init(uint16_t *data, uint16_t length)
{
    /* Enable DMA clock */
    CLOCK_EnableClock(kCLOCK_Dma);

    /* Assign the i2c1-dma request to dma-channel g_transferChannel */
    DMAMUX_Init(DMAMUX_BASE);
    // Procedure from manual 5.5.1
    // 2. Clear the CHCFG[ENBL] and CHCFG[TRIG] fields of the DMA channel.
    DMAMUX_DisableChannel(DMAMUX_BASE, g_transferChannel);

    // 3. Ensure that the DMA channel is properly configured in the DMA. The DMA channel
    //  may be enabled at this point.

    /* eDMA init */
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

    EDMA_Init(DMA0, &dmaConfig);


    // init edma tcd
    static edma_tcd_t HInTcd1 __ALIGNED(32);
    static edma_tcd_t HInTcd2 __ALIGNED(32);

    dma_createHInTcd(&HInTcd1, data, length, &HInTcd2, EXAMPLE_I2C_MASTER);
    dma_createHInTcd(&HInTcd2, data, length, &HInTcd1, EXAMPLE_I2C_MASTER);

    EDMA_TcdEnableAutoStopRequest(&HInTcd2, true);
    
    //EDMA_TcdSetChannelLink(&transferTcd, kEDMA_MajorLink, g_transferChannel);
    EDMA_InstallTCD(DMA0, g_transferChannel, &HInTcd1);
    EDMA_DisableChannelRequest(DMA0, g_transferChannel);

    // 4. Configure the corresponding timer.
    // 5. Select the source to be routed to the DMA channel. Write to the corresponding
    //      CHCFG register, ensuring that the CHCFG[ENBL] and CHCFG[TRIG] fields are
    //      set
    DMAMUX_SetSource(DMAMUX_BASE, g_transferChannel, kDmaRequestMuxLPI2C3);
    DMAMUX_EnableChannel(DMAMUX_BASE, g_transferChannel);
}

void i2c_init()
{
    lpi2c_master_config_t masterConfig;

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

    LPI2C_MasterGetDefaultConfig(&masterConfig);

    /* Change the default baudrate configuration */
    masterConfig.baudRate_Hz = LPI2C_BAUDRATE;
    masterConfig.ignoreAck = 1;

    /* activate IRQ */
    LPI2C_MasterEnableInterrupts(EXAMPLE_I2C_MASTER, kLPI2C_MasterFifoErrFlag | kLPI2C_MasterNackDetectFlag);
    EnableIRQ(LPI2C3_IRQn);

    /* Initialize the LPI2C master peripheral */
    LPI2C_MasterInit(EXAMPLE_I2C_MASTER, &masterConfig, LPI2C_CLOCK_FREQUENCY);

    // Set tx fifo watermark
    LPI2C_MasterSetWatermarks(EXAMPLE_I2C_MASTER, 1, 0);
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

void init_hardware()
{
    /* Init board hardware. */
    BOARD_ConfigMPU();
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    // Set frequency to 200Hz
    PIT_SetTimerPeriod(PIT_PERIPHERAL, PIT_CHANNEL_0, USEC_TO_COUNT(5000, CLOCK_GetFreq(kCLOCK_PerClk)));

    gpio_init();
    dma_init(g_i2cData, g_i2cDataLength);
    i2c_init();
}

void init_debug()
{

    SEGGER_RTT_Init();
    SEGGER_RTT_ConfigUpBuffer(1, "DataOut", &g_debugBuffer[0], sizeof(g_debugBuffer),
                              SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);

    SEGGER_RTT_WriteString(0, "SEGGER Real-Time-Terminal Sample\r\n\r\n");
    SEGGER_RTT_WriteString(0, BOARD_NAME);

}
