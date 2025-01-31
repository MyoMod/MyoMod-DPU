#pragma once
/**
 ******************************************************************************
 * @file    MAX11254.h
 * @author  Domen Jurkovic
 * @version V1.0
 * @date    18-Nov-2015
 * @brief   Header for MAX11254.c module
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include "math.h"
#include <array>

#include "max11254_hal.h"

/* Defines -------------------------------------------------------------------*/
// #define MAX11254_SIMULATED
#ifdef MAX11254_SIMULATED
#define MAX11254_SIM_FUNC(x) (sin(2 * 3.14 * ((float)x)) * ((float)(1 << 20)))
// #define MAX11254_SIM_FUNC(x)    (x)
#define MAX11254_SIM_STEP_SIZE (1)
#define MAX11254_SIM_CHN_OFFSET (300)
#endif
#define MAX11254_NUM_CHANNELS (6)
#define MAX11254_SAMPLE_RATE_ALPHA (0.01f)

/* ---------------------------- type definitions ---------------------------- */
// callback function for new data arrival
typedef void (*MAX11254_Callback)(std::array<int32_t, MAX11254_NUM_CHANNELS> &measurements, bool clipped, bool rangeExceeded, bool error);

/* Exported functions ------------------------------------------------------- */

class MAX11254
{
private:
#ifdef MAX11254_SIMULATED
    uint32_t _lastIndex;
    uint64_t _nextUpdate;
#endif

    MAX11254_Rate _rate;
    MAX11254_Seq_Mode _mode;
    uint8_t _pga_gain;
    uint8_t _channels;
    bool _singleCycle;
    bool _is2sComplement;
    bool _inCyclicMode;

    LPSPI_Type *_spi;
    bool _rxFifoMustBeFlushed;

    uint64_t _lastConversionTime;
    float _actualSampleRate;
    volatile bool _irqCalled = false;

    MAX11254_Callback _callback;

    MAX11254_Rate sampleRate2Rate(float sample_rate, bool singleCycle, float *actualSampleRate = NULL);
    float rate2SampleRate(MAX11254_Rate rate, bool singleCycle);
    MAX11254_Gain interger2PGA(uint8_t integer, uint8_t *actualGain = NULL);
    uint8_t PGA2Integer(MAX11254_Gain pga);

    void setMode(MAX11254_Seq_Mode mode);
    MAX11254_Seq_Mode getMode(void);

    bool resetADC(uint32_t timeout);
    bool setupADC(void);

public:
    MAX11254(LPSPI_Type *spi, MAX11254_Callback callback);
    ~MAX11254();

    bool begin(void);

    float setSampleRate(float sample_rate);
    uint8_t setGain(uint8_t gain);
    void setChannels(uint8_t channel);

    float getSampleRate(bool getActual = true);
    uint8_t getGain(void);
    uint8_t getChannels(void);

    MAX11254_STAT getStatus(void);
    bool dataAvailable(void);
    int32_t readMeasurement(uint32_t channel);

    void IRQ_handler(void);
    void async_handler(void);
    void startConversion(bool ignoreState = false);
    bool stopConversion(uint32_t timeout);

    void startCyclicConversion();
    void stopCyclicConversion();
};

/*****	END OF FILE	****/