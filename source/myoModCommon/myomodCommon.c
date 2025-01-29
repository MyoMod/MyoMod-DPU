/**
 * @file myomodCommon.cpp
 * @author Leon Farchau (leon2225)
 * @brief A Module that implents some commonly used functions and classes
 * @version 0.1
 * @date 28.01.2025
 * 
 * @copyright Copyright (c) 2025
 * 
 */

//#include "myomodCommon.h"

#include "fsl_qtmr.h"
#include "peripherals.h"

uint64_t time_us_64()
{
    float prescaler = QTMR_GetCurrentTimerCount(TMR1_PERIPHERAL, TMR1_TIMEPRESACLER_CHANNEL);
    uint64_t ms = TMR1_PERIPHERAL->CHANNEL[1].HOLD;
    ms += ( (uint64_t) TMR1_PERIPHERAL->CHANNEL[2].HOLD) << 16;
    ms += ( (uint64_t) TMR1_PERIPHERAL->CHANNEL[3].HOLD) << 32;

    return prescaler / 18.750f + ms * 1000;
}