/**
 * @file embeddedLED.cpp
 * @author Leon Farchau (leon2225)
 * @brief A driver for the LED embedded in the bracelet
 * @version 0.1
 * @date 06.02.2025
 * 
 * @copyright Copyright (c) 2025
 * 
 */

/* -------------------------------- Includes -------------------------------- */
#include "embeddedLED.h"

#include "etl/singleton.h"

#include "peripherals.h"
/* ---------------------------- Private typedefs ---------------------------- */


/* --------------------------------- Globals -------------------------------- */
int32_t g_activeInstances = 0;
volatile float g_ledValues[3] = {0,0,0};


/* ----------------------------- Implementation ----------------------------- */
EmbeddedLED::EmbeddedLED(std::array<char, 10> id) :
    EmbeddedDeviceNode{id, idArr("embed' LED")}
{
    //Link the input ports
    for(size_t i = 0; i < m_colorPorts.size(); i++)
    {
        m_colorPorts[i] = std::make_shared<InputPort<float>>();
        m_inputPorts.push_back(m_colorPorts[i]);
    }
}

/**
 * @brief Process the incoming data and write it to 
 *         the output Port
 * 
 */
void EmbeddedLED::processInData()
{
    // Do nothing
}

/**
 * @brief Process the data from the input Ports and
 *          write them to the device
 * 
 */
void EmbeddedLED::processOutData()
{
    // Get the color data
    float colors[3] = {0,0,0};
    for (size_t i = 0; i < m_colorPorts.size(); i++)
    {
        colors[i] = abs(m_colorPorts[i]->getValue());
        g_ledValues[i] = colors[i];
        colors[i] = MIN(1.0f, colors[i]);
    }

    // Write the data to the output Port
    PWM_UpdatePwmDutycycle(PWM1, PWM1_SM1, PWM1_SM1_LED_G, kPWM_EdgeAligned, colors[0] * 100.0f);
    PWM_UpdatePwmDutycycle(PWM1, PWM1_SM1, PWM1_SM1_LED_B, kPWM_EdgeAligned, colors[1] * 100.0f);
    PWM_UpdatePwmDutycycle(PWM1, PWM1_SM2, PWM1_SM2_LED_R, kPWM_EdgeAligned, colors[2] * 100.0f);
    PWM_SetPwmLdok(PWM1, 2 | 4, true);
}

/**
 * @brief Handler called during device synchronization phase
 * 
 */
void EmbeddedLED::sync()
{
    // Do nothing
}

/**
 * @brief Handler called when the device enters real time mode
 * 
 */
void EmbeddedLED::enterRealTimeMode()
{
    g_activeInstances++;
    if (g_activeInstances == 1)
    {
        // Enable the PWM
        PWM_OutputEnable(PWM1, PWM1_SM2_LED_R, PWM1_SM1);
        PWM_OutputEnable(PWM1, PWM1_SM1_LED_G, PWM1_SM1);
        PWM_OutputEnable(PWM1, PWM1_SM1_LED_B, PWM1_SM1);
        
        // disable all faults
        PWM_SetupFaultDisableMap(PWM1, PWM1_SM1, PWM1_SM1_LED_G, kPWM_faultchannel_0, 0);
        PWM_SetupFaultDisableMap(PWM1, PWM1_SM1, PWM1_SM1_LED_B, kPWM_faultchannel_0, 0);
        PWM_SetupFaultDisableMap(PWM1, PWM1_SM2, PWM1_SM2_LED_R, kPWM_faultchannel_0, 0);

        // Set PWM values
        PWM_UpdatePwmDutycycle(PWM1, PWM1_SM1, PWM1_SM1_LED_G, kPWM_EdgeAligned, 0);
        PWM_UpdatePwmDutycycle(PWM1, PWM1_SM1, PWM1_SM1_LED_B, kPWM_EdgeAligned, 0);
        PWM_UpdatePwmDutycycle(PWM1, PWM1_SM2, PWM1_SM2_LED_R, kPWM_EdgeAligned, 0);

        // Start timers for SM1 and SM2
        PWM_StartTimer(PWM1, 2 | 4);
        
        // Load buffered values 
        PWM_SetPwmLdok(PWM1, 2 | 4, true);
    }
    else
    {
        //TODO: Throw error, as only one instance is availiable and therefore allowed
    }
}

/**
 * @brief Handler called when the device exits real time mode
 * 
 */
void EmbeddedLED::exitRealTimeMode()
{
    g_activeInstances--;
    if (g_activeInstances == 0)
    {
        // Disable the PWM
        PWM_OutputDisable(PWM1, PWM1_SM2_LED_R, PWM1_SM1);
        PWM_OutputDisable(PWM1, PWM1_SM1_LED_G, PWM1_SM1);
        PWM_OutputDisable(PWM1, PWM1_SM1_LED_B, PWM1_SM1);
    }
}