/**
 * @file button.h
 * @author Leon Farchau (leon2225)
 * @brief 
 * @version 0.1
 * @date 26.07.2024
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include "etl/debounce.h"
#include "fsl_gpio.h"



template <unsigned int SAMPLE_TIME>
class Button
{
public:
    Button(GPIO_Type *base, uint32_t pin, bool activeLow = false)
        : m_base(base), m_pin(pin), m_activeLow(activeLow)
    {
        gpio_pin_config_t config = {
            kGPIO_DigitalInput,
            0,
        };
        GPIO_PinInit(base, pin, &config);
    }
    ~Button() = default;

    bool update()
    {
        bool state = (((m_base->DR) >> m_pin) & 0x1U);
        state ^= m_activeLow;
        return m_debounce.add(state);
    }

    bool isSet()
    {
        return m_debounce.is_set();
    }
    bool hasChanged()
    {
        return m_debounce.has_changed();
    }
    bool isHeld()
    {
        return m_debounce.is_held();
    }
    bool isRepeating()
    {
        return m_debounce.is_repeating();
    }
private:
    etl::debounce<50 / SAMPLE_TIME, 1000 / SAMPLE_TIME, 250 / SAMPLE_TIME> m_debounce;
    const GPIO_Type *m_base;
    const uint32_t m_pin;
    const bool m_activeLow;
};