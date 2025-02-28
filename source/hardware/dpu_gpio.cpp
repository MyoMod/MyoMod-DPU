#include "dpu_gpio.h"

constexpr uint32_t PINS[] = {DEBUG_DEBUG0_PIN, DEBUG_DEBUG1_PIN, DEBUG_DEBUG2_PIN, DEBUG_DEBUG3_PIN};


/**
 * @brief Set the value of the debug pins in a single gpio-write
 * 
 * @param value The value to set the pins to (0-15)
 */
void debugSetValue(uint32_t value)
{
    assert(DEBUG_DEBUG0_GPIO == DEBUG_DEBUG1_GPIO && 
            DEBUG_DEBUG1_GPIO == DEBUG_DEBUG2_GPIO && 
            DEBUG_DEBUG2_GPIO == DEBUG_DEBUG3_GPIO);

    constexpr uint32_t mask = ~((1 << DEBUG_DEBUG0_PIN) | (1 << DEBUG_DEBUG1_PIN) | (1 << DEBUG_DEBUG2_PIN) | (1 << DEBUG_DEBUG3_PIN));
    uint32_t priorState = DEBUG_DEBUG0_GPIO->DR & mask;

    for (size_t i = 0; i < 4; i++)
    {
        if (value & (1 << i))
        {
            priorState |= (1u << PINS[i]);
        }
    }

    DEBUG_DEBUG0_GPIO->DR = priorState;
}