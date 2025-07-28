#pragma once

#include <stdint.h>
#include <stddef.h>
#include "stm32f303xe.h"

typedef struct {
    uint8_t pin;  // GPIO pin number (0-15)
    uint8_t af;   // Alternate function number (0-15)
} GPIOConfig_t;

void applyGPIOConfig(GPIO_TypeDef *port, GPIOConfig_t config);