#include "gpio.h"
#include "stm32f303xe.h"
#include <stdint.h>

void applyGPIOConfig(GPIO_TypeDef *port, GPIOConfig_t config) {
    uint8_t pin = config.pin;
    uint8_t af = config.af;
    // clear the mode bits for the pin
    port->MODER &= ~(3U << (pin * 2));
    // set the mode to alternate function
    port->MODER |= (2U << (pin * 2));
    // set the alternate function for the pin
    // AFR[0] is used for pins 0-7, AFR[1] for pins 8-15
    if (pin < 8) {
        port->AFR[0] &= ~(0xFU << (pin * 4));
        port->AFR[0] |= (af << (pin * 4));
    }
    else {
        port->AFR[1] &= ~(0xFU << ((pin - 8) * 4));
        port->AFR[1] |= (af << ((pin - 8) * 4));
    }
    // set the speed for the pin
    port->OSPEEDR |= (3U << (pin * 2));
}