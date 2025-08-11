#pragma once

#include "stm32f303xe.h"    

//===============================================================================
// Hardware definitions for DMA
#define __DMA_COUNT__  12

#define __DMA_ARRAY__(X, ...)           \
    X(DMA1_Channel1, 0,  __VA_ARGS__)   \
    X(DMA1_Channel2, 1,  __VA_ARGS__)   \
    X(DMA1_Channel3, 2,  __VA_ARGS__)   \
    X(DMA1_Channel4, 3,  __VA_ARGS__)   \
    X(DMA1_Channel5, 4,  __VA_ARGS__)   \
    X(DMA1_Channel6, 5,  __VA_ARGS__)   \
    X(DMA1_Channel7, 6,  __VA_ARGS__)   \
    X(DMA2_Channel1, 7,  __VA_ARGS__)   \
    X(DMA2_Channel2, 8,  __VA_ARGS__)   \
    X(DMA2_Channel3, 9,  __VA_ARGS__)   \
    X(DMA2_Channel4, 10, __VA_ARGS__)   \
    X(DMA2_Channel5, 11, __VA_ARGS__)

//-------------------------------------------------------------------------------
// Hardware definitions for USART
#define __USART_COUNT__ 3

#define __USART_ARRAY__(X, ...) \
    X(USART1, 0, __VA_ARGS__)   \
    X(USART2, 1, __VA_ARGS__)   \
    X(USART3, 2, __VA_ARGS__)

#define __USART_1__ {                           \
        .usart = USART1,                        \
        .rx_dma = { .id = DMA1_Channel5_ID },   \
        .tx_dma = { .id = DMA1_Channel4_ID },   \
        .port = GPIOC,                          \
        .tx_pin = 4,                            \
        .rx_pin = 5,                            \
        .rcc_apb_mask = RCC_APB2ENR_USART1EN,   \
        .rcc_gpio_mask = RCC_AHBENR_GPIOCEN,    \
        .usart_irq = USART1_IRQn,               \
        .tx_dma_irq = DMA1_Channel4_IRQn        \
    }

#define __USART_2__ {                           \
        .usart = USART2,                        \
        .rx_dma = { .id = DMA1_Channel6_ID },   \
        .tx_dma = { .id = DMA1_Channel7_ID },   \
        .port = GPIOA,                          \
        .tx_pin = 2,                            \
        .rx_pin = 3,                            \
        .rcc_apb_mask = RCC_APB1ENR_USART2EN,   \
        .rcc_gpio_mask = RCC_AHBENR_GPIOAEN,    \
        .usart_irq = USART2_IRQn,               \
        .tx_dma_irq = DMA1_Channel7_IRQn        \
    }

#define __USART_3__ {                           \
        .usart = USART3,                        \
        .rx_dma = { .id = DMA1_Channel3_ID },   \
        .tx_dma = { .id = DMA1_Channel2_ID },   \
        .port = GPIOB,                          \
        .tx_pin = 10,                           \
        .rx_pin = 11,                           \
        .rcc_apb_mask = RCC_APB1ENR_USART3EN,   \
        .rcc_gpio_mask = RCC_AHBENR_GPIOBEN,    \
        .usart_irq = USART3_IRQn,               \
        .tx_dma_irq = DMA1_Channel2_IRQn        \
    }

#define __USART_LIST__ __USART_1__, __USART_2__, __USART_3__
//===============================================================================
// Helper macros

#define __IRQ_LIST__(name, idx, func, array, ...)    \
    void name##_IRQHandler(void) {              \                
        func(array[name##_ID]);                 \
    }

#define __ENUM_ENTRY__(name, id, ...)        name##_ID = id,
//===============================================================================
// dma specific helper macros
#define __DEFINE_DMA__(X, ...)  \
{                               \
    .on_ht = NULL,              \
    .on_tc = NULL,              \
    .ht_context = NULL,         \
    .tc_context = NULL,         \
    .dma = X,                   \
    .irq = X##_IRQn             \
},
//-------------------------------------------------------------------------------
// usart specific helper macros
// NAN
//===============================================================================
// final bsp macros for dma
#define BSP_DMA_IRQS(func, array)    __DMA_ARRAY__(__IRQ_LIST__, func, array)
#define BSP_DMA_CHANNELS    __DMA_ARRAY__(__DEFINE_DMA__)
#define BSP_DMA_COUNT       __DMA_COUNT__
//-------------------------------------------------------------------------------
// final bsp macros for usart
#define BSP_USART_IRQS(func, array) __USART_ARRAY__(__IRQ_LIST__, func, array)
#define BSP_USART_CHANNELS __USART_LIST__
#define BSP_USART_COUNT   __USART_COUNT__
//===============================================================================
// enum for dma
typedef enum {
    __DMA_ARRAY__(__ENUM_ENTRY__)
} DmaChannelId_t;
//-------------------------------------------------------------------------------
// enum for usart
typedef enum {
    __USART_ARRAY__(__ENUM_ENTRY__)
} UsartId_t;
//===============================================================================