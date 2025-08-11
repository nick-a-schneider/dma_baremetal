#include "stm32f303xe.h"
#include "bsp.h"
#include "dma.h"
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#define DISABLE_DMA_WAIT(channel)                   \
    channel->CCR &= ~DMA_CCR_EN;                    \
    while (channel->CCR & DMA_CCR_EN) { __NOP(); }  \

typedef struct {
    uint16_t size;
    DmaCallback on_ht;
    DmaCallback on_tc;
    void* ht_context;
    void* tc_context;
    volatile DMA_Channel_TypeDef* dma;
    IRQn_Type irq;
} DmaStaticConfig_t;

static DmaInstance_t* registered_instances[BSP_DMA_COUNT] = { NULL };

static DmaStaticConfig_t DMA_STATIC_CONFIG[BSP_DMA_COUNT] = {BSP_DMA_CHANNELS};

void DMA_IRQBase(DmaInstance_t* instance);

int registerDmaInstance(DmaInstance_t* instance) {
    if (!instance) return -EINVAL;
    DmaChannelId_t dma_id = instance->id;
    if (dma_id < 0 || dma_id >= BSP_DMA_COUNT) return -ENODEV;
    if (registered_instances[dma_id] != NULL) return -EPERM;
    instance->_hw = (void*)&DMA_STATIC_CONFIG[dma_id];
    registered_instances[dma_id] = instance;

    DMA_Channel_TypeDef* channel = ((DmaStaticConfig_t*)instance->_hw)->dma;
    const uint32_t rcc_en = (instance->id < 7) ? RCC_AHBENR_DMA1EN : RCC_AHBENR_DMA2EN; // TODO: BSP
    RCC->AHBENR |= rcc_en;
    DISABLE_DMA_WAIT(channel);
    channel->CCR = instance->config;

    return 0;
}

int unregisterDmaInstance(DmaInstance_t* instance) {
    if (!instance) return -EINVAL;
    DmaChannelId_t dma_id = instance->id;
    if (dma_id < 0 || dma_id >= BSP_DMA_COUNT) return -ENODEV;   // TODO: BSP
    if (registered_instances[dma_id] != instance) return -EPERM;
    registered_instances[dma_id] = NULL;
    instance->_hw = NULL;
    return 0;
}

int configureMemoryAddress(DmaInstance_t* instance, uint32_t* memory, uint16_t size) {
    int res = disableDma(instance);
    if (res) return res;
    DMA_Channel_TypeDef* channel;
    res = getDmaChannel(instance, &channel);
    if (res) return res;
    ((DmaStaticConfig_t*)instance->_hw)->size = size;
    channel->CMAR = (uint32_t)memory;
    channel->CNDTR = size;
    return 0;
}

int configurePeripheralAddress(DmaInstance_t* instance, uint32_t* peripheral) {
    int res = disableDma(instance);
    if (res) return res;
    DMA_Channel_TypeDef* channel;
    res = getDmaChannel(instance, &channel);
    if (res) return res;
    channel->CPAR = (uint32_t)peripheral;
    return 0;
}

int applyHalfTransferCallback(DmaInstance_t* instance, DmaCallback callback, void* context) {
    if (!instance) return -EINVAL;
    DmaStaticConfig_t* static_config = (DmaStaticConfig_t*)instance->_hw;
    static_config->on_ht = callback;
    static_config->ht_context = context;
    return 0;
}

int applyTransferCompleteCallback(DmaInstance_t* instance, DmaCallback callback, void* context) {
    if (!instance) return -EINVAL;
    DmaStaticConfig_t* static_config = (DmaStaticConfig_t*)instance->_hw;
    static_config->on_tc = callback;
    static_config->tc_context = context;
    return 0;
}

int enableDma(DmaInstance_t* instance) {
    if (!instance) return -EINVAL;
    if (!instance->_hw) return -EPERM;
    DMA_Channel_TypeDef* channel = ((DmaStaticConfig_t*)instance->_hw)->dma;
    channel->CCR |= DMA_CCR_EN;
    return 0;
}

int disableDma(DmaInstance_t* instance) {
    if (!instance) return -EINVAL;
    if (!instance->_hw) return -EPERM;
    DmaStaticConfig_t* static_config = (DmaStaticConfig_t*)instance->_hw;
    DMA_Channel_TypeDef* channel = static_config->dma;
    // if (0 < channel->CNDTR  && channel->CNDTR < static_config->size) return -EBUSY;
    DISABLE_DMA_WAIT(channel);
    channel->CNDTR = 0; 
    return 0;
}

void enableDmaInterrupt(DmaInstance_t* instance, uint8_t priority) {
    if (!instance) return;
    DmaStaticConfig_t* static_config = (DmaStaticConfig_t*)instance->_hw;
    if(!static_config) return;
    NVIC_ClearPendingIRQ(static_config->irq);
    NVIC_DisableIRQ(static_config->irq);
    static_config->dma->CCR |= DMA_CCR_TCIE;
    NVIC_SetPriority(static_config->irq, priority);
    NVIC_EnableIRQ(static_config->irq);
}

int getDmaTransferCount(DmaInstance_t* instance, uint16_t *count) {
    if (!instance) return -EINVAL;
    if (!instance->_hw) return -EPERM;
    if (!count) return -EINVAL;
    *count = ((DmaStaticConfig_t*)instance->_hw)->dma->CNDTR;
    return 0;
}

int setDmaTransferCount(DmaInstance_t* instance, uint16_t count) {
    if (!instance) return -EINVAL;
    if (!instance->_hw) return -EPERM;
    ((DmaStaticConfig_t*)instance->_hw)->dma->CNDTR = count;
    return 0;
}

int getDmaChannel(DmaInstance_t* instance, DMA_Channel_TypeDef** channel) {
    if (!instance) return -EINVAL;
    if (!instance->_hw) return -EPERM;
    if (!channel) return -EINVAL;
    *channel = ((DmaStaticConfig_t*)instance->_hw)->dma;
    return 0;
}

void DMA_IRQBase(DmaInstance_t* instance) {
    if (!instance || !instance->_hw) return;

    DMA_TypeDef* dma = (instance->id < 7) ? DMA1 : DMA2;        // TODO: BSP
    const uint32_t TC_BIT = (1U << (4 * ((instance->id % 7)))); // TODO: BSP
    DmaStaticConfig_t* static_config = (DmaStaticConfig_t*)instance->_hw;

    if (dma->ISR & TC_BIT) {
        dma->IFCR |= TC_BIT;
        if (static_config->on_ht) {
            static_config->on_ht(instance, static_config->ht_context);
        }
        if (static_config->on_tc) {
            static_config->on_tc(instance, static_config->tc_context);
        }
    }
}

BSP_DMA_IRQS(DMA_IRQBase, registered_instances);