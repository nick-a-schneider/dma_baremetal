#include "stm32f303xe.h"
#include "dma.h"
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#define DEFINE_DMA_INSTANCE(X)  \
{                               \
    .on_ht = NULL,              \
    .on_tc = NULL,              \
    .ht_context = NULL,         \
    .tc_context = NULL,         \
    .dma = X,                   \
    .irq = X##_IRQn             \
}

typedef struct {
    DmaCallback on_ht;
    DmaCallback on_tc;
    void* ht_context;
    void* tc_context;
    volatile DMA_Channel_TypeDef* dma;
    IRQn_Type irq;
} DmaStaticConfig_t;

static DmaInstance_t* registered_instances[12] = { NULL };

static DmaStaticConfig_t DMA_STATIC_CONFIG[12] = {
    DEFINE_DMA_INSTANCE(DMA1_Channel1),
    DEFINE_DMA_INSTANCE(DMA1_Channel2),
    DEFINE_DMA_INSTANCE(DMA1_Channel3),
    DEFINE_DMA_INSTANCE(DMA1_Channel4),
    DEFINE_DMA_INSTANCE(DMA1_Channel5),        
    DEFINE_DMA_INSTANCE(DMA1_Channel6),
    DEFINE_DMA_INSTANCE(DMA1_Channel7),
    DEFINE_DMA_INSTANCE(DMA2_Channel1),
    DEFINE_DMA_INSTANCE(DMA2_Channel2),
    DEFINE_DMA_INSTANCE(DMA2_Channel3),
    DEFINE_DMA_INSTANCE(DMA2_Channel4),
    DEFINE_DMA_INSTANCE(DMA2_Channel5)
};

void dmaHwConfigure(DmaInstance_t* instance);
void DMA_IRQBase(DmaInstance_t* instance);

int registerDmaInstance(DmaInstance_t* instance) {
    if (!instance) return -EINVAL;
    DmaChannelId_t dma_id = instance->id;
    if (dma_id < 0 || dma_id > 11) return -ENODEV;
    if (registered_instances[dma_id] != NULL) return -EPERM;
    instance->_hw = (void*)&DMA_STATIC_CONFIG[dma_id];
    registered_instances[dma_id] = instance;
    dmaHwConfigure(instance);
    return 0;
}

int unregisterDmaInstance(DmaInstance_t* instance) {
    if (!instance) return -EINVAL;
    DmaChannelId_t dma_id = instance->id;
    if (dma_id < 0 || dma_id > 11) return -ENODEV;
    if (registered_instances[dma_id] != instance) return -EPERM;
    registered_instances[dma_id] = NULL;
    instance->_hw = NULL;
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
    DMA_Channel_TypeDef* channel = ((DmaStaticConfig_t*)instance->_hw)->dma;
    channel->CCR &= ~DMA_CCR_EN;
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
}

void dmaHwConfigure(DmaInstance_t* instance) {
    DMA_Channel_TypeDef* channel = ((DmaStaticConfig_t*)instance->_hw)->dma;
    const uint32_t rcc_en = (instance->id < 7) ? RCC_AHBENR_DMA1EN : RCC_AHBENR_DMA2EN;
    RCC->AHBENR |= rcc_en;
    channel->CCR &= ~DMA_CCR_EN;
    channel->CPAR = instance->peripheral;
    channel->CMAR = instance->buffer->raw;
    channel->CNDTR = instance->buffer->size;
    channel->CCR = instance->config;
    return;
}

void DMA_IRQBase(DmaInstance_t* instance) {
    if (!instance || !instance->_hw) return;

    DMA_TypeDef* dma = (instance->id < 7) ? DMA1 : DMA2;
    const uint32_t TC_BIT = (1U << (4 * ((instance->id % 7))));
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

void DMA1_Channel1_IRQHandler(void) {
    DMA_IRQBase(registered_instances[DMA1_Channel1_ID]);
}
void DMA1_Channel2_IRQHandler(void) {
    DMA_IRQBase(registered_instances[DMA1_Channel2_ID]);
}
void DMA1_Channel3_IRQHandler(void) {
    DMA_IRQBase(registered_instances[DMA1_Channel3_ID]);
}
void DMA1_Channel4_IRQHandler(void) {
    DMA_IRQBase(registered_instances[DMA1_Channel4_ID]);
}
void DMA1_Channel5_IRQHandler(void) {
    DMA_IRQBase(registered_instances[DMA1_Channel5_ID]);
}
void DMA1_Channel6_IRQHandler(void) {
    DMA_IRQBase(registered_instances[DMA1_Channel6_ID]);
}
void DMA1_Channel7_IRQHandler(void) {
    DMA_IRQBase(registered_instances[DMA1_Channel7_ID]);
}
void DMA2_Channel1_IRQHandler(void) {
    DMA_IRQBase(registered_instances[DMA2_Channel1_ID]);
}
void DMA2_Channel2_IRQHandler(void) {
    DMA_IRQBase(registered_instances[DMA2_Channel2_ID]);
}
void DMA2_Channel3_IRQHandler(void) {
    DMA_IRQBase(registered_instances[DMA2_Channel3_ID]);
}
void DMA2_Channel4_IRQHandler(void) {
    DMA_IRQBase(registered_instances[DMA2_Channel4_ID]);
}
void DMA2_Channel5_IRQHandler(void) {
    DMA_IRQBase(registered_instances[DMA2_Channel5_ID]);
}
