
#pragma once

#include "stm32f303xe.h"
#include "buffer.h"
#include <stdint.h>
#include <stddef.h>

typedef enum {
    DMA1_Channel1_ID = 0,
    DMA1_Channel2_ID = 1,    
    DMA1_Channel3_ID = 2,
    DMA1_Channel4_ID = 3,
    DMA1_Channel5_ID = 4,
    DMA1_Channel6_ID = 5,
    DMA1_Channel7_ID = 6,
    DMA2_Channel1_ID = 7,
    DMA2_Channel2_ID = 8,
    DMA2_Channel3_ID = 9,
    DMA2_Channel4_ID = 10,
    DMA2_Channel5_ID = 11
} DmaChannelId_t;

typedef struct {
    uint32_t config;
    uint32_t* peripheral;
    Buffer* buffer;
    void* _hw;
    DmaChannelId_t id;
} DmaInstance_t;

typedef void (*DmaCallback)(struct DmaInstance_t* instance, void* context);

int registerDmaInstance(DmaInstance_t* instance);
int unregisterDmaInstance(DmaInstance_t* instance);
int applyHalfTransferCallback(DmaInstance_t* instance, DmaCallback callback, void* context);
int applyTransferCompleteCallback(DmaInstance_t* instance, DmaCallback callback, void* context);
int getDmaTransferCount(DmaInstance_t* instance , uint16_t* count);
int setDmaTransferCount(DmaInstance_t* instance, uint16_t count);
int getDmaChannel(DmaInstance_t* instance, DMA_Channel_TypeDef** channel);
int enableDma(DmaInstance_t* instance);
int disableDma(DmaInstance_t* instance);