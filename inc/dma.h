
#pragma once

#include "stm32f303xe.h"
#include "bsp.h"
#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint32_t config;
    void* _hw;
    DmaChannelId_t id;
} DmaInstance_t;

typedef void (*DmaCallback)(DmaInstance_t* instance, void* context);

int registerDmaInstance(DmaInstance_t* instance);
int unregisterDmaInstance(DmaInstance_t* instance);
int applyHalfTransferCallback(DmaInstance_t* instance, DmaCallback callback, void* context);
int applyTransferCompleteCallback(DmaInstance_t* instance, DmaCallback callback, void* context);
int configureMemoryAddress(DmaInstance_t* instance, uint32_t* memory, uint16_t size);
int getDmaTransferCount(DmaInstance_t* instance , uint16_t* count);
int setDmaTransferCount(DmaInstance_t* instance, uint16_t count);
int getDmaChannel(DmaInstance_t* instance, DMA_Channel_TypeDef** channel);
int enableDma(DmaInstance_t* instance);
int disableDma(DmaInstance_t* instance);
void enableDmaInterrupt(DmaInstance_t* instance, uint8_t priority);
int configureMemoryAddress(DmaInstance_t* instance, uint32_t* memory, uint16_t size);
int configurePeripheralAddress(DmaInstance_t* instance, uint32_t* peripheral);