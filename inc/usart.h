#pragma once

#include <stdint.h>
#include <stddef.h>
#include "buffer.h"

#define USART_OK 0

typedef enum {
    USART_1_ID = 0,
    USART_2_ID = 1,
    USART_3_ID = 2
} UsartId_t;

typedef struct {
    const void *_hw;  // opaque handle to internal static config
    Buffer *rx_buffer;
    Buffer *tx_buffer;
    uint32_t baudrate;  // Baud rate for USART communication
    UsartId_t id;  // ID of the USART instance
    // SemaphoreHandle_t tx_semaphore;
} UsartInstance_t;

typedef void (*UsartCallback)(UsartInstance_t* instance, void* context);

int registerUsartInstance(UsartInstance_t* instance);
int unregisterUsartInstance(UsartInstance_t* instance);
int applyIdleRxCallback(UsartInstance_t* instance, UsartCallback callback);
int usartBufferWrite(UsartInstance_t* instance, uint8_t* message, uint16_t length);
int usartWrite(UsartInstance_t* instance, uint8_t* message, uint16_t length);
int enableUsart(UsartInstance_t* instance);
int disableUsart(UsartInstance_t* instance);
bool usartTxBusy(UsartInstance_t* instance);