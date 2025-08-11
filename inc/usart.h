#pragma once

#include "bsp.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define USART_OK 0

typedef struct {
    const void *_hw;  // opaque handle to internal static config
    uint16_t rx_len;
    uint32_t baudrate;  // Baud rate for USART communication
    UsartId_t id;  // ID of the USART instance
    // SemaphoreHandle_t tx_semaphore;
} UsartInstance_t;

typedef void (*UsartCallback)(UsartInstance_t* instance, void* context);

int registerUsartInstance(UsartInstance_t* instance);
int unregisterUsartInstance(UsartInstance_t* instance);

int applyRxIdleCallback(UsartInstance_t* instance, UsartCallback callback, void* context);
int applyTxCompleteCallback(UsartInstance_t* instance, UsartCallback callback, void* context);

int usartWrite(UsartInstance_t* instance, uint32_t* message, uint16_t length);
int usartSetReadAddress(UsartInstance_t* instance, uint32_t* address, uint16_t length);

int enableUsart(UsartInstance_t* instance);
int disableUsart(UsartInstance_t* instance);