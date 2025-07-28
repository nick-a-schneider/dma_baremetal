#include "rcc.h"
#include "usart.h"
#include "buffer.h"
#include "allocator.h"
#include "dma.h"

#include <stdbool.h>
#include <stdint.h>

#define RAW 2048
#define BUFFER_SIZE 256

uint8_t mem[RAW];
Allocator allocator;

void usartRxCallback(UsartInstance_t* instance, void* context __attribute__((unused))) {
    uint8_t data[20];
    uint8_t len = 0;
    while (bufferRead(instance->rx_buffer, &data[len++])) {}
    while (usartTxBusy(instance)) {}
    (void)usartWrite(instance, data, len);
}

int main(void) {
    rccInit();
    
    uint8_t errors = 0;
    uint8_t writes = 0;
    uint8_t pending = 0;

    initAllocator(&allocator, 32, mem, RAW);
    Buffer* rx_buffer = bufferAllocate(&allocator, BUFFER_SIZE - 1);
    Buffer* tx_buffer = bufferAllocate(&allocator, BUFFER_SIZE - 1);

    UsartInstance_t usart2 = {
        .id = USART_2_ID,
        .baudrate = 38400,
        .rx_buffer = rx_buffer,
        .tx_buffer = tx_buffer,
    };

    if(registerUsartInstance(&usart2) != USART_OK) {
        errors++;
    }
    if (applyIdleRxCallback(&usart2, usartRxCallback) != USART_OK) {
        errors++;
    }
    if (enableUsart(&usart2) != USART_OK) {
        errors++;
    }
    while (true) {}
    return 0;
}