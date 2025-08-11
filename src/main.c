#include "rcc.h"
#include "usart.h"
#include "queue.h"

#include <stdbool.h>
#include <stdint.h>

#define MESSAGE_SIZE 32
#define QUEUE_SIZE 12

typedef struct {
    Queue* queue;
    uint16_t errors;
    uint16_t q_rx_idx;
    uint16_t q_tx_idx;
    uint32_t* rx_addr;
    uint32_t* tx_addr;
} UsartQueueState_t;

void usartRxCallback(UsartInstance_t* instance, void* context) {
    UsartQueueState_t* state = (UsartQueueState_t*)context;

    int res = queueWriteRelease(state->queue, state->q_rx_idx);
    if (res < QUEUE_OK) state->errors++;

    state->q_rx_idx = queueWriteClaim(state->queue, &state->rx_addr);
    if (state->q_rx_idx < QUEUE_OK) state->errors++;

    res = usartSetReadAddress(instance, state->rx_addr, MESSAGE_SIZE);
    if (res < USART_OK) state->errors++;

    state->q_tx_idx = queueReadClaim(state->queue, &state->tx_addr);
    if (state->q_tx_idx < QUEUE_OK) state->errors++;

    res = usartWrite(instance, state->tx_addr, instance->rx_len);
    if (res < USART_OK) state->errors++;
}

void usartTxCallback(UsartInstance_t* instance __attribute__((unused)), void* context) {
    UsartQueueState_t* state = (UsartQueueState_t*)context;
    int res = queueReadRelease(state->queue, state->q_tx_idx);
    if (res < QUEUE_OK) state->errors++;
}

int main(void) {

    rccInit();

    CREATE_QUEUE(queue, MESSAGE_SIZE, QUEUE_SIZE);
    
    UsartQueueState_t state = {
        .queue = &queue,
        .q_rx_idx = 0,
        .q_tx_idx = 0,
        .rx_addr = NULL,
        .tx_addr = NULL,
        .errors = 0
    };

    UsartInstance_t usart2 = {
        .id = USART2_ID,
        .baudrate = 38400,
    };

    int res;
    res = registerUsartInstance(&usart2);
    if (res < USART_OK) state.errors++;

    res = applyRxIdleCallback(&usart2, usartRxCallback, (void*)&state);
    if (res < USART_OK) state.errors++;

    res = applyTxCompleteCallback(&usart2, usartTxCallback, (void*)&state);
    if (res < USART_OK) state.errors++;

    state.q_rx_idx = queueWriteClaim(state.queue, &state.rx_addr);
    if (state.q_rx_idx < QUEUE_OK) state.errors++;

    res = usartSetReadAddress(&usart2, state.rx_addr, MESSAGE_SIZE);
    if (res < USART_OK) state.errors++;

    res = enableUsart(&usart2);
    if (res < USART_OK) state.errors++;

    while (true) {}

    return 0;
}