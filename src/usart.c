
#include "stm32f303xe.h"
#include "usart.h"
#include "buffer.h"
#include "dma.h"
#include "gpio.h"

#include <stdint.h>
#include <errno.h>

typedef struct {
    volatile USART_TypeDef *usart;
    volatile GPIO_TypeDef *port;
    DmaInstance_t rx_dma;
    DmaInstance_t tx_dma;
    uint8_t rx_pin;
    uint8_t tx_pin;
    uint32_t rcc_apb_mask;
    uint32_t rcc_gpio_mask;
    IRQn_Type usart_irq;
    IRQn_Type tx_dma_irq;
    uint16_t rx_count;
    uint16_t tx_count;
    bool tx_busy;
    UsartCallback rx_callback;
} UsartStaticConfig_t;

static UsartStaticConfig_t USART_STATIC_CONFIG[3] = {
// #ifdef USE_USART1
    {
        .usart = USART1,
        .rx_dma = { .id = DMA1_Channel5_ID },
        .tx_dma = { .id = DMA1_Channel4_ID },
        .port = GPIOA,
        .tx_pin = 2,
        .rx_pin = 3,
        .rcc_apb_mask = RCC_APB2ENR_USART1EN,
        .rcc_gpio_mask = RCC_AHBENR_GPIOAEN,
        .usart_irq = USART1_IRQn,
        .tx_dma_irq = DMA1_Channel4_IRQn
    },
// #endif
// #ifdef USE_USART2
    {
        .usart = USART2,
        .rx_dma = { .id = DMA1_Channel6_ID },
        .tx_dma = { .id = DMA1_Channel7_ID },
        .port = GPIOA,
        .tx_pin = 2,
        .rx_pin = 3,
        .rcc_apb_mask = RCC_APB1ENR_USART2EN,
        .rcc_gpio_mask = RCC_AHBENR_GPIOAEN,
        .usart_irq = USART2_IRQn,
        .tx_dma_irq = DMA1_Channel7_IRQn
    },
// #endif
// #ifdef USE_USART3
    {
        .usart = USART3,
        .rx_dma = { .id = DMA1_Channel3_ID },
        .tx_dma = { .id = DMA1_Channel2_ID },
        .port = GPIOA,
        .tx_pin = 2,
        .rx_pin = 3,
        .rcc_apb_mask = RCC_APB1ENR_USART3EN,
        .rcc_gpio_mask = RCC_AHBENR_GPIOBEN,
        .usart_irq = USART3_IRQn,
        .tx_dma_irq = DMA1_Channel2_IRQn
    }
// #endif
};

static UsartInstance_t* registered_instances[3] = { NULL, NULL, NULL };

int usartHwConfigure(UsartInstance_t* instance);
void applyUsartConfig(USART_TypeDef* usart, uint32_t baudrate);
int usartBufferSend(UsartInstance_t* instance);

void usartDmaTxCallback(DmaInstance_t* instance, void* context);

int registerUsartInstance(UsartInstance_t* instance) {
    if (!instance) return -EINVAL;
    UsartId_t usart_id = instance->id;
    if (usart_id < 0 || usart_id >= 3) return -ENODEV;
    if (registered_instances[usart_id] != NULL) return -EPERM;
    instance->_hw = (void*)&USART_STATIC_CONFIG[usart_id];
    registered_instances[usart_id] = instance;
    return usartHwConfigure(instance);
}

int unregisterUsartInstance(UsartInstance_t* instance) {
    if (!instance) return -EINVAL;
    (void)disableUsart(instance);
    for (int i = 0; i < 3; i++) {
        if (registered_instances[i] == instance) {
            registered_instances[i] = NULL;
            instance->_hw = NULL;
            return 0;
        }
    }
    return -ENODEV;
}

int enableUsart(UsartInstance_t* instance) {
    if (!instance) return -EINVAL;
    if (!instance->_hw) return -EPERM;
    UsartStaticConfig_t* config = (UsartStaticConfig_t*)instance->_hw;
    USART_TypeDef* usart = config->usart;
    usart->CR1 |= USART_CR1_UE;
    NVIC_SetPriority(config->usart_irq, 2);
    NVIC_EnableIRQ(config->usart_irq);
    return USART_OK;
}

int disableUsart(UsartInstance_t* instance) {
    if (!instance) return -EINVAL;
    if (!instance->_hw) return -EPERM;
    UsartStaticConfig_t* config = (UsartStaticConfig_t*)instance->_hw;
    USART_TypeDef* usart = config->usart;
    usart->CR1 &= ~USART_CR1_UE;
    NVIC_DisableIRQ(config->usart_irq);
    return USART_OK;
}

int usartHwConfigure(UsartInstance_t* instance) {
    UsartStaticConfig_t* config = (UsartStaticConfig_t*)instance->_hw;
    USART_TypeDef* usart = config->usart;
    config->rx_count = 0;
    // Enable the GPIO port clock
    RCC->AHBENR |= config->rcc_gpio_mask;
    RCC->APB1ENR |= config->rcc_apb_mask;

    applyGPIOConfig(config->port, (GPIOConfig_t){ .pin=config->rx_pin, .af=7 });
    applyGPIOConfig(config->port, (GPIOConfig_t){ .pin=config->tx_pin, .af=7 });

    DmaInstance_t* rx_dma = &config->rx_dma;
    DmaInstance_t* tx_dma = &config->tx_dma;

    rx_dma->buffer = instance->rx_buffer;
    tx_dma->buffer = instance->tx_buffer;

    rx_dma->config = DMA_CCR_MINC | DMA_CCR_CIRC; // memory increment, circular mode
    tx_dma->config = DMA_CCR_MINC | DMA_CCR_DIR; // memory increment, memory to peripheral

    rx_dma->peripheral = (uint32_t)&config->usart->RDR; 
    tx_dma->peripheral = (uint32_t)&config->usart->TDR;
    
    (void)registerDmaInstance(rx_dma);
    (void)registerDmaInstance(tx_dma);
    setDmaTransferCount(tx_dma, 0);

    (void)applyTransferCompleteCallback(tx_dma, usartDmaTxCallback, (void*)instance);

    (void)enableDma(rx_dma);
    (void)enableDmaInterrupt(tx_dma, 0);
    
    applyUsartConfig(usart, instance->baudrate);
    return 0;
}
   
void applyUsartConfig(USART_TypeDef* usart, uint32_t baudrate) {

    // Disable USART before configuration (UE: USART enable bit)
    usart->CR1 &= ~USART_CR1_UE;

    // Set baud rate:
    // BRR = f_ck / baud, where f_ck = PCLK1 = SystemCoreClock / 2
    // For baud = 38400, BRR = (SystemCoreClock / 2) / 38400
    usart->BRR = SystemCoreClock / (2 * baudrate);

    // Configure word length, parity, and oversampling:
    // - OVER8 = 0 → oversampling by 16 (better timing tolerance)
    // - M[1:0] = 00 → 8 data bits
    // - PCE = 0 → no parity bit
    usart->CR1 &= ~(USART_CR1_OVER8 | USART_CR1_M0 | USART_CR1_M1 | USART_CR1_PCE);

    usart->CR1 |= USART_CR1_TE       // Enable transmitter
                | USART_CR1_RE       // Enable receiver
                | USART_CR1_IDLEIE;  // Enable IDLE interrupt

    // Set 1 stop bit (STOP[13:12] = 00)
    usart->CR2 &= ~USART_CR2_STOP;

    // Enable DMA for transmit and receive
    // Also enable IDLE line interrupt (used to detect end of variable-length reception)
    usart->CR3 |= USART_CR3_DMAT     // Enable DMA for TX
                | USART_CR3_DMAR;    // Enable DMA for RX
}

int usartBufferWrite(UsartInstance_t* instance, uint8_t* message, uint16_t length) {
    if (!instance) return -EINVAL;
    if (!instance->_hw) return -EPERM;
    if (!message || length == 0) return -EINVAL; // Invalid parameters
    Buffer* tx_buffer = instance->tx_buffer;
    if (length > tx_buffer->size) return -EFBIG;
    UsartStaticConfig_t* config = (UsartStaticConfig_t*)instance->_hw;
    if (config->tx_busy) return EBUSY; // DMA is currently busy, cannot write

    if (config->tx_count + length > tx_buffer->size) {
        (void)usartBufferSend(instance);
        while (config->tx_busy) {};
    }
    uint8_t* head = (uint8_t*)(tx_buffer->raw + config->tx_count);
    for (uint16_t i = 0; i < length; i++) {
        head[i] = message[i];
    }
    config->tx_count += length;
    return 0;
}

int usartBufferSend(UsartInstance_t* instance) {
    if (!instance) return -EINVAL;
    if (!instance->_hw) return -EPERM;
    UsartStaticConfig_t* config = (UsartStaticConfig_t*)instance->_hw;

    DMA_Channel_TypeDef* channel;
    (void)getDmaChannel(&config->tx_dma, &channel);
    config->tx_busy = true;
    channel->CCR &= ~DMA_CCR_EN;
    channel->CNDTR = config->tx_count;
    channel->CCR |= DMA_CCR_EN;
    return USART_OK;
}

int usartWrite(UsartInstance_t* instance, uint8_t* message, uint16_t length) {
    if (!instance) return -EINVAL;
    if (!instance->_hw) return -EPERM;
    if (!message || length == 0) return -EINVAL;
    UsartStaticConfig_t* config = (UsartStaticConfig_t*)instance->_hw;
    if (config->tx_busy) return EBUSY;
    DMA_Channel_TypeDef* channel;
    (void)getDmaChannel(&config->tx_dma, &channel);
    config->tx_busy = true;
    channel->CCR &= ~DMA_CCR_EN;
    channel->CMAR = (uint32_t)message;
    channel->CNDTR = length;
    channel->CCR |= DMA_CCR_EN;
    return USART_OK;  
}

bool usartTxBusy(UsartInstance_t* instance) {
    if (!instance) return true;
    if (!instance->_hw) return true;
    UsartStaticConfig_t* config = (UsartStaticConfig_t*)instance->_hw;
    return config->tx_busy;
}

int applyIdleRxCallback(UsartInstance_t* instance, UsartCallback callback) {
    if (!instance) return -EINVAL;
    if (!instance->_hw) return -EPERM;
    if (!callback) return -EINVAL;
    UsartStaticConfig_t* config = (UsartStaticConfig_t*)instance->_hw;
    config->rx_callback = callback;
    return USART_OK;
}

void usartDmaTxCallback(DmaInstance_t* instance, void* context) {
    UsartInstance_t* usart_instance = (UsartInstance_t*)context;
    UsartStaticConfig_t* config = (UsartStaticConfig_t*)usart_instance->_hw;
    config->tx_busy = false;
    config->tx_count = 0;
    (void)disableDma(&config->tx_dma);
}

void USART_IRQBase(UsartInstance_t* instance) {
    if (!instance || !instance->_hw) return;
    UsartStaticConfig_t* config = (UsartStaticConfig_t*)instance->_hw;
    USART_TypeDef* usart = config->usart;
    if (usart->ISR & USART_ISR_IDLE) {
        usart->ICR |= USART_ICR_IDLECF;
        DmaInstance_t* rx_dma = &config->rx_dma;
        uint16_t prev_cndtr = config->rx_count;
        uint16_t current_cndtr;
        (void)getDmaTransferCount(rx_dma, &current_cndtr);
        uint16_t buffer_size = instance->rx_buffer->size;
        uint16_t received_now = (prev_cndtr >= current_cndtr) 
                                ? (prev_cndtr - current_cndtr) 
                                : (prev_cndtr + (buffer_size - current_cndtr));
        config->rx_count = current_cndtr;
        __bufferMoveHead(instance->rx_buffer, received_now);
        if (config->rx_callback) config->rx_callback(instance, NULL);
    }
}

void USART1_IRQHandler(void) {
    USART_IRQBase(registered_instances[USART_1_ID]);
}

void USART2_IRQHandler(void) {
    USART_IRQBase(registered_instances[USART_2_ID]);
}

void USART3_IRQHandler(void) {
    USART_IRQBase(registered_instances[USART_3_ID]);
}
