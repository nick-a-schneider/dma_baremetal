
#include "stm32f303xe.h"
#include "bsp.h"
#include "usart.h"
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
    bool tx_busy;
    uint16_t rx_limit;
    UsartCallback rx_callback;
    UsartCallback tx_callback;
    void* rx_callback_context;
    void* tx_callback_context;
} UsartStaticConfig_t;

static UsartStaticConfig_t USART_STATIC_CONFIG[BSP_USART_COUNT] = { BSP_USART_CHANNELS };

static UsartInstance_t* registered_instances[BSP_USART_COUNT] = { NULL };

int usartHwConfigure(UsartInstance_t* instance);
void applyUsartConfig(USART_TypeDef* usart, uint32_t baudrate);
int usartBufferSend(UsartInstance_t* instance);

void usartDmaTxCallback(DmaInstance_t* instance, void* context);

int registerUsartInstance(UsartInstance_t* instance) {
    if (!instance) return -EINVAL;
    UsartId_t usart_id = instance->id;
    if (usart_id < 0 || usart_id >= BSP_USART_COUNT) return -ENODEV;
    if (registered_instances[usart_id] != NULL) return -EPERM;
    instance->_hw = (void*)&USART_STATIC_CONFIG[usart_id];
    registered_instances[usart_id] = instance;
    return usartHwConfigure(instance);
}

int unregisterUsartInstance(UsartInstance_t* instance) {
    if (!instance) return -EINVAL;
    (void)disableUsart(instance);
    for (int i = 0; i < BSP_USART_COUNT; i++) {
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
    // Enable the GPIO port clock
    RCC->AHBENR |= config->rcc_gpio_mask;
    RCC->APB1ENR |= config->rcc_apb_mask;

    applyGPIOConfig(config->port, (GPIOConfig_t){ .pin=config->rx_pin, .af=7 });
    applyGPIOConfig(config->port, (GPIOConfig_t){ .pin=config->tx_pin, .af=7 });

    DmaInstance_t* rx_dma = &config->rx_dma;
    DmaInstance_t* tx_dma = &config->tx_dma;

    rx_dma->config = DMA_CCR_MINC; // | DMA_CCR_CIRC; // memory increment, circular mode
    tx_dma->config = DMA_CCR_MINC | DMA_CCR_DIR; // memory increment, memory to peripheral


    (void)registerDmaInstance(rx_dma);
    (void)registerDmaInstance(tx_dma);

    (void)configurePeripheralAddress(rx_dma, (uint32_t*)&usart->RDR);
    (void)configurePeripheralAddress(tx_dma, (uint32_t*)&usart->TDR);

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

int usartWrite(UsartInstance_t* instance, uint32_t* message, uint16_t length) {
    if (!instance) return -EINVAL;
    if (!instance->_hw) return -EPERM;
    if (!message || length == 0) return -EINVAL; // Invalid parameters
    UsartStaticConfig_t* config = (UsartStaticConfig_t*)instance->_hw;
    if (config->tx_busy) return -EBUSY; // DMA is currently busy, cannot write
    int res = configureMemoryAddress(&config->tx_dma, message, length);
    if (res < 0) return res; 
    config->tx_busy = true;
    return enableDma(&config->tx_dma);
}

int usartSetReadAddress(UsartInstance_t* instance, uint32_t* address, uint16_t length) {
    if (!instance) return -EINVAL;
    if (!instance->_hw) return -EPERM;
    if (!address || length == 0) return -EINVAL; // Invalid parameters
    UsartStaticConfig_t* config = (UsartStaticConfig_t*)instance->_hw;
    config->rx_limit = length;
    int res =  configureMemoryAddress(&config->rx_dma, address, length);
    if (res < 0) return res; 
    return enableDma(&config->rx_dma);
}

int applyRxIdleCallback(UsartInstance_t* instance, UsartCallback callback, void* context) {
    if (!instance) return -EINVAL;
    if (!instance->_hw) return -EPERM;
    if (!callback) return -EINVAL;
    UsartStaticConfig_t* config = (UsartStaticConfig_t*)instance->_hw;
    config->rx_callback = callback;
    config->rx_callback_context = context;
    return USART_OK;
}

int applyTxCompleteCallback(UsartInstance_t* instance, UsartCallback callback, void* context) {
    if (!instance) return -EINVAL;
    if (!instance->_hw) return -EPERM;
    if (!callback) return -EINVAL;
    UsartStaticConfig_t* config = (UsartStaticConfig_t*)instance->_hw;
    config->tx_callback = callback;
    config->tx_callback_context = context;
    return USART_OK;
}

void usartDmaTxCallback(DmaInstance_t* instance, void* context) {
    UsartInstance_t* usart_instance = (UsartInstance_t*)context;
    UsartStaticConfig_t* config = (UsartStaticConfig_t*)usart_instance->_hw;
    config->tx_busy = false;
    (void)disableDma(&config->tx_dma);
    if (config->tx_callback) config->tx_callback(usart_instance, config->tx_callback_context);
}

void USART_IRQBase(UsartInstance_t* instance) {
    if (!instance || !instance->_hw) return;
    UsartStaticConfig_t* config = (UsartStaticConfig_t*)instance->_hw;
    USART_TypeDef* usart = config->usart;
    if (usart->ISR & USART_ISR_IDLE) {
        usart->ICR |= USART_ICR_IDLECF;
        DmaInstance_t* rx_dma = &config->rx_dma;
        uint16_t current_cndtr;
        (void)getDmaTransferCount(rx_dma, &current_cndtr);
        instance->rx_len = config->rx_limit - current_cndtr;
        if (!instance->rx_len) return; // random isr trigger, ignore
        if (config->rx_callback) config->rx_callback(instance, config->rx_callback_context);
    }
}

BSP_USART_IRQS(USART_IRQBase, registered_instances);