#include "buffer.h"
#include "allocator.h"
#include "stm32f303xe.h"

#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"

#define RX_BUFFER_SIZE 16
#define AF7_USART      0b0111

extern uint32_t SystemCoreClock;


uint8_t rx_buffer[RX_BUFFER_SIZE];

// uint8_t tx_buffer[RX_BUFFER_SIZE];
uint8_t* tx_buffer = NULL;
uint8_t raw[1024];
Allocator allocator;

Buffer rx_dma_buffer = CREATE_BUFFER(.raw=rx_buffer, .size=(RX_BUFFER_SIZE-1));

static volatile uint16_t prev_cndtr = RX_BUFFER_SIZE;
bool tx_dma_busy = false;


void printUart2(void);
void clearTx(void);

void USART2_IRQHandler(void) {
    // Check if IDLE line is detected on USART2 (signals end of reception)
    if (USART2->ISR & USART_ISR_IDLE) {
        // Clear the IDLE flag by writing to the interrupt clear register
        USART2->ICR |= USART_ICR_IDLECF;

        // Read the current number of data items left to transfer in DMA channel 6 (USART2_RX)
        uint16_t current_cndtr = DMA1_Channel6->CNDTR;
        uint16_t received_now = (prev_cndtr >= current_cndtr) 
                              ? (prev_cndtr - current_cndtr) 
                              : (prev_cndtr + (RX_BUFFER_SIZE - current_cndtr));

        prev_cndtr = current_cndtr;
        __bufferMoveHead(&rx_dma_buffer, received_now);
        printUart2();
    }
}

void DMA1_Channel7_IRQHandler(void) {
    // Check if Transfer Complete flag is set
    if (DMA1->ISR & DMA_ISR_TCIF7) {
        tx_dma_busy = false;

        // Clear the Transfer Complete interrupt flag by writing 1 to the bit
        DMA1->IFCR |= DMA_IFCR_CTCIF7;

        // Disable DMA Channel 7 to stop any further transfers until re-enabled
        DMA1_Channel7->CCR &= ~DMA_CCR_EN;
    }
}


void RCCConfig(void) {
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;

  // Disable PLL before configuring (PLL must be off before changing its settings)
  RCC->CR &= ~RCC_CR_PLLON;
  while (RCC->CR & RCC_CR_PLLRDY);  // Wait until PLL is fully stopped

  // Enable High-Speed Internal oscillator (HSI = 8 MHz), used as PLL input
  RCC->CR |= RCC_CR_HSION;
  while (!(RCC->CR & RCC_CR_HSIRDY));  // Wait until HSI is ready

  // Configure PLL input and multiplier:
  // - Use HSI/2 as PLL source (via PREDIV)
  // - Multiply by 9 to get 8 MHz / 2 * 9 = 36 MHz PLL output
  // - Set APB1 prescaler to divide by 2, so APB1 = 36 MHz (max for many peripherals)
  RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_PPRE1);
  RCC->CFGR |=   RCC_CFGR_PLLSRC_HSI_PREDIV   // Select HSI/2 as PLL input
             |   RCC_CFGR_PLLMUL9              // Multiply by 9
             |   RCC_CFGR_PPRE1_DIV2;          // APB1 = SYSCLK / 2

  // Set Flash latency to 2 wait states (required for 48+ MHz system clock)
  FLASH->ACR &= ~FLASH_ACR_LATENCY;
  FLASH->ACR |= FLASH_ACR_LATENCY_2;

  // Enable PLL with the new configuration
  RCC->CR |= RCC_CR_PLLON;
  while (!(RCC->CR & RCC_CR_PLLRDY));  // Wait until PLL is locked

  // Select PLL as system clock source
  RCC->CFGR &= ~RCC_CFGR_SW;
  RCC->CFGR |= RCC_CFGR_SW_PLL;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);  // Wait until switch is complete

  // Manually update the SystemCoreClock variable to match the new clock (72 MHz)
  SystemCoreClock = 72000000;
}


void GPIOConfig(void) {
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  // Configure PA2 and PA3 as Alternate Function mode (10b)
  // MODER2 and MODER3 are 2-bit fields: 00=input, 01=output, 10=alt func, 11=analog
  GPIOA->MODER &= ~(GPIO_MODER_MODER2_Msk | GPIO_MODER_MODER3_Msk);        // Clear mode bits
  GPIOA->MODER |=  (GPIO_MODER_MODER2_1 | GPIO_MODER_MODER3_1);            // Set mode to AF (bit1 = 1, bit0 = 0)

  // Set PA2 and PA3 to use Alternate Function 7 (AF7 = USART TX/RX)
  GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL2_Msk | GPIO_AFRL_AFRL3_Msk);           // Clear AF selection for PA2/PA3
  GPIOA->AFR[0] |=  (AF7_USART << GPIO_AFRL_AFRL2_Pos)                     // Set AF7 for PA2 (USART2_TX)
                 |  (AF7_USART << GPIO_AFRL_AFRL3_Pos);                    // Set AF7 for PA3 (USART2_RX)

  // Set output speed for PA2 and PA3 to high speed (11b)
  // High speed is recommended for USART pins for better signal integrity at high baud rates
  GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR2 | GPIO_OSPEEDER_OSPEEDR3);
}


void DMAConfig(void) {
  // Enable clock for DMA1 peripheral (required before configuring any DMA channels)
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

  /*** DMA1 Channel 6 — USART2_RX configuration ***/

  // Disable DMA channel before making changes
  DMA1_Channel6->CCR &= ~DMA_CCR_EN;

  // Enable memory increment mode and circular mode:
  // - MINC: automatically increment memory address after each transfer
  // - CIRC: when CNDTR reaches 0, restart transfer automatically from the beginning
  DMA1_Channel6->CCR |= DMA_CCR_MINC | DMA_CCR_CIRC;

  // Set number of bytes to transfer (size of RX buffer)
  DMA1_Channel6->CNDTR = RX_BUFFER_SIZE;

  // Set peripheral address to USART2 receive data register (RDR)
  DMA1_Channel6->CPAR = (uint32_t)&USART2->RDR;

  // Set memory address to receive buffer in RAM
  DMA1_Channel6->CMAR = (uint32_t)rx_buffer;

  // Enable the DMA channel (starts reception)
  DMA1_Channel6->CCR |= DMA_CCR_EN;

  /*** DMA1 Channel 7 — USART2_TX configuration ***/

  // Disable DMA channel before making changes
  DMA1_Channel7->CCR &= ~DMA_CCR_EN;

  // Enable memory increment mode and set transfer direction:
  // - MINC: memory pointer auto-increment
  // - DIR: read from memory, write to peripheral (i.e., memory-to-peripheral)
  DMA1_Channel7->CCR |= DMA_CCR_MINC | DMA_CCR_DIR;

  // Set peripheral address to USART2 transmit data register (TDR)
  DMA1_Channel7->CPAR = (uint32_t)&USART2->TDR;

  // Set memory address to the transmit buffer in RAM (length will be set before each TX)
  DMA1_Channel7->CMAR = (uint32_t)tx_buffer;

  // Enable transfer complete interrupt (used to detect when TX is done)
  DMA1_Channel7->CCR |= DMA_CCR_TCIE;

  // Enable DMA1 Channel 7 interrupt in NVIC (triggers ISR on transfer complete)
  NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}


void USART2Config(void) {
  // Enable clock for USART2 peripheral
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

  // Disable USART before configuration (UE: USART enable bit)
  USART2->CR1 &= ~USART_CR1_UE;

  // Set baud rate:
  // BRR = f_ck / baud, where f_ck = PCLK1 = SystemCoreClock / 2
  // For baud = 38400, BRR = (SystemCoreClock / 2) / 38400
  USART2->BRR = SystemCoreClock / (2 * 38400);

  // Configure word length, parity, and oversampling:
  // - OVER8 = 0 → oversampling by 16 (better timing tolerance)
  // - M[1:0] = 00 → 8 data bits
  // - PCE = 0 → no parity bit
  USART2->CR1 &= ~(USART_CR1_OVER8 | USART_CR1_M0 | USART_CR1_M1 | USART_CR1_PCE);

  USART2->CR1 |= USART_CR1_TE       // Enable transmitter
               | USART_CR1_RE       // Enable receiver
               | USART_CR1_IDLEIE;  // Enable IDLE interrupt

  // Set 1 stop bit (STOP[13:12] = 00)
  USART2->CR2 &= ~USART_CR2_STOP;

  // Enable DMA for transmit and receive
  // Also enable IDLE line interrupt (used to detect end of variable-length reception)
  USART2->CR3 |= USART_CR3_DMAT     // Enable DMA for TX
               | USART_CR3_DMAR;    // Enable DMA for RX

  // Enable USART peripheral
  USART2->CR1 |= USART_CR1_UE;

  // Enable USART2 global interrupt in NVIC
  NVIC_EnableIRQ(USART2_IRQn);
}

int main(void) {
  initAllocator(&allocator, RX_BUFFER_SIZE, raw, 1024);
  tx_buffer = (uint8_t*)allocate(&allocator, RX_BUFFER_SIZE);

  RCCConfig();

  GPIOConfig();

  DMAConfig();

  USART2Config();

  clearTx(); // 
  while (1) {}
}

void clearTx(void) {
    tx_dma_busy = false;
    DMA1_Channel7->CCR &= ~DMA_CCR_EN;
}

void printUart2(void) {
  if (tx_dma_busy) return;
  uint16_t len = 0;
  while (bufferRead(&rx_dma_buffer, &tx_buffer[len++]) != false)
  tx_dma_busy = true;
  // Disable DMA channel 7 before updating transfer settings
  // This ensures no transfer is active while we change buffer size and pointers
  DMA1_Channel7->CCR &= ~DMA_CCR_EN;

  // Set the number of data items to transfer in this DMA transaction
  // 'len' should be the size of the data buffer to send over USART2 TX
  DMA1_Channel7->CNDTR = len;

  // Re-enable DMA channel 7 to start the transfer with the updated settings
  DMA1_Channel7->CCR |= DMA_CCR_EN;
}

