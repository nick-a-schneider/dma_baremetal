#include "stm32f303xe.h"
#include "rcc.h"

extern uint32_t SystemCoreClock;

void rccInit(void) {
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
