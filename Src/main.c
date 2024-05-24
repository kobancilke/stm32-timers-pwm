/**
 * Project template
 * RT & Embedded Systems course material
 *
 * Prepared by: Jakub Mnich (jakub.mnich@pwr.edu.pl)
 * Wrocław University of Science and Technology
 * March 2022
 *
 * Works with STM32F407VGT6 (STM32F407G-DISC1)
 ******************************************************************************
 */

/*
 * Note:
 * "stm32xxxxxx.h" are header files delivered by the MCU manufacturer. These files
 * contain information allowing us to refer to registers by their names rather than
 * their actual addresses in the memory. This makes it much easier to write the
 * code and it will also be more readable.
 */
#include "stm32f407xx.h"

void SystemClock_Config(void);
void TIM4_PWM_Config(void);
void GPIO_Config(void);
void delay(uint32_t delay);

int main(void)
{
	 // System Clock Configuration
	    SystemClock_Config();

	    // GPIO Configuration for LEDs
	    GPIO_Config();

	    // Timer Configuration for PWM
	    TIM4_PWM_Config();

	    const float pi = 3.14159265359;
	    float phase = 0.0;
	    float increment = 0.005; // Start frequency in Hz

	while(1) {
		// Calculate duty cycle with a sine wave
		float value = (sin(phase) + 1) * 499.5; // Generates a value between 0 and 999
		uint32_t duty_cycle = (uint32_t)value;

	    // Apply duty cycle to all channels
		TIM4->CCR1 = duty_cycle;
		TIM4->CCR2 = duty_cycle;
		TIM4->CCR3 = duty_cycle;
		TIM4->CCR4 = duty_cycle;

		// Increase phase
		phase += increment;

		// Adjust frequency based on the sine wave
		float frequency = 0.1 + 1.9 * (sin(phase * 0.05) + 1) / 2; // Changes between 0.1 Hz and 2 Hz
		uint32_t period = 1000000 / (frequency * 1000); // Calculate period in microseconds

		delay(period); // Adjust delay according to the period

		if (phase >= 2 * pi) { // Reset phase if it exceeds 2π
			phase -= 2 * pi;
		}
	}
}

void SystemClock_Config(void) {
    // Enable HSE
	RCC->CR |= 0b1 << 16; // HSEON
	while (!(RCC->CR & (0b1 << 17))); // Wait for HSE Ready

    // Flash latency
    FLASH->ACR |= 0b0101; // 5 wait states

    // Configure PLL
    RCC->PLLCFGR = (0b1 << 22) | (168 << 6) | (8 << 0) | (0b00 << 16) | (7 << 24);


    // Enable PLL
    RCC->CR |= 0b1 << 24; // PLLON
    while (!(RCC->CR & (0b1 << 25))); // Wait for PLL Ready

    // AHB, APB1 and APB2 prescalers
    RCC->CFGR |= (0b0000 << 4);  // AHB prescaler = 1
    RCC->CFGR |= (0b101 << 10);  // APB1 prescaler = 4
    RCC->CFGR |= (0b100 << 13);  // APB2 prescaler = 2

    // Set PLL as system clock source
    RCC->CFGR |= 0b10 << 0;
    while ((RCC->CFGR & (0b11 << 2)) != (0b10 << 2));
}

void GPIO_Config(void) {
	// Enable GPIOG clock
	RCC->AHB1ENR |= (1 << 3);  // Enable the clock for GPIOD

    GPIOD->MODER &= ~((0b11 << (12 * 2)) | (0b11 << (13 * 2)) | (0b11 << (14 * 2)) | (0b11 << (15 * 2)));
	GPIOD->MODER |= (0b10 << (12 * 2)) | (0b10 << (13 * 2)) | (0b10 << (14 * 2)) | (0b10 << (15 * 2));
	GPIOD->AFR[1] |= (2 << ((12 - 8) * 4)) | (2 << ((13 - 8) * 4)) | (2 << ((14 - 8) * 4)) | (2 << ((15 - 8) * 4));
	}

void TIM4_PWM_Config(void) {
	// Enable TIM4 clock
	RCC->APB1ENR |= 0b1 << 2; // TIM4EN

	// Set prescaler value to get 1 kHz frequency
	TIM4->PSC = 84 - 1;
	TIM4->ARR = 1000 - 1;

    // Configure TIM4 in PWM mode
	TIM4->CCMR1 |= 0b110 << 4 | 0b110 << 12; // PWM mode 1 for channel 1 and 2
	TIM4->CCMR2 |= 0b110 << 4 | 0b110 << 12; // PWM mode 1 for channel 3 and 4
	TIM4->CCER |= 0b1 << 0 | 0b1 << 4 | 0b1 << 8 | 0b1 << 12; // Enable channels
	TIM4->CR1 |= 0b1 << 7; // ARPE
	TIM4->EGR |= 0b1 << 0; // UG

	// Start TIM4 counter
    TIM4->CR1 |= 0b1 << 0; // CEN
}

void delay(uint32_t delay) {

	while (delay--) {
	        __NOP();
    }
}







