/**
 * @file config.h
 * @brief Configuration functions for system clock, GPIO, UART, and ADC.
 *
 * This header file contains the declarations of functions used to configure
 * the system clock, GPIO, UART, and ADC peripherals on an STM32 microcontroller.
 * These functions are essential for setting up the hardware and enabling
 * communication and data acquisition.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "global.h" /**< Global definitions and declarations */

/**
 * @brief Configures the system clock and enables peripheral clocks.
 *
 * This function sets up the system clock using the PLL (Phase-Locked Loop)
 * configuration with an external high-speed oscillator (HSE) of 8 MHz to achieve
 * a system clock frequency of 72 MHz. It also enables the clock for the
 * Alternate Function I/O (AFIO) peripheral.
 */
void system_clock_setup(void);

/**
 * @brief Configures the GPIO settings.
 *
 * This function enables the clock for GPIO port A and sets the mode of GPIO pin 0
 * to analog input.
 */
void gpio_setup(void);

/**
 * @brief Setup and initialize the UART peripheral.
 *
 * This function configures the UART peripheral (USART2) with the following settings:
 * - Enables the clock for GPIOA and USART2 peripherals.
 * - Configures GPIOA pins for USART2 TX (transmit) and RX (receive) functionality.
 * - Sets the baud rate to 9600.
 * - Configures the data bits to 8.
 * - Sets the stop bits to 1.
 * - Configures the UART mode to transmit and receive.
 * - Disables parity.
 * - Disables flow control.
 * - Enables the USART2 peripheral.
 * - Enables interrupt for USART2 reception.
 * - Enables the NVIC interrupt for USART2.
 */
void uart_setup(void);

/**
 * @brief Setup and initialize the ADC and DMA for single conversion mode.
 *
 * This function configures the ADC1 peripheral and DMA1 for single conversion mode.
 * It enables the necessary clocks, configures the ADC for single conversion mode,
 * sets the sample time, and configures the DMA to transfer the ADC conversion result
 * to a memory location.
 *
 * Steps performed:
 * 1. Enable clocks for ADC1 and DMA1.
 * 2. Power off the ADC and configure it for single conversion mode with scan mode enabled.
 * 3. Set the sample time for ADC channel 0.
 * 4. Configure the regular sequence for ADC with channel 0.
 * 5. Enable DMA for ADC and power on the ADC.
 * 6. Reset and calibrate the ADC.
 * 7. Configure the DMA channel 1 for ADC data transfer.
 * 8. Set the peripheral and memory addresses for DMA.
 * 9. Set the number of data items to transfer and configure DMA settings.
 * 10. Enable DMA interrupts for transfer complete and transfer error.
 * 11. Enable the DMA channel and start the ADC conversion.
 */
void adc_setup(void);

#endif // CONFIG_H
