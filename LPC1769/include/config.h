#ifndef CONFIG_H
#define CONFIG_H

#include "global.h"

/**
 * @brief Configures the GPIO pins for the application.
 *
 * This function sets up the GPIO pins with the following configurations:
 * - P2.1 as a general-purpose output pin with pull-up resistor and normal mode.
 * - P3.25 as an output pin for the green LED and clears its value.
 * - P0.22 as an output pin for the red LED and clears its value.
 */
void configGPIO(void);

/**
 * @brief Configures the timer for PWM generation.
 *
 * This function sets up the timer with the specified prescale value and match configurations
 * for PWM generation. It initializes the timer, configures two match channels, and enables
 * the timer interrupt.
 * The function performs the following steps:
 * 1. Initializes the timer with the specified prescale value.
 * 2. Configures match channel 0 with the specified duty cycle.
 * 3. Configures match channel 1 with the specified PWM frequency.
 * 4. Enables the timer interrupt.
 * 5. Starts the timer.
 */
void configTimer(void);

/**
 * @brief Configures UART0 and sets up DMA for UART0 RX.
 *
 * This function configures the UART0 peripheral with specified settings
 * and initializes the DMA for receiving data from UART0. It sets up the
 * pin configuration, UART parameters, and DMA channel for UART0 RX.
 *
 * UART0 Configuration:
 * - Port: 0
 * - Pin: 3
 * - Function: 1 (UART0)
 * - Pin mode: 0 (default)
 * - Open drain: 0 (disabled)
 *
 * UART Parameters:
 * - Baud rate: Defined by BAUD_RATE
 * - Parity: None
 * - Data bits: 8
 * - Stop bits: 1
 *
 * DMA Configuration:
 * - Channel: 0
 * - Source memory address: 0 (UART0 RX)
 * - Destination memory address: Address of receivedData variable
 * - Transfer size: 1 byte
 * - Transfer width: Byte
 * - Transfer type: Peripheral to memory (P2M)
 * - Source connection: UART0 RX
 * - Destination connection: None
 * - Linked list item: None
 *
 * The function also enables the DMA interrupt in the NVIC.
 */
void configUART_DMA(void);

/**
 * @brief Configures the ADC (Analog-to-Digital Converter) peripheral.
 *
 * This function sets up the pin configuration for the ADC channel and initializes
 * the ADC peripheral with the specified frequency. It also enables the specified
 * ADC channel for conversion.
 *
 * Pin configuration:
 * - Port: 0
 * - Pin: 23
 * - Function: 1 (ADC function)
 * - Pin mode: 0 (default mode)
 *
 * ADC configuration:
 * - Frequency: Defined by ADC_FREQUENCY
 * - Channel: 0 (ADC_CHANNEL_0)
 */
void configADC(void);

/**
 * @brief Configures the Digital-to-Analog Converter (DAC) on the LPC1769.
 *
 * This function sets up the pin configuration for the DAC on pin P0.26 and
 * initializes the DAC peripheral.
 *
 * The pin configuration is set as follows:
 * - Port number: 0
 * - Pin number: 26
 * - Function number: 2 (DAC function)
 *
 * After configuring the pin, the DAC peripheral is initialized using the
 * DAC_Init function.
 */
void configDAC(void);

#endif // CONFIG_H
