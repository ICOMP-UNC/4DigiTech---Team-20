/**
 * @file global.h
 * @brief Global definitions and declarations.
 *
 * This header file contains global definitions, declarations, and includes necessary
 * headers for the STM32 microcontroller project. It defines constants, buffer sizes,
 * and external variable declarations used across multiple source files.
 */

#ifndef GLOBAL_H
#define GLOBAL_H

#include "FreeRTOS.h"               /**< FreeRTOS header for real-time operating system */
#include "semphr.h"                 /**< Header for using FreeRTOS semaphores */
#include "task.h"                   /**< Header for creating and managing FreeRTOS tasks */
#include <libopencm3/cm3/nvic.h>    /**< NVIC configuration header for handling interrupts */
#include <libopencm3/stm32/adc.h>   /**< ADC peripheral configuration header */
#include <libopencm3/stm32/dma.h>   /**< DMA peripheral configuration header */
#include <libopencm3/stm32/exti.h>  /**< External interrupt configuration header */
#include <libopencm3/stm32/gpio.h>  /**< GPIO configuration and control header */
#include <libopencm3/stm32/rcc.h>   /**< Reset and Clock Control configuration header */
#include <libopencm3/stm32/usart.h> /**< USART peripheral configuration header */
#include <stdint.h>                 /**< Standard integer type definitions */
#include <stdio.h>                  /**< Standard Input/Output functions */

#define LED_PORT GPIOC         /**< Port where the potentiometer is connected */
#define LED_PIN GPIO14         /**< Pin number where the potentiometer is connected */
#define START_CHARACTER_1 0x42 /**< First start character (0x42 in hex) */
#define START_CHARACTER_2 0x4D /**< Second start character (0x4D in hex) */
#define FRAME_LENGTH 28        /**< Length of the data frame in bytes */
#define UART_TX_BUFFER_SIZE 32 /**< Size of the UART transmission buffer */
#define ADC_BUFFER_SIZE 10     /**< Size of the ADC buffer for averaging */

extern volatile uint16_t adc_value;                          /**< ADC value acquired via DMA */
extern volatile uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE]; /**< Buffer for UART transmission data */
extern volatile bool dma_transfer_completed;                 /**< Flag indicating completion of a DMA transfer */
extern TaskHandle_t uartTaskHandle;                          /**< Handle for the UART task */
extern TaskHandle_t adcTaskHandle;                           /**< Handle for the ADC task */
extern SemaphoreHandle_t dmaSemaphore;                       /**< Semaphore for DMA transfer synchronization */

#endif // GLOBAL_H
