/**
 * @file main.c
 * @brief UART and ADC Configuration and Data Transmission for STM32 Microcontroller.
 *
 * This program is designed to configure UART, ADC, and DMA peripherals on an STM32 microcontroller,
 * enabling real-time sensor data simulation and transmission using FreeRTOS tasks.
 * The main features include:
 * - ADC setup for syncronized sampling with DMA
 * - UART communication for data transmission and reception
 * - Data frame construction for sensor data and checksum validation
 * - Calculation of Air Quality Index (AQI) from PM2.5 sensor readings
 *
 * @note This code utilizes FreeRTOS for task management and libopencm3 for hardware abstraction.
 *
 * @details The system clock is configured to run at 72 MHz using an external 8 MHz oscillator.
 * GPIO pins are set up for analog input and UART functionality. The ADC is configured for single conversion
 * mode with DMA, while UART is used for data transmission and frame reception. The main function initializes
 * all peripherals and creates FreeRTOS tasks to handle ADC sampling, UART communication, and frame processing.
 * The system runs indefinitely, processing and transmitting sensor data.
 *
 * @authors
 * @date 13-11-2024
 */

#include "config.h" /**< Configuration functions for system clock, GPIO, UART, and ADC */
#include "global.h" /**< Global definitions and declarations */
#include "tasks.h"  /**< Task functions for UART, ADC, and frame processing */

/**
 * @brief Main entry point of the application.
 *
 * This function sets up the system clock, GPIO, UART, and ADC peripherals.
 * It then creates three FreeRTOS tasks:
 * - UART Task: Handles UART communication.
 * - ADC Task: Handles ADC data acquisition.
 * - Frame Processing Task: Processes frames of data.
 *
 * After creating the tasks, it starts the FreeRTOS scheduler.
 * The main loop is left empty as the tasks are managed by the scheduler.
 */
int main(void)
{
    system_clock_setup();
    gpio_setup();
    uart_setup();
    adc_setup();
    xTaskCreate(uart_task, "UART Task", 128, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(adc_task, "ADC Task", 128, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(frame_processing_task, "Frame Processing Task", 128, NULL, configMAX_PRIORITIES - 3, NULL);
    vTaskStartScheduler();
    while (1)
    {
    }
}
