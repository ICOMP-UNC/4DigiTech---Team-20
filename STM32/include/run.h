/**
 * @file tasks.h
 * @brief Task functions for UART, ADC, and frame processing.
 *
 * This header file contains the declarations of the FreeRTOS tasks used for UART communication,
 * ADC data acquisition, and frame processing. It also includes the stack overflow hook function.
 */

#ifndef RUN_H
#define RUN_H

#include "frame.h"  /**< Frame processing functions */
#include "global.h" /**< Global definitions and declarations */
#include "isr.h"    /**< ISR function declarations */

/**
 * @brief UART task that reads sensor data, processes it, and sends it over UART.
 *
 * This task runs in an infinite loop, reading ADC values, calculating PM2.5 concentration,
 * and populating a sensor data array with various particulate matter concentrations and
 * particle counts. The data is then sent using the `send_data_frame` function. The task
 * also monitors its stack usage and prints the high water mark for debugging purposes.
 *
 * @param pvParameters Pointer to task arguments (unused).
 */
void uart_task(void* pvParameters);

/**
 * @brief Task to handle ADC operations.
 *
 * This task continuously handles DMA ADC transfers and monitors the stack high water mark.
 * It prints the stack high water mark to the console for debugging purposes.
 *
 * @param pvParameters Pointer to the arguments passed to the task (unused).
 */
void adc_task(void* pvParameters);

/**
 * @brief Stack overflow hook function.
 *
 * This function is called when a stack overflow is detected in a FreeRTOS task.
 * It enters an infinite loop to halt the system, allowing for debugging.
 *
 * @param xTask Handle of the task that caused the stack overflow.
 * @param pcTaskName Name of the task that caused the stack overflow.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName);

#endif // RUN_H
