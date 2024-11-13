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

#include "FreeRTOS.h"               /**< FreeRTOS header for real-time operating system */
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

#define LED_PORT GPIOC         /**< Port where the potenciometer is connected */
#define LED_PIN GPIO14         /**< Pin number where the potenciometer is connected */
#define START_CHARACTER_1 0x42 /**< First start character (0x42 in hex) */
#define START_CHARACTER_2 0x4D /**< Second start character (0x4D in hex) */
#define FRAME_LENGTH 28        /**< Length of the data frame in bytes */
#define UART_TX_BUFFER_SIZE 32 /**< Size of the UART transmission buffer */
#define RX_BUFFER_SIZE 32      /**< Size of the receive buffer */

volatile uint8_t rx_buffer[RX_BUFFER_SIZE];           /**< Buffer to store received UART data */
volatile uint8_t rx_index = 0;                        /**< Index for the UART receive buffer */
volatile uint16_t adc_value = 0;                      /**< ADC value acquired via DMA */
volatile uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE]; /**< Buffer for UART transmission data */
volatile bool dma_transfer_completed = false;         /**< Flag indicating completion of a DMA transfer */
volatile uint8_t received_data = 0;                   /**< Last received byte from UART */
volatile bool frame_received = false;          /**< Flag indicating that a complete data frame has been received */
int aqi = 0;                                   /**< Air Quality Index (AQI) value calculated from PM2.5 data */
TaskHandle_t uartTaskHandle = NULL;            /**< Handle for the UART task */
TaskHandle_t adcTaskHandle = NULL;             /**< Handle for the ADC task */
TaskHandle_t frameProcessingTaskHandle = NULL; /**< Handle for the frame processing task */

void system_clock_setup(void);
void gpio_setup(void);
void uart_setup(void);
void uart_task(void* args);
void adc_setup(void);
void adc_task(void* args);
uint16_t calculate_checksum(const uint8_t* data, uint16_t length);
void send_data_frame(uint16_t* sensor_data);
void handle_dma_adc_transfer(void);
void process_received_frame(void);
void frame_processing_task(void* args);
int calculate_aqi_from_pm2_5(uint16_t pm2_5);

/**
 * @brief Configures the system clock and enables peripheral clocks.
 *
 * This function sets up the system clock using the PLL (Phase-Locked Loop)
 * configuration with an external high-speed oscillator (HSE) of 8 MHz to achieve
 * a system clock frequency of 72 MHz. It also enables the clock for the
 * Alternate Function I/O (AFIO) peripheral.
 */
void system_clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
    rcc_periph_clock_enable(RCC_AFIO);
}

/**
 * @brief Configures the GPIO settings.
 *
 * This function enables the clock for GPIO port A and sets the mode of GPIO pin 0
 * to analog input.
 */
void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);
}

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
void uart_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

    usart_set_baudrate(USART2, 9600);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    usart_enable(USART2);
    usart_enable_rx_interrupt(USART2);
    nvic_enable_irq(NVIC_USART2_IRQ);
}

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
void adc_setup(void)
{
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_DMA1);

    adc_power_off(ADC1);
    adc_enable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);

    adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_239DOT5CYC);
    uint8_t channel_array[] = {ADC_CHANNEL0};
    adc_set_regular_sequence(ADC1, 1, channel_array);

    adc_enable_dma(ADC1);
    adc_power_on(ADC1);
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);

    dma_channel_reset(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC_DR(ADC1));
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)&adc_value);
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, 1);
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);

    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_32BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_32BIT);

    dma_disable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_set_priority(DMA1, DMA_CHANNEL1, DMA_CCR_PL_HIGH);

    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
    dma_enable_transfer_error_interrupt(DMA1, DMA_CHANNEL1);
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);

    dma_enable_channel(DMA1, DMA_CHANNEL1);
    adc_start_conversion_direct(ADC1);
}

/**
 * @brief Processes the received frame and calculates the AQI from PM2.5 value.
 *
 * This function checks if a frame has been received. If a frame is received, it calculates the checksum
 * of the received data and compares it with the received checksum. If the checksums match, it extracts
 * the PM2.5 value from the received data, scales it, and calculates the AQI (Air Quality Index) from the
 * PM2.5 value.
 *
 */
void process_received_frame(void)
{
    if (frame_received)
    {
        frame_received = false;

        uint16_t calculated_checksum = calculate_checksum((const uint8_t*)rx_buffer, UART_TX_BUFFER_SIZE - 2);
        uint16_t received_checksum = (rx_buffer[UART_TX_BUFFER_SIZE - 2] << 8) | rx_buffer[UART_TX_BUFFER_SIZE - 1];

        if (calculated_checksum == received_checksum)
        {
            uint16_t pm2_5_value = (rx_buffer[6] << 8) | rx_buffer[7];
            pm2_5_value *= 10;
            aqi = calculate_aqi_from_pm2_5(pm2_5_value);
        }
    }
}

/**
 * @brief Calculates the checksum of the given data.
 *
 * This function computes a simple checksum by summing up all the bytes
 * in the provided data array.
 *
 * @param data Pointer to the data array.
 * @param length Length of the data array.
 * @return The calculated checksum.
 */
uint16_t calculate_checksum(const uint8_t* data, uint16_t length)
{
    uint16_t checksum = 0;
    for (uint16_t i = 0; i < length; i++)
    {
        checksum += data[i];
    }
    return checksum;
}

/**
 * @brief Sends a data frame containing sensor data over USART.
 *
 * This function constructs a data frame with a start sequence, frame length,
 * sensor data, and a checksum, and sends it over USART2.
 *
 * @param sensor_data Pointer to an array of 13 uint16_t sensor data values.
 *
 * The data frame format is as follows:
 * - Byte 0: START_CHARACTER_1
 * - Byte 1: START_CHARACTER_2
 * - Byte 2: High byte of frame length
 * - Byte 3: Low byte of frame length
 * - Bytes 4-29: Sensor data (each sensor value is 2 bytes, high byte first)
 * - Byte 30: High byte of checksum
 * - Byte 31: Low byte of checksum
 *
 * The checksum is calculated over the first 30 bytes of the frame.
 */
void send_data_frame(uint16_t* sensor_data)
{
    uart_tx_buffer[0] = START_CHARACTER_1;
    uart_tx_buffer[1] = START_CHARACTER_2;

    uart_tx_buffer[2] = (FRAME_LENGTH >> 8) & 0xFF;
    uart_tx_buffer[3] = FRAME_LENGTH & 0xFF;

    for (int i = 0; i < 13; i++)
    {
        uart_tx_buffer[4 + (i * 2)] = (sensor_data[i] >> 8) & 0xFF;
        uart_tx_buffer[5 + (i * 2)] = sensor_data[i] & 0xFF;
    }

    uint16_t checksum = calculate_checksum((const uint8_t*)uart_tx_buffer, 30);
    uart_tx_buffer[30] = (checksum >> 8) & 0xFF;
    uart_tx_buffer[31] = checksum & 0xFF;

    for (int i = 0; i < UART_TX_BUFFER_SIZE; i++)
    {
        usart_send_blocking(USART2, uart_tx_buffer[i]);
    }
}

/**
 * @brief Calculate the Air Quality Index (AQI) from PM2.5 concentration.
 *
 * This function converts the PM2.5 concentration (in µg/m³, scaled by 10) to the corresponding AQI value.
 * The AQI is calculated based on the breakpoints defined by the U.S. Environmental Protection Agency (EPA).
 *
 * @param pm2_5 The PM2.5 concentration in µg/m³, scaled by 10 (e.g., 120 represents 12.0 µg/m³).
 * @return The corresponding AQI value.
 *
 * The AQI is calculated using the following breakpoints:
 * - 0 to 12.0 µg/m³ (scaled by 10): AQI 0 to 50
 * - 12.1 to 35.4 µg/m³ (scaled by 10): AQI 51 to 100
 * - 35.5 to 55.4 µg/m³ (scaled by 10): AQI 101 to 150
 * - 55.5 to 150.4 µg/m³ (scaled by 10): AQI 151 to 200
 * - 150.5 to 250.4 µg/m³ (scaled by 10): AQI 201 to 300
 * - 250.5 to 350.4 µg/m³ (scaled by 10): AQI 301 to 400
 * - 350.5 to 500.4 µg/m³ (scaled by 10): AQI 401 to 500
 * - Above 500.4 µg/m³ (scaled by 10): AQI 500 (maximum value)
 */
int calculate_aqi_from_pm2_5(uint16_t pm2_5)
{
    if (pm2_5 <= 120)
        return (pm2_5 * 50) / 120;
    else if (pm2_5 <= 354)
        return 50 + ((pm2_5 - 120) * 50) / (354 - 120);
    else if (pm2_5 <= 554)
        return 100 + ((pm2_5 - 354) * 50) / (554 - 354);
    else if (pm2_5 <= 1504)
        return 150 + ((pm2_5 - 554) * 100) / (1504 - 554);
    else if (pm2_5 <= 2504)
        return 200 + ((pm2_5 - 1504) * 100) / (2504 - 1504);
    else if (pm2_5 <= 3504)
        return 300 + ((pm2_5 - 2504) * 100) / (3504 - 2504);
    else if (pm2_5 <= 5004)
        return 400 + ((pm2_5 - 3504) * 100) / (5004 - 3504);
    else
        return 500;
}

/**
 * @brief USART2 interrupt service routine.
 *
 * This ISR is triggered when data is received on USART2. It reads the received
 * data and processes it according to a predefined protocol. The protocol expects
 * the first two characters to be specific start characters. If the received data
 * does not match these start characters, the reception is reset. The received data
 * is stored in a buffer, and when the buffer is full, a flag is set to indicate
 * that a complete frame has been received.
 */
void usart2_isr(void)
{
    received_data = usart_recv(USART2);

    if (rx_index == 0 && received_data != START_CHARACTER_1)
    {
        rx_index = 0;
        return;
    }
    else if (rx_index == 1 && received_data != START_CHARACTER_2)
    {
        rx_index = 0;
        return;
    }
    rx_buffer[rx_index++] = received_data;
    if (rx_index >= UART_TX_BUFFER_SIZE)
    {
        frame_received = true;
        rx_index = 0;
    }
}

/**
 * @brief DMA1 Channel 1 Interrupt Service Routine.
 *
 * This ISR handles the DMA1 Channel 1 interrupts. It checks for the transfer complete
 * and transfer error interrupt flags, clears them, and sets the dma_transfer_completed
 * flag if the transfer is complete.
 */
void dma1_channel1_isr(void)
{
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF))
    {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
        dma_transfer_completed = true;
    }

    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TEIF))
    {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TEIF);
    }
}

/**
 * @brief Handles the completion of a DMA ADC transfer.
 *
 * This function is called when a DMA transfer is completed. It resets the DMA
 * channel, re-enables it, and starts a new ADC conversion.
 *
 * The function performs the following steps:
 * 1. Checks if the DMA transfer is completed.
 * 2. Resets the `dma_transfer_completed` flag.
 * 3. Disables the DMA channel.
 * 4. Sets the number of data to be transferred by the DMA channel to 1.
 * 5. Re-enables the DMA channel.
 * 6. Starts a new ADC conversion.
 */
void handle_dma_adc_transfer(void)
{
    if (dma_transfer_completed)
    {
        dma_transfer_completed = false;

        dma_disable_channel(DMA1, DMA_CHANNEL1);
        dma_set_number_of_data(DMA1, DMA_CHANNEL1, 1);
        dma_enable_channel(DMA1, DMA_CHANNEL1);

        adc_start_conversion_direct(ADC1);
    }
}

/**
 * @brief Task for processing received frames.
 *
 * This task continuously processes received frames in an infinite loop.
 * It also monitors the stack high water mark for the task and prints it
 * to the console.
 *
 * @param args Pointer to the arguments passed to the task (unused).
 */
void frame_processing_task(void* args)
{
    (void)args;
    while (1)
    {
        process_received_frame();
        UBaseType_t frameHighWaterMark = uxTaskGetStackHighWaterMark(frameProcessingTaskHandle);
        printf("Frame Processing Task Stack High Water Mark: %lu\n", frameHighWaterMark);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief UART task that reads sensor data, processes it, and sends it over UART.
 *
 * This task runs in an infinite loop, reading ADC values, calculating PM2.5 concentration,
 * and populating a sensor data array with various particulate matter concentrations and
 * particle counts. The data is then sent using the `send_data_frame` function. The task
 * also monitors its stack usage and prints the high water mark for debugging purposes.
 *
 * @param args Pointer to task arguments (unused).
 */
void uart_task(void* args)
{
    (void)args;
    while (1)
    {

        uint16_t pm2_5_value = (adc_value * 300) / 4095; // 12-bit ADC
        uint16_t sensor_data[13] = {
            150,         // Data 1: PM1.0 concentration in µg/m³ (standard particles) (Frame 4 and 5)
            pm2_5_value, // Data 2: Variable PM2.5 concentration in µg/m³ (scaled from adc_value) (Frame 6 and 7)
            300,         // Data 3: PM10 concentration in µg/m³ (standard particles) (Frame 8 and 9)
            160,         // Data 4: PM1.0 concentration in µg/m³ (under atmospheric environment) (Frame 10 and 11)
            270,         // Data 5: PM2.5 concentration in µg/m³ (under atmospheric environment) (Frame 12 and 13)
            320,         // Data 6: PM10 concentration in µg/m³ (under atmospheric environment) (Frame 14 and 15)
            1200,        // Data 7: number of particles with diameter > 0.3 µm in 0.1 L of air (Frame 16 and 17)
            800,         // Data 8: number of particles with diameter > 0.5 µm in 0.1 L of air (Frame 18 and 19)
            400,         // Data 9: number of particles with diameter > 1.0 µm in 0.1 L of air (Frame 20 and 21)
            200,         // Data 10: number of particles with diameter > 2.5 µm in 0.1 L of air (Frame 22 and 23)
            100,         // Data 11: number of particles with diameter > 5.0 µm in 0.1 L of air (Frame 24 and 25)
            50,          // Data 12: number of particles with diameter > 10 µm in 0.1 L of air (Frame 26 and 27)
            0            // Reserved
        };
        send_data_frame(sensor_data);
        UBaseType_t uartHighWaterMark = uxTaskGetStackHighWaterMark(uartTaskHandle);
        printf("UART Task Stack High Water Mark: %lu\n", uartHighWaterMark);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/**
 * @brief Task to handle ADC operations.
 *
 * This task continuously handles DMA ADC transfers and monitors the stack high water mark.
 * It prints the stack high water mark to the console for debugging purposes.
 *
 * @param args Pointer to the arguments passed to the task (unused).
 */
void adc_task(void* args)
{
    (void)args;
    while (1)
    {
        handle_dma_adc_transfer();
        UBaseType_t adcHighWaterMark = uxTaskGetStackHighWaterMark(adcTaskHandle);
        printf("ADC Task Stack High Water Mark: %lu\n", adcHighWaterMark);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/**
 * @brief Stack overflow hook function.
 *
 * This function is called when a stack overflow is detected in a FreeRTOS task.
 * It enters an infinite loop to halt the system, allowing for debugging.
 *
 * @param xTask Handle of the task that caused the stack overflow.
 * @param pcTaskName Name of the task that caused the stack overflow.
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    while (1)
        ;
}

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
