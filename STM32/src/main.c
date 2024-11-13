#include "FreeRTOS.h"
#include "task.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <stdint.h>
#include <stdio.h>

#define LED_PORT GPIOC
#define LED_PIN GPIO14
#define BUTTON_PORT GPIOB
#define BUTTON_PIN GPIO0
#define START_CHARACTER_1 0x42
#define START_CHARACTER_2 0x4D
#define FRAME_LENGTH 28 // Frame length for 2x13 + 2 bytes
#define UART_TX_BUFFER_SIZE 32
#define RX_BUFFER_SIZE 32
uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint8_t rx_index = 0;

volatile uint16_t adc_value = 0;
uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
volatile bool dma_transfer_completed = false;
volatile bool uart_dma_transfer_completed = false;
uint8_t received_data = 0;

void system_clock_setup(void);
void gpio_setup(void);
void uart_setup(void);
void uart_task(void* args);
void adc_setup(void);
void adc_task(void* args);
uint16_t calculate_checksum(const uint8_t* data, uint16_t length);
void send_data_frame(uint16_t* sensor_data);
void handle_dma_adc_transfer(void);
void handle_uart_dma_transfer(void);

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
    // Fill the uart_tx_buffer with the frame data
    uart_tx_buffer[0] = START_CHARACTER_1;
    uart_tx_buffer[1] = START_CHARACTER_2;

    uart_tx_buffer[2] = (FRAME_LENGTH >> 8) & 0xFF;
    uart_tx_buffer[3] = FRAME_LENGTH & 0xFF;

    for (int i = 0; i < 13; i++)
    {
        uart_tx_buffer[4 + (i * 2)] = (sensor_data[i] >> 8) & 0xFF;
        uart_tx_buffer[5 + (i * 2)] = sensor_data[i] & 0xFF;
    }

    uint16_t checksum = calculate_checksum(uart_tx_buffer, 30);
    uart_tx_buffer[30] = (checksum >> 8) & 0xFF;
    uart_tx_buffer[31] = checksum & 0xFF;

    for (int i = 0; i < UART_TX_BUFFER_SIZE; i++)
    {
        usart_send_blocking(USART2, uart_tx_buffer[i]);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void system_clock_setup(void)
{
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
    rcc_periph_clock_enable(RCC_AFIO);
}

void gpio_setup(void)
{
    // Configurar PA0 como entrada analógica (en lugar de PA5)
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0); // Cambiado a GPIO0 (PA0)
}

void uart_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

    usart_set_baudrate(USART2, 9600);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX); // Enable both TX and RX
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    usart_enable(USART2);

    // Configure DMA for USART2 TX
    dma_channel_reset(DMA1, DMA_CHANNEL7); // Reset DMA channel for USART2 TX
    dma_set_peripheral_address(DMA1, DMA_CHANNEL7, (uint32_t)&USART2_DR);
    dma_set_memory_address(DMA1, DMA_CHANNEL7, (uint32_t)uart_tx_buffer);
    dma_set_number_of_data(DMA1, DMA_CHANNEL7, 1);
    dma_set_read_from_memory(DMA1, DMA_CHANNEL7);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL7);
    dma_set_priority(DMA1, DMA_CHANNEL7, DMA_CCR_PL_HIGH);

    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL7); // Enable transfer complete interrupt
    dma_enable_transfer_error_interrupt(DMA1, DMA_CHANNEL7);    // Enable transfer error interrupt
    nvic_enable_irq(NVIC_DMA1_CHANNEL7_IRQ);                    // Enable the DMA interrupt in the NVIC

    dma_enable_channel(DMA1, DMA_CHANNEL7);

    // Enable DMA for USART2 TX
    usart_enable_tx_dma(USART2);
    usart_enable_rx_interrupt(USART2); // Enable RX interrupt
    nvic_enable_irq(NVIC_USART2_IRQ);
}

void usart2_isr(void)
{
    // Read the data, which clears the RXNE flag
    rx_buffer[rx_index++] = usart_recv(USART2);

    // Wrap around if buffer size is exceeded
    if (rx_index >= RX_BUFFER_SIZE)
    {
        rx_index = 0;
    }
}

void handle_uart_dma_transfer(void)
{
    if (uart_dma_transfer_completed)
    {
        uart_dma_transfer_completed = false; // Reset the flag

        // Reconfigure the DMA for the next transfer
        dma_disable_channel(DMA1, DMA_CHANNEL7);
        dma_set_number_of_data(DMA1, DMA_CHANNEL7, 1);
        dma_enable_channel(DMA1, DMA_CHANNEL7); // Re-enable the DMA channel
    }
}

void dma1_channel7_isr(void)
{
    // Check if the transfer complete interrupt flag is set
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL7, DMA_TCIF))
    {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL7, DMA_TCIF); // Clear the transfer complete flag
        uart_dma_transfer_completed = true;                      // Set the flag
    }

    // Check if the transfer error interrupt flag is set
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL7, DMA_TEIF))
    {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL7, DMA_TEIF); // Clear the transfer error flag
    }
}

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
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    while (1)
        ;
}

void adc_setup(void)
{
    rcc_periph_clock_enable(RCC_ADC1);
    rcc_periph_clock_enable(RCC_DMA1); // Enable clock for DMA1

    adc_power_off(ADC1);
    adc_enable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);

    // Configure ADC channel 0 (PA0)
    adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_239DOT5CYC);
    uint8_t channel_array[] = {ADC_CHANNEL0};
    adc_set_regular_sequence(ADC1, 1, channel_array);

    adc_enable_dma(ADC1); // Enable DMA for ADC
    adc_power_on(ADC1);
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);

    // Configure DMA
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

    // Enable DMA transfer complete and error interrupts for ADC
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
    dma_enable_transfer_error_interrupt(DMA1, DMA_CHANNEL1);
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ); // Enable the DMA1 Channel 1 interrupt in the NVIC

    dma_enable_channel(DMA1, DMA_CHANNEL1);
    adc_start_conversion_direct(ADC1);
}

void dma1_channel1_isr(void)
{
    // Check if the transfer complete interrupt flag is set
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF))
    {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF); // Clear the transfer complete flag
        dma_transfer_completed = true;                           // Set the flag
    }

    // Check if the transfer error interrupt flag is set
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TEIF))
    {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TEIF); // Clear the transfer error flag
    }
}

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

void adc_task(void* args)
{
    (void)args;
    while (1)
    {
        handle_dma_adc_transfer();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main(void)
{
    system_clock_setup();
    gpio_setup();
    uart_setup();
    adc_setup();

    xTaskCreate(uart_task, "UART Task", 128, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(adc_task, "ADC Task", 128, NULL, configMAX_PRIORITIES - 2, NULL);
    vTaskStartScheduler();
    while (1)
    {
    }
}
