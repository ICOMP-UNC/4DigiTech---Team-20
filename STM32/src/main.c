#include "FreeRTOS.h"
#include "task.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/adc.h>
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

volatile uint16_t adc_value = 0;
const uint16_t goodValMin = 0;
const uint16_t goodValMax = 680;
const uint16_t moderateValMin = 681;
const uint16_t moderateValMax = 1360;
const uint16_t unSensitiveGrValMin = 1361;
const uint16_t unSensitiveGrValMax = 2040;
const uint16_t unhealthyValMin = 2041;
const uint16_t unhealthyValMax = 2720;
const uint16_t veryUnhealthyValMin = 2721;
const uint16_t veryUnhealthyValMax = 4083;
const uint16_t hazardousValMin = 4084;
const uint16_t hazardousValMax = 4096;

void system_clock_setup(void);
void gpio_setup(void);
void exti_setup(void);
void uart_setup(void);
void uart_send_string(const char* str);
void uart_task(void* args);
void delay_ms(uint32_t ms);
void adc_setup(void);
void adc_task(void* args);

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
    uint8_t frame[32];

    frame[0] = START_CHARACTER_1;
    frame[1] = START_CHARACTER_2;

    frame[2] = (FRAME_LENGTH >> 8) & 0xFF;
    frame[3] = FRAME_LENGTH & 0xFF;

    for (int i = 0; i < 13; i++)
    {
        frame[4 + (i * 2)] = (sensor_data[i] >> 8) & 0xFF;
        frame[5 + (i * 2)] = sensor_data[i] & 0xFF;
    }

    uint16_t checksum = calculate_checksum(frame, 30);
    frame[30] = (checksum >> 8) & 0xFF;
    frame[31] = checksum & 0xFF;

    for (int i = 0; i < 32; i++)
    {
        usart_send_blocking(USART2, frame[i]);
    }
}

void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms * 8000; i++)
    {
        __asm__("nop");
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
    usart_set_mode(USART2, USART_MODE_TX);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    usart_enable(USART2);
}

void uart_send_string(const char* str)
{
    while (*str)
    {
        usart_send_blocking(USART2, *str++);
    }
}

void uart_task(void* args)
{
    (void)args;
    while (1)
    {
        uint16_t pm2_5_value = (adc_value * 300) / 4095; // 12-bit ADC
        uint16_t sensor_data[13] = {
            150,         // Data 1: PM1.0 concentration in µg/m³ (standard particles)
            pm2_5_value, // Data 2: Variable PM2.5 concentration in µg/m³ (scaled from adc_value)
            300,         // Data 3: PM10 concentration in µg/m³ (standard particles)
            160,         // Data 4: PM1.0 concentration in µg/m³ (under atmospheric environment)
            270,         // Data 5: PM2.5 concentration in µg/m³ (under atmospheric environment)
            320,         // Data 6: PM10 concentration in µg/m³ (under atmospheric environment)
            1200,        // Data 7: number of particles with diameter > 0.3 µm in 0.1 L of air
            800,         // Data 8: number of particles with diameter > 0.5 µm in 0.1 L of air
            400,         // Data 9: number of particles with diameter > 1.0 µm in 0.1 L of air
            200,         // Data 10: number of particles with diameter > 2.5 µm in 0.1 L of air
            100,         // Data 11: number of particles with diameter > 5.0 µm in 0.1 L of air
            50,          // Data 12: number of particles with diameter > 10 µm in 0.1 L of air
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
    adc_power_off(ADC1);
    adc_enable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);

    // Configurar el canal 0 (PA0) en lugar de canal 5 (PA5)
    adc_set_sample_time(ADC1, ADC_CHANNEL0, ADC_SMPR_SMP_239DOT5CYC);
    uint8_t channel_array[] = {ADC_CHANNEL0}; // Usar canal 0
    adc_set_regular_sequence(ADC1, 1, channel_array);

    adc_power_on(ADC1);
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);
}

void adc_task(void* args)
{
    (void)args;
    while (1)
    {
        adc_start_conversion_direct(ADC1);
        while (!adc_eoc(ADC1))
            ;
        uint16_t raw_adc = adc_read_regular(ADC1);
        adc_value = raw_adc;
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
