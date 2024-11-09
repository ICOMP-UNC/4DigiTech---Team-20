#include "FreeRTOS.h"
#include "task.h"
<<<<<<< HEAD
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
=======
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
>>>>>>> d1fc3e0 (Initial commit)
#include <stdio.h>

// Define LED parameters
#define LED_PORT GPIOC
#define LED_PIN GPIO13

void system_clock_setup(void);
void gpio_setup(void);
void uart_setup(void);

// Function to set up the system clock
<<<<<<< HEAD
void system_clock_setup(void) {
=======
void system_clock_setup(void)
{
>>>>>>> d1fc3e0 (Initial commit)
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
}

// Function to set up GPIO
<<<<<<< HEAD
void gpio_setup(void) {
=======
void gpio_setup(void)
{
>>>>>>> d1fc3e0 (Initial commit)
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);
}

// Function to set up UART
void uart_setup(void)
{
    // Enable the clock for GPIOA and USART2
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);

    // Configure GPIOA pins for USART2 TX (PA2) and RX (PA3)
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART2_RX);

    // Configure USART2
    usart_set_baudrate(USART2, 9600);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_mode(USART2, USART_MODE_TX_RX); // Enable both transmit and receive
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

    // Enable the USART2 interrupt in the NVIC
    nvic_enable_irq(NVIC_USART2_IRQ);

    USART_CR1(USART2) |= USART_CR1_TCIE;

    // Enable USART2
    usart_enable(USART2);
}

// USART2 Interrupt Service Routine
void usart2_isr(void)
{
    // Check if the Transmission Complete (TC) flag is set
    if (usart_get_flag(USART2, USART_SR_TC))
    {
        // Optional: Toggle an LED to indicate the interrupt occurred
        gpio_toggle(GPIOC, GPIO13);
    }
}

// Task to blink the LED with a 200 ms delay
<<<<<<< HEAD
void led_task_200ms(void *args) {
    (void)args;
    while (1) {
=======
void led_task_200ms(void* args)
{
    (void)args;
    while (1)
    {
>>>>>>> d1fc3e0 (Initial commit)
        gpio_clear(LED_PORT, LED_PIN);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set(LED_PORT, LED_PIN);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

// Function to send a string over UART
void uart_send_string(const char* str)
{
    while (*str)
    {
        usart_send_blocking(USART2, *str++); // Send each character
    }
}

// Task to send a message over UART every 1 second
<<<<<<< HEAD
void uart_task(void *args) {
    (void)args;
    while (1) {
=======
void uart_task(void* args)
{
    (void)args;
    while (1)
    {
>>>>>>> d1fc3e0 (Initial commit)
        uart_send_string("1057478");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

<<<<<<< HEAD
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    while (1);
}

// Main function
int main(void) {
=======
void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    while (1)
        ;
}

// Main function
int main(void)
{
>>>>>>> d1fc3e0 (Initial commit)
    system_clock_setup();
    gpio_setup();
    uart_setup();

    xTaskCreate(led_task_200ms, "LED Task", 100, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(uart_task, "UART Task", 100, NULL, configMAX_PRIORITIES - 1, NULL);

    vTaskStartScheduler();

<<<<<<< HEAD
    while (1);
=======
    while (1)
        ;
>>>>>>> d1fc3e0 (Initial commit)
    return 0;
}
