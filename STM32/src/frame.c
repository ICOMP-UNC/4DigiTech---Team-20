#include "frame.h"

uint16_t calculate_checksum(const uint8_t* data, uint16_t length)
{
    uint16_t checksum = 0;
    for (uint16_t i = 0; i < length; i++)
    {
        checksum += data[i];
    }
    return checksum;
}

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
    gpio_toggle(GPIOC, GPIO13);
}
