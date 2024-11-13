#include "frame.h" 

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
}

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
