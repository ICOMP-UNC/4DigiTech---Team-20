#include "isr.h"

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
