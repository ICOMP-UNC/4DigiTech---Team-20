#include "isr.h"

void dma1_channel1_isr(void)
{
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL1, DMA_TCIF))
    {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_TCIF);
        dma_transfer_completed = true;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(dmaSemaphore, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
        dma_disable_channel(DMA1, DMA_CHANNEL1);
        dma_set_number_of_data(DMA1, DMA_CHANNEL1, 1);
        dma_enable_channel(DMA1, DMA_CHANNEL1);
        adc_start_conversion_direct(ADC1);
        dma_transfer_completed = false;
    }
}
