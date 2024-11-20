#include "isr.h"

void DMA_IRQHandler(void)
{
    if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, DMA_CHANNEL_0))
    {
        GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, DMA_CHANNEL_0);
        processDMAReceivedData();
    }
}

void TIMER0_IRQHandler(void)
{
    if (TIM_GetIntStatus(LPC_TIM0, TIM_MR0_INT))
    {
        GPIO_SetValue(PORT_TWO, PIN_PWM_OUT);
        TIM_ClearIntPending(LPC_TIM0, TIM_MR0_INT);
    }
    if (TIM_GetIntStatus(LPC_TIM0, TIM_MR1_INT))
    {
        GPIO_ClearValue(PORT_TWO, PIN_PWM_OUT);
        TIM_ClearIntPending(LPC_TIM0, TIM_MR1_INT);
    }
}
