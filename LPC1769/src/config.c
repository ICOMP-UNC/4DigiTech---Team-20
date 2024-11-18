#include "config.h"

void configGPIO(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = PWM_PORT;
    PinCfg.Pinnum = PINSEL_PIN_1;
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&PinCfg);

    GPIO_SetDir(PWM_PORT, PIN_PWM_OUT, OUTPUT);
    GPIO_ClearValue(PWM_PORT, PIN_PWM_OUT);

    GPIO_SetDir(LED_GREEN_PORT, LED_GREEN_PIN, OUTPUT);
    GPIO_ClearValue(LED_GREEN_PORT, LED_GREEN_PIN);

    GPIO_SetDir(LED_RED_PORT, LED_RED_PIN, OUTPUT);
    GPIO_ClearValue(LED_RED_PORT, LED_RED_PIN);
}

void configTimer(void)
{
    TIM_TIMERCFG_Type TimerCfg;
    TIM_MATCHCFG_Type MatchCfg;

    TimerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    TimerCfg.PrescaleValue = PRESCALE_VALUE;
    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &TimerCfg);

    MatchCfg.MatchChannel = MATCH_CHANNEL_0;
    MatchCfg.IntOnMatch = ENABLE;
    MatchCfg.ResetOnMatch = DISABLE;
    MatchCfg.StopOnMatch = DISABLE;
    MatchCfg.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    MatchCfg.MatchValue = (SystemCoreClock / 4) / PWM_FREQUENCY * dutyCycle / 100;
    TIM_ConfigMatch(LPC_TIM0, &MatchCfg);

    MatchCfg.MatchChannel = MATCH_CHANNEL_1;
    MatchCfg.IntOnMatch = ENABLE;
    MatchCfg.ResetOnMatch = ENABLE;
    MatchCfg.MatchValue = (SystemCoreClock / 4) / PWM_FREQUENCY;
    TIM_ConfigMatch(LPC_TIM0, &MatchCfg);

    NVIC_EnableIRQ(TIMER0_IRQn);
    TIM_Cmd(LPC_TIM0, ENABLE);
}

void configUART_DMA(void)
{
    PINSEL_CFG_Type pinConfig;
    pinConfig.Portnum = PINSEL_PORT_0;
    pinConfig.Pinnum = PINSEL_PIN_3;
    pinConfig.Funcnum = PINSEL_FUNC_1;
    pinConfig.Pinmode = PINSEL_PINMODE_PULLUP;
    pinConfig.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&pinConfig);

    UART_CFG_Type UARTConfigStruct;
    UARTConfigStruct.Baud_rate = BAUD_RATE;
    UARTConfigStruct.Parity = UART_PARITY_NONE;
    UARTConfigStruct.Databits = UART_DATABIT_8;
    UARTConfigStruct.Stopbits = UART_STOPBIT_1;

    UART_Init(LPC_UART0, &UARTConfigStruct);
    UART_TxCmd(LPC_UART0, DISABLE);

    GPDMA_Init();
    DMAConfig.ChannelNum = DMA_CHANNEL_0;
    DMAConfig.SrcMemAddr = PERIFERICAL;
    DMAConfig.DstMemAddr = (uint32_t)&receivedData;
    DMAConfig.TransferSize = TRANSFER_SIZE;
    DMAConfig.TransferWidth = GPDMA_WIDTH_BYTE;
    DMAConfig.TransferType = GPDMA_TRANSFERTYPE_P2M;
    DMAConfig.SrcConn = GPDMA_CONN_UART0_Rx;
    DMAConfig.DstConn = NON_USED;
    DMAConfig.DMALLI = NON_USED;
    GPDMA_Setup(&DMAConfig);
    GPDMA_ChannelCmd(DMA_CHANNEL_0, ENABLE);

    NVIC_EnableIRQ(DMA_IRQn);
}

void configADC(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = PINSEL_PORT_0;
    PinCfg.Pinnum = PINSEL_PIN_23;
    PinCfg.Funcnum = PINSEL_FUNC_1;
    PinCfg.Pinmode = PINSEL_PINMODE_NORMAL;

    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, ADC_FREQUENCY);
    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
}

void configDAC(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = PINSEL_PORT_0;
    PinCfg.Pinnum = PINSEL_PIN_26;
    PinCfg.Funcnum = PINSEL_FUNC_2;

    PINSEL_ConfigPin(&PinCfg);

    DAC_Init(LPC_DAC);
}
