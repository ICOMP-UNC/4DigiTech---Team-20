#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_gpdma.h" // Include the DMA header
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"

#define PIN_PWM_OUT (1 << 1) // P2.1 para la salida PWM
#define PORT_TWO 2           // Puerto 2
#define PWM_FREQUENCY 100000 // Frecuencia de PWM en Hz
#define PRESCALE_VALUE 1     // Valor de preescala para el timer
#define MATCH_CHANNEL_0 0    // Canal de match 0
#define MATCH_CHANNEL_1 1    // Canal de match 1
#define MATCH_VALUE_0 (SystemCoreClock / 4) / PWM_FREQUENCY* dutyCycle / 100
#define MATCH_VALUE_1 (SystemCoreClock / 4) / PWM_FREQUENCY
#define BAUD_RATE 9600                                                    // Baud rate para UART
#define ADC_FREQUENCY 200000                                              // Frecuencia de muestreo del ADC
#define CALCULATE_DAC_VALUE(adcValue) (1023 - ((adcValue * 1023) / 4095)) // Cálculo de valor para el DAC
#define DMA_CHANNEL_0 0                                                   // Canal 0 para DMA
#define PERIFERICAL 0                                                     // Transferencia de periférico a memoria
#define DMA_BUFFER_SIZE 10                                                // Tamaño del buffer de DMA

volatile uint8_t receivedData = 0;    // Variable global para el dato recibido por UART
volatile uint32_t dutyCycle = 90;     // Duty cycle inicial
volatile uint16_t adcValue = 0;       // Valor global del ADC
volatile uint16_t dacValue = 0;       // Valor global del DAC
volatile uint8_t enableDACOutput = 0; // Variable para controlar salida del DAC
volatile uint32_t data_Rx[DMA_BUFFER_SIZE];

void configGPIO(void);
void configTimer(void);
void configUART(void);
void configADC(void);
void configDAC(void);
void configDMA(void);

/**
 * @brief Configures the DMA (Direct Memory Access) for UART0 reception.
 *
 * This function initializes the GPDMA (General Purpose DMA) and sets up the DMA channel
 * for transferring data from the UART0 peripheral to a memory buffer. It configures the
 * DMA channel with the specified parameters and enables the DMA interrupt.
 *
 */
void configDMA()
{
    GPDMA_Init();
    GPDMA_Channel_CFG_Type GPDMACfg;
    GPDMACfg.ChannelNum = DMA_CHANNEL_0;
    GPDMACfg.SrcMemAddr = PERIFERICAL;
    GPDMACfg.DstMemAddr = (uint32_t)data_Rx;
    GPDMACfg.TransferSize = DMA_BUFFER_SIZE;
    GPDMACfg.TransferWidth = GPDMA_WIDTH_WORD;
    GPDMACfg.TransferType = GPDMA_TRANSFERTYPE_P2M;
    GPDMACfg.SrcConn = GPDMA_CONN_UART0_Rx;
    GPDMACfg.DstConn = 0;
    GPDMACfg.DMALLI = 0;
    GPDMA_Setup(&GPDMACfg);
    GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, 0);
    GPDMA_ChannelCmd(0, ENABLE);

    NVIC_EnableIRQ(DMA_IRQn);
}

/**
 * @brief Configures the GPIO settings for the specified pins.
 *
 * This function sets up the GPIO configuration for a specific pin on port 2.
 * After configuring the pin, it sets the direction of the pin to output and
 * clears the value of the pin.
 */
void configGPIO(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = PINSEL_PORT_2;
    PinCfg.Pinnum = PINSEL_PIN_1;
    PinCfg.Funcnum = PINSEL_FUNC_0;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
    PINSEL_ConfigPin(&PinCfg);

    GPIO_SetDir(PORT_TWO, PIN_PWM_OUT, 1);
    GPIO_ClearValue(PORT_TWO, PIN_PWM_OUT);
}

/**
 * @brief Configures the timer for PWM generation.
 *
 * This function sets up the timer with the specified configurations for PWM generation.
 * It initializes the timer with a prescale value, configures match channels for PWM duty cycle
 * and frequency, and enables the timer interrupt. Additionally, it activates the timer.
 */
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
    MatchCfg.MatchValue = MATCH_VALUE_0;
    TIM_ConfigMatch(LPC_TIM0, &MatchCfg);

    MatchCfg.MatchChannel = MATCH_CHANNEL_1;
    MatchCfg.IntOnMatch = ENABLE;
    MatchCfg.ResetOnMatch = ENABLE;
    MatchCfg.MatchValue = MATCH_VALUE_1;
    TIM_ConfigMatch(LPC_TIM0, &MatchCfg);

    NVIC_EnableIRQ(TIMER0_IRQn);
    TIM_Cmd(LPC_TIM0, ENABLE);
}

/**
 * @brief Configures the UART0 peripheral with specified settings.
 *
 * This function configures the UART0 peripheral on the LPC1769 microcontroller.
 * It sets up the pin configuration for UART0, initializes the UART0 peripheral
 * with a baud rate of 9600, no parity, 8 data bits, and 1 stop bit. Additionally,
 * it enables the receive interrupt for UART0 and activates the UART0 interrupt
 * in the NVIC.
 */
void configUART(void)
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

    UART_FIFO_CFG_Type UARTFIFOConfigStruct;
    UARTFIFOConfigStruct.FIFO_DMAMode = ENABLE; // Enable DMA mode for FIFO
    UARTFIFOConfigStruct.FIFO_Level = UART_FIFO_TRGLEV0;
    UARTFIFOConfigStruct.FIFO_ResetRxBuf = ENABLE;
    UARTFIFOConfigStruct.FIFO_ResetTxBuf = ENABLE;

    UART_ConfigStructInit(&UARTConfigStruct); // Initialize UART with default settings
    UART_Init(LPC_UART0, &UARTConfigStruct);
    UART_FIFOConfig(LPC_UART0, &UARTFIFOConfigStruct); // Configure FIFO
    UART_TxCmd(LPC_UART0, DISABLE);

    // Habilita la interrupción de recepción en UART
    UART_IntConfig(LPC_UART0, UART_INTCFG_RBR, ENABLE);
    NVIC_EnableIRQ(UART0_IRQn); // Activa la interrupción de UART0 en el NVIC
}

void UART0_IRQHandler(void)
{
    if (UART_GetIntId(LPC_UART0) & UART_IIR_INTID_RDA)
    {
        receivedData = UART_ReceiveByte(LPC_UART0);
        if (receivedData >= '1')
        {
            dutyCycle = 10;
            enableDACOutput = 1;
        }
        else
        {
            dutyCycle = 90;
            enableDACOutput = 0;
        }
        TIM_UpdateMatchValue(LPC_TIM0, 0, MATCH_VALUE_0);
    }
}

void configADC(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = PINSEL_PORT_0;
    PinCfg.Pinnum = PINSEL_PIN_23;
    PinCfg.Funcnum = PINSEL_FUNC_1;
    PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
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

void updateADCandDAC(void)
{
    // Inicia y espera la conversión del ADC
    ADC_StartCmd(LPC_ADC, ADC_START_NOW);
    while (!(ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_0, ADC_DATA_DONE)))
        ;
    adcValue = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);

    // Invierte el valor del ADC para el DAC: ADC alto -> DAC bajo, ADC bajo -> DAC alto
    dacValue = CALCULATE_DAC_VALUE(adcValue);

    // Actualiza el DAC con el valor invertido si está habilitado
    if (enableDACOutput)
    {
        DAC_UpdateValue(LPC_DAC, dacValue);
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

void DMA_IRQHandler()
{
    if (GPDMA_IntGetStatus(GPDMA_STAT_INT, 0))
    {
        GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, 0); // Clear interrupt on transfer complete
    }
    else if (GPDMA_IntGetStatus(GPDMA_STAT_INTERR, 0) || GPDMA_IntGetStatus(GPDMA_STAT_INTERR, 1))
    {
        {
            while (1)
                ; // Stay in an infinite loop on error
        }
    }
}

int main(void)
{
    configGPIO();
    configTimer();
    configUART();
    configADC();
    configDAC();
    configDMA();
    while (1)
    {
        // Actualiza continuamente el ADC y el DAC
        updateADCandDAC();
    }

    return 0;
}
