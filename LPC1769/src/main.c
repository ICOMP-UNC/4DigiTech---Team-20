#include "LPC17xx.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_dac.h"
#include "lpc17xx_gpdma.h" // Incluye el encabezado para DMA
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"

#define PIN_PWM_OUT (1 << 1)                                              // P2.1 para la salida PWM
#define PORT_TWO 2                                                        // Puerto 2
#define PWM_FREQUENCY 100000                                              // Frecuencia de PWM en Hz
#define PRESCALE_VALUE 1                                                  // Valor de preescala para el timer
#define BAUD_RATE 9600                                                    // Baud rate para UART
#define ADC_FREQUENCY 200000                                              // Frecuencia de muestreo del ADC
#define CALCULATE_DAC_VALUE(adcValue) (1023 - ((adcValue * 1023) / 4095)) // Cálculo de valor para el DAC
#define BUFFER_SIZE 32                                                    // Tamaño del buffer para UART
#define START_CHARACTER_1 0x42                                            //"B"
#define START_CHARACTER_2 0x4D                                            //"D"
#define LED_GREEN_PORT 3
#define LED_GREEN_PIN (1 << 25) // P3.25 para el LED verde
#define LED_RED_PORT 0
#define LED_RED_PIN (1 << 22) // P0.22 para el LED rojo

volatile uint8_t receivedData = 0;      // Variable global para el dato recibido por UART mediante DMA
volatile uint32_t dutyCycle = 90;       // Duty cycle inicial
volatile uint16_t adcValue = 0;         // Valor global del ADC
volatile uint16_t dacValue = 0;         // Valor global del DAC
volatile uint8_t enableDACOutput = 0;   // Variable para controlar salida del DAC
GPDMA_Channel_CFG_Type DMAConfig;       // Variable global para la configuración del DMA
volatile uint8_t RxBuffer[BUFFER_SIZE]; // Buffer para almacenar los datos recibidos por UART
volatile uint8_t index = 0;             // Índice para el buffer de recepción
volatile uint8_t flag_1 = 0;            // bandera para inicio de conversion
volatile uint8_t flag_2 = 0;            // bandera inicio de conversion
volatile uint32_t aqi = 0;

void configGPIO(void);
void configTimer(void);
void configUART_DMA(void);
void configADC(void);
void configDAC(void);
void updateADCandDAC(void);
void processDMAReceivedData(void);
uint16_t calculate_checksum(const uint8_t* data, uint16_t length);
int calculate_aqi_from_pm2_5(uint16_t pm2_5);
uint32_t calculateDutyCycle(uint32_t aqi);

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

    // Configuración de P3.25 como salida (LED verde)
    GPIO_SetDir(LED_GREEN_PORT, LED_GREEN_PIN, 1);
    GPIO_ClearValue(LED_GREEN_PORT, LED_GREEN_PIN);

    // Configuración de P0.22 como salida (LED rojo)
    GPIO_SetDir(LED_RED_PORT, LED_RED_PIN, 1);
    GPIO_ClearValue(LED_RED_PORT, LED_RED_PIN);
}

void configTimer(void)
{
    TIM_TIMERCFG_Type TimerCfg;
    TIM_MATCHCFG_Type MatchCfg;

    TimerCfg.PrescaleOption = TIM_PRESCALE_USVAL;
    TimerCfg.PrescaleValue = PRESCALE_VALUE;
    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &TimerCfg);

    MatchCfg.MatchChannel = 0;
    MatchCfg.IntOnMatch = ENABLE;
    MatchCfg.ResetOnMatch = DISABLE;
    MatchCfg.StopOnMatch = DISABLE;
    MatchCfg.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    MatchCfg.MatchValue = (SystemCoreClock / 4) / PWM_FREQUENCY * dutyCycle / 100;
    TIM_ConfigMatch(LPC_TIM0, &MatchCfg);

    MatchCfg.MatchChannel = 1;
    MatchCfg.IntOnMatch = ENABLE;
    MatchCfg.ResetOnMatch = ENABLE;
    MatchCfg.MatchValue = (SystemCoreClock / 4) / PWM_FREQUENCY;
    TIM_ConfigMatch(LPC_TIM0, &MatchCfg);

    NVIC_EnableIRQ(TIMER0_IRQn);
    TIM_Cmd(LPC_TIM0, ENABLE);
}

void configUART_DMA(void)
{
    // Configuración de UART0
    PINSEL_CFG_Type pinConfig;
    pinConfig.Portnum = 0;
    pinConfig.Pinnum = 3;
    pinConfig.Funcnum = 1;
    pinConfig.Pinmode = 0;
    pinConfig.OpenDrain = 0;
    PINSEL_ConfigPin(&pinConfig);

    UART_CFG_Type UARTConfigStruct;
    UARTConfigStruct.Baud_rate = BAUD_RATE;
    UARTConfigStruct.Parity = UART_PARITY_NONE;
    UARTConfigStruct.Databits = UART_DATABIT_8;
    UARTConfigStruct.Stopbits = UART_STOPBIT_1;

    UART_Init(LPC_UART0, &UARTConfigStruct);
    UART_TxCmd(LPC_UART0, DISABLE);

    // Configuración del DMA para UART0 RX
    GPDMA_Init();
    DMAConfig.ChannelNum = 0;
    DMAConfig.SrcMemAddr = 0;
    DMAConfig.DstMemAddr = (uint32_t)&receivedData;
    DMAConfig.TransferSize = 1;
    DMAConfig.TransferWidth = GPDMA_WIDTH_BYTE;
    DMAConfig.TransferType = GPDMA_TRANSFERTYPE_P2M;
    DMAConfig.SrcConn = GPDMA_CONN_UART0_Rx;
    DMAConfig.DstConn = 0;
    DMAConfig.DMALLI = 0;
    GPDMA_Setup(&DMAConfig);
    GPDMA_ChannelCmd(0, ENABLE);

    NVIC_EnableIRQ(DMA_IRQn);
}

void configADC(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 23;
    PinCfg.Funcnum = 1;
    PinCfg.Pinmode = 0;
    PINSEL_ConfigPin(&PinCfg);

    ADC_Init(LPC_ADC, ADC_FREQUENCY);
    ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
}

void configDAC(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 0;
    PinCfg.Pinnum = 26;
    PinCfg.Funcnum = 2;
    PINSEL_ConfigPin(&PinCfg);

    DAC_Init(LPC_DAC);
}

void updateADCandDAC(void)
{
    // Realiza una conversión de ADC
    ADC_StartCmd(LPC_ADC, ADC_START_NOW);
    while (!(ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_0, ADC_DATA_DONE)))
        ;
    adcValue = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);

    // Calcula el valor del DAC en función del ADC
    dacValue = CALCULATE_DAC_VALUE(adcValue);

    // Solo actualiza el DAC si la salida está habilitada
    if (enableDACOutput)
    {
        DAC_UpdateValue(LPC_DAC, dacValue);
    }
}

int calculate_aqi_from_pm2_5(uint16_t pm2_5)
{
    if (pm2_5 <= 120) // Equivalent to 12.0 µg/m³ (scaled by 10)
        return (pm2_5 * 50) / 120;
    else if (pm2_5 <= 354) // Equivalent to 35.4 µg/m³ (scaled by 10)
        return 50 + ((pm2_5 - 120) * 50) / (354 - 120);
    else if (pm2_5 <= 554) // Equivalent to 55.4 µg/m³ (scaled by 10)
        return 100 + ((pm2_5 - 354) * 50) / (554 - 354);
    else if (pm2_5 <= 1504) // Equivalent to 150.4 µg/m³ (scaled by 10)
        return 150 + ((pm2_5 - 554) * 100) / (1504 - 554);
    else if (pm2_5 <= 2504) // Equivalent to 250.4 µg/m³ (scaled by 10)
        return 200 + ((pm2_5 - 1504) * 100) / (2504 - 1504);
    else if (pm2_5 <= 3504) // Equivalent to 350.4 µg/m³ (scaled by 10)
        return 300 + ((pm2_5 - 2504) * 100) / (3504 - 2504);
    else if (pm2_5 <= 5004) // Equivalent to 500.4 µg/m³ (scaled by 10)
        return 400 + ((pm2_5 - 3504) * 100) / (5004 - 3504);
    else
        return 500; // AQI maximum value
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

uint32_t calculateDutyCycle(uint32_t aqi)
{
    if (aqi == 0)
    {
        return 0; // Ventilador apagado
    }
    else if (aqi >= 150)
    {
        return 100; // Ventilador al 100% de duty cycle
    }
    else
    {
        // Escalado lineal entre AQI 1-149 a un duty cycle de 0-100%
        return (aqi * 100) / 150;
    }
}

void processDMAReceivedData(void)
{
    RxBuffer[index] = receivedData;
    if (index == 0 && receivedData != START_CHARACTER_1)
    {
        // Si el primer carácter no es "B", reinicia el índice
        index = 0;
        return;
    }
    else if (index == 1 && receivedData != START_CHARACTER_2)
    {
        // Si el segundo carácter no es "M", reinicia el índice
        index = 0;
        return;
    }
    if ((index + 1) == BUFFER_SIZE)
    {
        uint16_t calculated_checksum = calculate_checksum(RxBuffer, BUFFER_SIZE - 2);
        uint16_t received_checksum = (RxBuffer[BUFFER_SIZE - 2] << 8) | RxBuffer[BUFFER_SIZE - 1];

        if (calculated_checksum == received_checksum)
        {
            GPIO_SetValue(LED_GREEN_PORT, LED_GREEN_PIN); // Enciende el LED verde
            GPIO_ClearValue(LED_RED_PORT, LED_RED_PIN);   // Apaga el LED rojo
            // Extract PM2.5 value from the rx_buffer (assuming it is at index 6 and 7)
            uint16_t pm2_5_value = (RxBuffer[6] << 8) | RxBuffer[7];

            // Scale PM2.5 value by 10 for integer AQI calculation
            pm2_5_value *= 10;

            // Calculate the AQI based on PM2.5 concentration
            aqi = calculate_aqi_from_pm2_5(pm2_5_value);

            uint32_t dutyCycle = calculateDutyCycle(aqi);
            TIM_UpdateMatchValue(LPC_TIM0, 0, (SystemCoreClock / 4) / PWM_FREQUENCY * dutyCycle / 100);
            dutyCycle = 0;
        }
        else
        {
            GPIO_SetValue(LED_RED_PORT, LED_RED_PIN);       // Enciende el LED rojo
            GPIO_ClearValue(LED_GREEN_PORT, LED_GREEN_PIN); // Apaga el LED verde
        }
        flag_1 = 0;
        flag_2 = 0;
    }

    index = (index + 1) % 32;

    GPDMA_ChannelCmd(0, DISABLE);
    GPDMA_Setup(&DMAConfig);
    GPDMA_ChannelCmd(0, ENABLE);
}

void DMA_IRQHandler(void)
{
    if (GPDMA_IntGetStatus(GPDMA_STAT_INTTC, 0))
    {
        GPDMA_ClearIntPending(GPDMA_STATCLR_INTTC, 0);
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

int main(void)
{
    configGPIO();
    configTimer();
    configUART_DMA();
    configADC();
    configDAC();

    while (1)
    {
        updateADCandDAC(); // Actualización continua de ADC y DAC
    }

    return 0;
}
