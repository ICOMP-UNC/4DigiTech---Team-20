#include "LPC17xx.h"        /**< Main header file for LPC17xx microcontroller definitions */
#include "lpc17xx_adc.h"    /**< Header file for ADC configuration and control */
#include "lpc17xx_dac.h"    /**< Header file for DAC configuration and control */
#include "lpc17xx_gpdma.h"  /**< Header file for DMA configuration and handling */
#include "lpc17xx_gpio.h"   /**< Header file for GPIO configuration and control */
#include "lpc17xx_pinsel.h" /**< Header file for pin selection and configuration */
#include "lpc17xx_timer.h"  /**< Header file for timer configuration and management */
#include "lpc17xx_uart.h"   /**< Header file for UART configuration and communication */

#define PIN_PWM_OUT (1 << 1) /**< P2.1 configured for PWM output */
#define PORT_TWO 2           /**< Port 2 identifier */
#define PWM_FREQUENCY 100000 /**< Frequency of the PWM signal in Hz */
#define PRESCALE_VALUE 1     /**< Timer prescale value for microsecond precision */
#define BAUD_RATE 9600       /**< Baud rate for UART communication */
#define ADC_FREQUENCY 200000 /**< Sampling frequency for the ADC in Hz */
#define CALCULATE_DAC_VALUE(adcValue)                                                                                  \
    (1023 - ((adcValue * 1023) / 4095)) /**< Macro to calculate the DAC value from ADC reading */
#define BUFFER_SIZE 32                  /**< Buffer size for UART communication */
#define START_CHARACTER_1 0x42          /**< ASCII character 'B' (Start character 1 for data frame) */
#define START_CHARACTER_2 0x4D          /**< ASCII character 'M' (Start character 2 for data frame) */
#define LED_GREEN_PORT 3                /**< Port number for the green LED */
#define LED_GREEN_PIN (1 << 25)         /**< Pin number (P3.25) for the green LED */
#define LED_RED_PORT 0                  /**< Port number for the red LED */
#define LED_RED_PIN (1 << 22)           /**< Pin number (P0.22) for the red LED */
#define PWM_PORT 2                      /**< Port number for PWM output */

#define DMA_CHANNEL_0 0 /**< Identifier for DMA channel 0 */
#define PERIFERICAL 0   /**< Placeholder for peripheral configuration */
#define TRANSFER_SIZE 1 /**< Transfer size for DMA operations */
#define NON_USED 0      /**< Placeholder for unused configurations */
#define INPUT 0         /**< Macro for setting pin direction to input */
#define OUTPUT 1        /**< Macro for setting pin direction to output */

#define MATCH_CHANNEL_0 0 /**< Identifier for match channel 0 (timer configuration) */
#define MATCH_CHANNEL_1 1 /**< Identifier for match channel 1 (timer configuration) */

#define PM2_5_BREAKPOINT_12_0 120   /**< PM2.5 concentration breakpoint: 12.0 µg/m³ (scaled by 10) */
#define PM2_5_BREAKPOINT_35_4 354   /**< PM2.5 concentration breakpoint: 35.4 µg/m³ (scaled by 10) */
#define PM2_5_BREAKPOINT_55_4 554   /**< PM2.5 concentration breakpoint: 55.4 µg/m³ (scaled by 10) */
#define PM2_5_BREAKPOINT_150_4 1504 /**< PM2.5 concentration breakpoint: 150.4 µg/m³ (scaled by 10) */
#define PM2_5_BREAKPOINT_250_4 2504 /**< PM2.5 concentration breakpoint: 250.4 µg/m³ (scaled by 10) */
#define PM2_5_BREAKPOINT_350_4 3504 /**< PM2.5 concentration breakpoint: 350.4 µg/m³ (scaled by 10) */
#define PM2_5_BREAKPOINT_500_4 5004 /**< PM2.5 concentration breakpoint: 500.4 µg/m³ (scaled by 10) */

#define AQI_CATEGORY_GOOD 50                 /**< AQI for PM2.5 concentration up to 12.0 µg/m³ */
#define AQI_CATEGORY_MODERATE 100            /**< AQI for PM2.5 concentration up to 35.4 µg/m³ */
#define AQI_CATEGORY_UNHEALTHY_SENSITIVE 150 /**< AQI for PM2.5 concentration up to 55.4 µg/m³ */
#define AQI_CATEGORY_UNHEALTHY 200           /**< AQI for PM2.5 concentration up to 150.4 µg/m³ */
#define AQI_CATEGORY_VERY_UNHEALTHY 300      /**< AQI for PM2.5 concentration up to 250.4 µg/m³ */
#define AQI_CATEGORY_HAZARDOUS 400           /**< AQI for PM2.5 concentration up to 350.4 µg/m³ */
#define AQI_MAXIMUM 500                      /**< Maximum AQI value for PM2.5 concentration above 500.4 µg/m³ */
#define AQI_THRESHOLD_OFF 0                  /**< AQI threshold value indicating no air quality concern (0 µg/m³) */
#define AQI_THRESHOLD_MAX 150                /**< Maximum AQI threshold value for moderate air quality concern */

#define DUTY_CYCLE_OFF 0   /**< Duty cycle value indicating that the PWM output is turned off */
#define DUTY_CYCLE_MAX 100 /**< Maximum duty cycle value for full PWM output (100%) */
#define SCALE_BY_10 10     /**< Scaling factor used to adjust values by multiplying or dividing by 10 */

volatile uint8_t receivedData = 0; /**< Global variable to store data received via UART using DMA */
volatile uint32_t dutyCycle = 90;  /**< Initial duty cycle value for PWM control */
volatile uint16_t adcValue = 0;    /**< Global variable to hold the ADC reading */
volatile uint16_t dacValue = 0;    /**< Global variable to hold the DAC output value */
volatile uint8_t enableDACOutput =
    0; /**< Flag to control whether the DAC output is enabled (0 = Disabled, 1 = Enabled) */
volatile uint8_t RxBuffer[BUFFER_SIZE]; /**< Buffer to store incoming data received through UART */
volatile uint8_t index = 0;             /**< Index used to track the current position in the UART receive buffer */
volatile uint32_t aqi = 0;              /**< Variable to store the calculated Air Quality Index (AQI) */

GPDMA_Channel_CFG_Type DMAConfig; /**< Global configuration structure for DMA settings */

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

/**
 * @brief Configures the GPIO pins for the application.
 *
 * This function sets up the GPIO pins with the following configurations:
 * - P2.1 as a general-purpose output pin with pull-up resistor and normal mode.
 * - P3.25 as an output pin for the green LED and clears its value.
 * - P0.22 as an output pin for the red LED and clears its value.
 */
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

/**
 * @brief Configures the timer for PWM generation.
 *
 * This function sets up the timer with the specified prescale value and match configurations
 * for PWM generation. It initializes the timer, configures two match channels, and enables
 * the timer interrupt.
 * The function performs the following steps:
 * 1. Initializes the timer with the specified prescale value.
 * 2. Configures match channel 0 with the specified duty cycle.
 * 3. Configures match channel 1 with the specified PWM frequency.
 * 4. Enables the timer interrupt.
 * 5. Starts the timer.
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

/**
 * @brief Configures UART0 and sets up DMA for UART0 RX.
 *
 * This function configures the UART0 peripheral with specified settings
 * and initializes the DMA for receiving data from UART0. It sets up the
 * pin configuration, UART parameters, and DMA channel for UART0 RX.
 *
 * UART0 Configuration:
 * - Port: 0
 * - Pin: 3
 * - Function: 1 (UART0)
 * - Pin mode: 0 (default)
 * - Open drain: 0 (disabled)
 *
 * UART Parameters:
 * - Baud rate: Defined by BAUD_RATE
 * - Parity: None
 * - Data bits: 8
 * - Stop bits: 1
 *
 * DMA Configuration:
 * - Channel: 0
 * - Source memory address: 0 (UART0 RX)
 * - Destination memory address: Address of receivedData variable
 * - Transfer size: 1 byte
 * - Transfer width: Byte
 * - Transfer type: Peripheral to memory (P2M)
 * - Source connection: UART0 RX
 * - Destination connection: None
 * - Linked list item: None
 *
 * The function also enables the DMA interrupt in the NVIC.
 */
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

/**
 * @brief Configures the ADC (Analog-to-Digital Converter) peripheral.
 *
 * This function sets up the pin configuration for the ADC channel and initializes
 * the ADC peripheral with the specified frequency. It also enables the specified
 * ADC channel for conversion.
 *
 * Pin configuration:
 * - Port: 0
 * - Pin: 23
 * - Function: 1 (ADC function)
 * - Pin mode: 0 (default mode)
 *
 * ADC configuration:
 * - Frequency: Defined by ADC_FREQUENCY
 * - Channel: 0 (ADC_CHANNEL_0)
 */
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

/**
 * @brief Configures the Digital-to-Analog Converter (DAC) on the LPC1769.
 *
 * This function sets up the pin configuration for the DAC on pin P0.26 and
 * initializes the DAC peripheral.
 *
 * The pin configuration is set as follows:
 * - Port number: 0
 * - Pin number: 26
 * - Function number: 2 (DAC function)
 *
 * After configuring the pin, the DAC peripheral is initialized using the
 * DAC_Init function.
 */
void configDAC(void)
{
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = PINSEL_PORT_0;
    PinCfg.Pinnum = PINSEL_PIN_26;
    PinCfg.Funcnum = PINSEL_FUNC_2;

    PINSEL_ConfigPin(&PinCfg);

    DAC_Init(LPC_DAC);
}

/**
 * @brief Updates the ADC and DAC values.
 *
 * This function performs an ADC conversion, calculates the corresponding DAC value,
 * and updates the DAC output if it is enabled.
 *
 * The function follows these steps:
 * 1. Starts an ADC conversion.
 * 2. Waits for the ADC conversion to complete.
 * 3. Retrieves the ADC conversion result.
 * 4. Calculates the DAC value based on the ADC result.
 * 5. Updates the DAC output if the DAC output is enabled.
 */
void updateADCandDAC(void)
{
    ADC_StartCmd(LPC_ADC, ADC_START_NOW);
    while (!(ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_0, ADC_DATA_DONE)))
        ;
    adcValue = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);
    dacValue = CALCULATE_DAC_VALUE(adcValue);
    if (enableDACOutput)
    {
        DAC_UpdateValue(LPC_DAC, dacValue);
    }
}

/**
 * @brief Calculate the Air Quality Index (AQI) from PM2.5 concentration.
 *
 * This function calculates the AQI based on the PM2.5 concentration provided.
 * The PM2.5 concentration is expected to be scaled by 10.
 * The AQI is calculated using the breakpoints defined by the US EPA.
 *
 * @param pm2_5 The PM2.5 concentration (scaled by 10).
 * @return The calculated AQI value.
 */
int calculate_aqi_from_pm2_5(uint16_t pm2_5)
{
    if (pm2_5 <= PM2_5_BREAKPOINT_12_0)
        return (pm2_5 * AQI_CATEGORY_GOOD) / PM2_5_BREAKPOINT_12_0;
    else if (pm2_5 <= PM2_5_BREAKPOINT_35_4)
        return AQI_CATEGORY_GOOD + ((pm2_5 - PM2_5_BREAKPOINT_12_0) * (AQI_CATEGORY_MODERATE - AQI_CATEGORY_GOOD)) /
                                       (PM2_5_BREAKPOINT_35_4 - PM2_5_BREAKPOINT_12_0);
    else if (pm2_5 <= PM2_5_BREAKPOINT_55_4)
        return AQI_CATEGORY_MODERATE +
               ((pm2_5 - PM2_5_BREAKPOINT_35_4) * (AQI_CATEGORY_UNHEALTHY_SENSITIVE - AQI_CATEGORY_MODERATE)) /
                   (PM2_5_BREAKPOINT_55_4 - PM2_5_BREAKPOINT_35_4);
    else if (pm2_5 <= PM2_5_BREAKPOINT_150_4)
        return AQI_CATEGORY_UNHEALTHY_SENSITIVE +
               ((pm2_5 - PM2_5_BREAKPOINT_55_4) * (AQI_CATEGORY_UNHEALTHY - AQI_CATEGORY_UNHEALTHY_SENSITIVE)) /
                   (PM2_5_BREAKPOINT_150_4 - PM2_5_BREAKPOINT_55_4);
    else if (pm2_5 <= PM2_5_BREAKPOINT_250_4)
        return AQI_CATEGORY_UNHEALTHY +
               ((pm2_5 - PM2_5_BREAKPOINT_150_4) * (AQI_CATEGORY_VERY_UNHEALTHY - AQI_CATEGORY_UNHEALTHY)) /
                   (PM2_5_BREAKPOINT_250_4 - PM2_5_BREAKPOINT_150_4);
    else if (pm2_5 <= PM2_5_BREAKPOINT_350_4)
        return AQI_CATEGORY_VERY_UNHEALTHY +
               ((pm2_5 - PM2_5_BREAKPOINT_250_4) * (AQI_CATEGORY_HAZARDOUS - AQI_CATEGORY_VERY_UNHEALTHY)) /
                   (PM2_5_BREAKPOINT_350_4 - PM2_5_BREAKPOINT_250_4);
    else if (pm2_5 <= PM2_5_BREAKPOINT_500_4)
        return AQI_CATEGORY_HAZARDOUS + ((pm2_5 - PM2_5_BREAKPOINT_350_4) * (AQI_MAXIMUM - AQI_CATEGORY_HAZARDOUS)) /
                                            (PM2_5_BREAKPOINT_500_4 - PM2_5_BREAKPOINT_350_4);
    else
        return AQI_MAXIMUM;
}

/**
 * @brief Calculates the checksum of the given data.
 *
 * This function computes the checksum by summing up all the bytes in the provided data array.
 *
 * @param data Pointer to the array of data bytes.
 * @param length The number of bytes in the data array.
 * @return The calculated checksum as a 16-bit unsigned integer.
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
 * @brief Calculate the duty cycle for a fan based on the Air Quality Index (AQI).
 *
 * This function determines the duty cycle percentage for a fan based on the provided AQI value.
 * - If the AQI is 0, the fan is turned off (0% duty cycle).
 * - If the AQI is 150 or higher, the fan runs at full speed (100% duty cycle).
 * - For AQI values between 1 and 149, the duty cycle is linearly scaled between 0% and 100%.
 *
 * @param aqi The Air Quality Index (AQI) value.
 * @return The calculated duty cycle percentage (0-100).
 */
uint32_t calculateDutyCycle(uint32_t aqi)
{
    if (aqi == AQI_THRESHOLD_OFF)
    {
        return DUTY_CYCLE_OFF;
    }
    else if (aqi >= AQI_THRESHOLD_MAX)
    {
        return DUTY_CYCLE_MAX;
    }
    else
    {
        return (aqi * DUTY_CYCLE_MAX) / AQI_THRESHOLD_MAX;
    }
}

/**
 * @brief Processes the data received via DMA.
 *
 * This function processes the data received via DMA and performs the following tasks:
 * - Stores the received data in the RxBuffer.
 * - Checks if the first two characters match the expected start characters.
 * - If the buffer is full, it calculates and verifies the checksum.
 * - If the checksum is valid, it extracts the PM2.5 value, scales it, calculates the AQI,
 *   and updates the PWM duty cycle accordingly.
 * - If the checksum is invalid, it sets the appropriate LED indicators.
 * - Resets the index and reconfigures the DMA channel for the next reception.
 */
void processDMAReceivedData(void)
{
    RxBuffer[index] = receivedData;
    if (index == 0 && receivedData != START_CHARACTER_1)
    {
        index = 0;
        return;
    }
    else if (index == 1 && receivedData != START_CHARACTER_2)
    {
        index = 0;
        return;
    }
    if ((index + 1) == BUFFER_SIZE)
    {
        uint16_t calculated_checksum = calculate_checksum(RxBuffer, BUFFER_SIZE - 2);
        uint16_t received_checksum = (RxBuffer[BUFFER_SIZE - 2] << 8) | RxBuffer[BUFFER_SIZE - 1];

        if (calculated_checksum == received_checksum)
        {
            GPIO_SetValue(LED_GREEN_PORT, LED_GREEN_PIN);
            GPIO_ClearValue(LED_RED_PORT, LED_RED_PIN);
            uint16_t pm2_5_value = (RxBuffer[6] << 8) | RxBuffer[7];
            pm2_5_value *= SCALE_BY_10;
            aqi = calculate_aqi_from_pm2_5(pm2_5_value);
            uint32_t dutyCycle = calculateDutyCycle(aqi);
            TIM_UpdateMatchValue(LPC_TIM0, MATCH_CHANNEL_0, (SystemCoreClock / 4) / PWM_FREQUENCY * dutyCycle / 100);
            dutyCycle = DUTY_CYCLE_OFF;
        }
        else
        {
            GPIO_SetValue(LED_RED_PORT, LED_RED_PIN);
            GPIO_ClearValue(LED_GREEN_PORT, LED_GREEN_PIN);
        }
    }
    index = (index + 1) % BUFFER_SIZE;
    GPDMA_ChannelCmd(DMA_CHANNEL_0, DISABLE);
    GPDMA_Setup(&DMAConfig);
    GPDMA_ChannelCmd(DMA_CHANNEL_0, ENABLE);
}

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

int main(void)
{
    configGPIO();
    configTimer();
    configUART_DMA();
    configADC();
    configDAC();

    while (1)
    {
        updateADCandDAC();
    }

    return 0;
}
