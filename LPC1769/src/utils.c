#include "utils.h"

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
    if (aqi == AQI_THRESHOLD_OFF)
    {
        enableDACOutput = 0;
        DAC_UpdateValue(LPC_DAC, 0);
        return DUTY_CYCLE_OFF;
    }
    else if (aqi >= AQI_THRESHOLD_MAX)
    {
        enableDACOutput = 1;
        return DUTY_CYCLE_MAX;
    }
    else
    {
        enableDACOutput = 0;
        DAC_UpdateValue(LPC_DAC, 0);
        return (DUTY_CYCLE_MAX - (aqi * 90) / AQI_THRESHOLD_MAX);
    }
}

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
