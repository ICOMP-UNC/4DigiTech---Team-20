#ifndef GLOBAL_H
#define GLOBAL_H

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
    (1023 - ((adcValue * 1023) / 4095)) /**< Macro to calculate the DAC value from the ADC reading */
#define BUFFER_SIZE 32                  /**< Buffer size for UART communication */
#define START_CHARACTER_1 0x42          /**< ASCII character 'B' (Start character 1 for data frame) */
#define START_CHARACTER_2 0x4D          /**< ASCII character 'M' (Start character 2 for data frame) */
#define LED_GREEN_PORT 3                /**< Port number for the green LED */
#define LED_GREEN_PIN (1 << 25)         /**< Pin number (P3.25) for the green LED */
#define LED_RED_PORT 0                  /**< Port number for the red LED */
#define LED_RED_PIN (1 << 22)           /**< Pin number (P0.22) for the red LED */
#define PWM_PORT 2                      /**< Port number for the PWM output */

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

extern volatile uint8_t receivedData; /**< Global variable to store data received via UART using DMA */
extern volatile uint32_t dutyCycle;   /**< Initial duty cycle value for PWM control */
extern volatile uint16_t adcValue;    /**< Global variable to hold the ADC reading */
extern volatile uint16_t dacValue;    /**< Global variable to hold the DAC output value */
extern volatile uint8_t
    enableDACOutput; /**< Flag to control whether the DAC output is enabled (0 = Disabled, 1 = Enabled) */
extern volatile uint8_t RxBuffer[BUFFER_SIZE]; /**< Buffer to store incoming data received through UART */
extern volatile uint8_t index;           /**< Index used to track the current position in the UART receive buffer */
extern volatile uint32_t aqi;            /**< Variable to store the calculated Air Quality Index (AQI) */
extern GPDMA_Channel_CFG_Type DMAConfig; /**< Global configuration structure for DMA settings */

#endif // GLOBAL_H
