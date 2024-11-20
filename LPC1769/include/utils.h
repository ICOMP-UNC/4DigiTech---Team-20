#ifndef UTILS_H
#define UTILS_H

#include "global.h"

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
void updateADCandDAC(void);

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
void processDMAReceivedData(void);

/**
 * @brief Calculates the checksum of the given data.
 *
 * This function computes the checksum by summing up all the bytes in the provided data array.
 *
 * @param data Pointer to the array of data bytes.
 * @param length The number of bytes in the data array.
 * @return The calculated checksum as a 16-bit unsigned integer.
 */
uint16_t calculate_checksum(const uint8_t* data, uint16_t length);

/**
 * @brief Calculates the Air Quality Index (AQI) from the PM2.5 value.
 *
 * This function calculates the Air Quality Index (AQI) based on the provided PM2.5 value.
 * The AQI is calculated based on the breakpoints defined by the EPA for PM2.5 concentrations.
 *
 * @param pm2_5 The PM2.5 value to calculate the AQI for.
 * @return The calculated Air Quality Index (AQI).
 */
int calculate_aqi_from_pm2_5(uint16_t pm2_5);

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
uint32_t calculateDutyCycle(uint32_t aqi);

#endif // UTILS_H
