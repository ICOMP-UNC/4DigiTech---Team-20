/**
 * @file frame.h
 * @brief Functions for processing and transmitting data frames, and calculating AQI.
 *
 * This header file contains the declarations of functions used to process received data frames,
 * calculate checksums, send data frames over USART, and calculate the Air Quality Index (AQI)
 * from PM2.5 concentration values.
 */

#ifndef FRAME_H
#define FRAME_H

#include "global.h" /**< Global definitions and declarations */

/**
 * @brief Processes the received frame and calculates the AQI from PM2.5 value.
 *
 * This function checks if a frame has been received. If a frame is received, it calculates the checksum
 * of the received data and compares it with the received checksum. If the checksums match, it extracts
 * the PM2.5 value from the received data, scales it, and calculates the AQI (Air Quality Index) from the
 * PM2.5 value.
 *
 */
void process_received_frame(void);

/**
 * @brief Calculates the checksum of the given data.
 *
 * This function computes a simple checksum by summing up all the bytes
 * in the provided data array.
 *
 * @param data Pointer to the data array.
 * @param length Length of the data array.
 * @return The calculated checksum.
 */
uint16_t calculate_checksum(const uint8_t* data, uint16_t length);

/**
 * @brief Sends a data frame containing sensor data over USART.
 *
 * This function constructs a data frame with a start sequence, frame length,
 * sensor data, and a checksum, and sends it over USART2.
 *
 * @param sensor_data Pointer to an array of 13 uint16_t sensor data values.
 *
 * The data frame format is as follows:
 * - Byte 0: START_CHARACTER_1
 * - Byte 1: START_CHARACTER_2
 * - Byte 2: High byte of frame length
 * - Byte 3: Low byte of frame length
 * - Bytes 4-29: Sensor data (each sensor value is 2 bytes, high byte first)
 * - Byte 30: High byte of checksum
 * - Byte 31: Low byte of checksum
 *
 * The checksum is calculated over the first 30 bytes of the frame.
 */
void send_data_frame(uint16_t* sensor_data);

/**
 * @brief Calculate the Air Quality Index (AQI) from PM2.5 concentration.
 *
 * This function converts the PM2.5 concentration (in µg/m³, scaled by 10) to the corresponding AQI value.
 * The AQI is calculated based on the breakpoints defined by the U.S. Environmental Protection Agency (EPA).
 *
 * @param pm2_5 The PM2.5 concentration in µg/m³, scaled by 10 (e.g., 120 represents 12.0 µg/m³).
 * @return The corresponding AQI value.
 *
 * The AQI is calculated using the following breakpoints:
 * - 0 to 12.0 µg/m³ (scaled by 10): AQI 0 to 50
 * - 12.1 to 35.4 µg/m³ (scaled by 10): AQI 51 to 100
 * - 35.5 to 55.4 µg/m³ (scaled by 10): AQI 101 to 150
 * - 55.5 to 150.4 µg/m³ (scaled by 10): AQI 151 to 200
 * - 150.5 to 250.4 µg/m³ (scaled by 10): AQI 201 to 300
 * - 250.5 to 350.4 µg/m³ (scaled by 10): AQI 301 to 400
 * - 350.5 to 500.4 µg/m³ (scaled by 10): AQI 401 to 500
 * - Above 500.4 µg/m³ (scaled by 10): AQI 500 (maximum value)
 */
int calculate_aqi_from_pm2_5(uint16_t pm2_5);

#endif // FRAME_H
