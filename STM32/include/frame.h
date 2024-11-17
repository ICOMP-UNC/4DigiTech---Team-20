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

#endif // FRAME_H
