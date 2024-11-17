/**
 * @file isr.h
 * @brief Interrupt Service Routines (ISRs) for USART and DMA.
 *
 * This header file contains the declarations of the interrupt service routines (ISRs)
 * for handling USART and DMA interrupts. It also includes a function for handling
 * the completion of DMA ADC transfers.
 */

#ifndef ISR_H
#define ISR_H

#include "global.h" /**< Global definitions and declarations */
#include "isr.h"    /**< ISR function declarations */

/**
 * @brief Handles the completion of a DMA ADC transfer.
 *
 * This function is called when a DMA transfer is completed. It resets the DMA
 * channel, re-enables it, and starts a new ADC conversion.
 *
 * The function performs the following steps:
 * 1. Checks if the DMA transfer is completed.
 * 2. Resets the `dma_transfer_completed` flag.
 * 3. Disables the DMA channel.
 * 4. Sets the number of data to be transferred by the DMA channel to 1.
 * 5. Re-enables the DMA channel.
 * 6. Starts a new ADC conversion.
 */
void handle_dma_adc_transfer(void);

/**
 * @brief DMA1 Channel 1 Interrupt Service Routine.
 *
 * This ISR handles the DMA1 Channel 1 interrupts. It checks for the transfer complete
 * and transfer error interrupt flags, clears them, and sets the dma_transfer_completed
 * flag if the transfer is complete.
 */
void dma1_channel1_isr(void);

#endif // ISR_H
