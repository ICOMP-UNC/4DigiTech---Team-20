#ifndef ISR_H
#define ISR_H

#include "global.h"

/**
 * @brief DMA Interrupt Handler
 *
 * This function handles the DMA interrupt for channel 0. It checks if the
 * interrupt is due to a terminal count (transfer complete) and clears the
 * interrupt pending status. If the interrupt is due to a terminal count,
 * it calls the processDMAReceivedData() function to handle the received data.
 */
void DMA_IRQHandler(void);

/**
 * @brief TIMER0 Interrupt Handler
 *
 * This function handles the interrupt for TIMER0. It checks the interrupt status
 * for match register 0 (MR0) and match register 1 (MR1) and performs the following actions:
 * - If the interrupt is from MR0, it sets the value of the specified GPIO pin.
 * - If the interrupt is from MR1, it clears the value of the specified GPIO pin.
 */
void TIMER0_IRQHandler(void);

#endif // ISR_H
