#include "global.h"

volatile uint16_t adc_value = 0;                      /**< ADC value acquired via DMA */
volatile uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE]; /**< Buffer for UART transmission data */
volatile bool dma_transfer_completed = false;         /**< Flag indicating completion of a DMA transfer */
TaskHandle_t uartTaskHandle = NULL;                   /**< Handle for the UART task */
TaskHandle_t adcTaskHandle = NULL;                    /**< Handle for the ADC task */
SemaphoreHandle_t dmaSemaphore;                       /**< Semaphore for DMA transfer synchronization */
