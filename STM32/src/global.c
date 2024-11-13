#include "global.h"

volatile uint8_t rx_buffer[RX_BUFFER_SIZE];           /**< Buffer to store received UART data */
volatile uint8_t rx_index = 0;                        /**< Index for the UART receive buffer */
volatile uint16_t adc_value = 0;                      /**< ADC value acquired via DMA */
volatile uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE]; /**< Buffer for UART transmission data */
volatile bool dma_transfer_completed = false;         /**< Flag indicating completion of a DMA transfer */
volatile uint8_t received_data = 0;                   /**< Last received byte from UART */
volatile bool frame_received = false;          /**< Flag indicating that a complete data frame has been received */
int aqi = 0;                                   /**< Air Quality Index (AQI) value calculated from PM2.5 data */
TaskHandle_t uartTaskHandle = NULL;            /**< Handle for the UART task */
TaskHandle_t adcTaskHandle = NULL;             /**< Handle for the ADC task */
TaskHandle_t frameProcessingTaskHandle = NULL; /**< Handle for the frame processing task */
