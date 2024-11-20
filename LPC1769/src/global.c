#include "global.h"

// Define global variables
volatile uint8_t receivedData = 0;
volatile uint32_t dutyCycle = 90;
volatile uint16_t adcValue = 0;
volatile uint16_t dacValue = 0;
volatile uint8_t enableDACOutput = 0;
volatile uint8_t RxBuffer[BUFFER_SIZE];
volatile uint8_t index = 0;
volatile uint32_t aqi = 0;

GPDMA_Channel_CFG_Type DMAConfig;
