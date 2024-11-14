#include "config.h"
#include "global.h"
#include "isr.h"
#include "utils.h"

/**
 * @brief Main function.
 *
 * This function initializes the GPIO, Timer, UART with DMA, ADC, and DAC
 * peripherals. It then enters an infinite loop where it continuously updates
 * the ADC and DAC values.
 *
 * @return int Returns 0 on successful execution.
 */
int main(void)
{
    configGPIO();
    configTimer();
    configUART_DMA();
    configADC();
    configDAC();

    while (1)
    {
        updateADCandDAC();
    }

    return 0;
}
