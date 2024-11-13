#include "tasks.h"

void uart_task(void* args)
{
    (void)args;
    while (1)
    {

        uint16_t pm2_5_value = (adc_value * 300) / 4095; // 12-bit ADC
        uint16_t sensor_data[13] = {
            150,         // Data 1: PM1.0 concentration in µg/m³ (standard particles) (Frame 4 and 5)
            pm2_5_value, // Data 2: Variable PM2.5 concentration in µg/m³ (scaled from adc_value) (Frame 6 and 7)
            300,         // Data 3: PM10 concentration in µg/m³ (standard particles) (Frame 8 and 9)
            160,         // Data 4: PM1.0 concentration in µg/m³ (under atmospheric environment) (Frame 10 and 11)
            270,         // Data 5: PM2.5 concentration in µg/m³ (under atmospheric environment) (Frame 12 and 13)
            320,         // Data 6: PM10 concentration in µg/m³ (under atmospheric environment) (Frame 14 and 15)
            1200,        // Data 7: number of particles with diameter > 0.3 µm in 0.1 L of air (Frame 16 and 17)
            800,         // Data 8: number of particles with diameter > 0.5 µm in 0.1 L of air (Frame 18 and 19)
            400,         // Data 9: number of particles with diameter > 1.0 µm in 0.1 L of air (Frame 20 and 21)
            200,         // Data 10: number of particles with diameter > 2.5 µm in 0.1 L of air (Frame 22 and 23)
            100,         // Data 11: number of particles with diameter > 5.0 µm in 0.1 L of air (Frame 24 and 25)
            50,          // Data 12: number of particles with diameter > 10 µm in 0.1 L of air (Frame 26 and 27)
            0            // Reserved
        };
        send_data_frame(sensor_data);
        UBaseType_t uartHighWaterMark = uxTaskGetStackHighWaterMark(uartTaskHandle);
        printf("UART Task Stack High Water Mark: %lu\n", uartHighWaterMark);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void adc_task(void* args)
{
    (void)args;
    while (1)
    {
        handle_dma_adc_transfer();
        UBaseType_t adcHighWaterMark = uxTaskGetStackHighWaterMark(adcTaskHandle);
        printf("ADC Task Stack High Water Mark: %lu\n", adcHighWaterMark);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void frame_processing_task(void* args)
{
    (void)args;
    while (1)
    {
        process_received_frame();
        UBaseType_t frameHighWaterMark = uxTaskGetStackHighWaterMark(frameProcessingTaskHandle);
        printf("Frame Processing Task Stack High Water Mark: %lu\n", frameHighWaterMark);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    while (1)
        ;
}
