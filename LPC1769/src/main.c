/*
 * @file main.c
 * @brief Main file for the project
 */

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#ifdef __USE_MCUEXPRESSO
#include <cr_section_macros.h> /* The cr_section_macros is specific to the MCUXpresso delivered toolchain */
#endif

#include "lpc17xx_gpio.h"    /* GPIO handling */
#include "lpc17xx_pinsel.h"  /* Pin function selection */
#include "lpc17xx_systick.h" /* SysTick handling */
#include "lpc17xx_timer.h"   /* Timer handling*/

/* Pin Definitions */
#define LED_PIN ((uint32_t)(1 << 30))               /* P0.30 connected to LED */
#define REED_SWITCH_OPEN_PIN ((uint32_t)(1 << 8))   /* P0.8 connected to REED SWITCH SENSOR (DOOR OPEN) */
#define REED_SWITCH_CLOSED_PIN ((uint32_t)(1 << 9)) /* P0.9 connected to REED SWITCH SENSOR (DOOR CLOSED) */
#define TEMP_SENSOR_PIN ((uint32_t)(1 << 16))       /* P0.16 connected to TEMPERATURE SENSOR */
#define BATTERY_VH_PIN ((uint32_t)(1 << 20))        /* P0.20 connected to BATTERY - VERY HIGH*/
#define BATTERY_H_PIN ((uint32_t)(1 << 19))         /* P0.19 connected to BATTERY - HIGH */
#define BATTERY_L_PIN ((uint32_t)(1 << 18))         /* P0.18 connected to BATTERY - LOW */
#define DOOR_OPEN_PIN ((uint32_t)(1 << 12))         /* P0.12 connected to DOOR */
#define DOOR_CLOSE_PIN ((uint32_t)(1 << 13))        /* P0.13 connected to DOOR */
#define BUTTON_INSIDE_PIN ((uint32_t)(1 << 21))     /* P0.21 connected to INSIDE BUTTON */
#define BUTTON_OUTSIDE_PIN ((uint32_t)(1 << 22))    /* P0.22 connected to OUTSIDE BUTTON */

/* GPIO Direction Definitions */
#define INPUT 0
#define OUTPUT 1

/* Define time variables */
#define SYSTICK_INITIAL_TIME 1 /* Expressed in milliseconds */

/* Define edge variable */
#define EDGE_RISING 0
#define EDGE_FALLING 1

/* Boolean Values */
#define TRUE 1
#define FALSE 0

#define SECOND 10000
#define COUNTER_LIMIT 1800

/* Function prototypes */
void configure_pins(void);
void configure_systick(void);
void configure_timer(void);

uint8_t door_opening_flag = 0;
uint8_t door_closing_flag = 0;
uint8_t event_flag = 0;
int close_count;

int main(void)
{
    SystemInit(); /*Initialize the system clock*/

    configure_pins();

    configure_systick();

    SYSTICK_IntCmd(ENABLE); /* Enable SysTick interrupt */

    SYSTICK_Cmd(ENABLE); /* Enable SysTick counter */

    NVIC_SetPriority(SysTick_IRQn, 0);
    NVIC_SetPriority(TIMER0_IRQn, 1);
    NVIC_SetPriority(EINT3_IRQn, 3); // CAMBIAR PRIORIDADES

    NVIC_EnableIRQ(SysTick_IRQn);
    NVIC_EnableIRQ(TIMER0_IRQn);
    NVIC_EnableIRQ(EINT3_IRQn);

    while (TRUE)
    {
    }

    return 0; /* Should never reach this */
}

void configure_pins(void)
{
    PINSEL_CFG_Type pin_cfg; /* Create a variable to store the configuration of the pin */

    /* We need to configure the struct with the desired configuration */
    pin_cfg.Portnum = PINSEL_PORT_0;           /* The port number is 0 */
    pin_cfg.Pinnum = PINSEL_PIN_21;            /* The pin number is 21 */
    pin_cfg.Funcnum = PINSEL_FUNC_0;           /* The function number is 0 */
    pin_cfg.Pinmode = PINSEL_PINMODE_PULLUP;   /* The pin mode is pull-up */
    pin_cfg.OpenDrain = PINSEL_PINMODE_NORMAL; /* The pin is in the normal mode */
    PINSEL_ConfigPin(&pin_cfg);                // BUTTON_INSIDE_CONFIG
    GPIO_SetDir(PINSEL_PORT_0, BUTTON_INSIDE_PIN, INPUT);

    pin_cfg.Pinnum = PINSEL_PIN_22;
    PINSEL_ConfigPin(&pin_cfg); // BUTTON_OUTSIDE_CONFIG
    GPIO_SetDir(PINSEL_PORT_0, BUTTON_OUTSIDE_PIN, INPUT);

    pin_cfg.Pinnum = PINSEL_PIN_8;
    PINSEL_ConfigPin(&pin_cfg); // REED_SWITCH_OPEN_CONFIG
    GPIO_SetDir(PINSEL_PORT_0, REED_SWITCH_OPEN_PIN, INPUT);

    pin_cfg.Pinnum = PINSEL_PIN_9;
    PINSEL_ConfigPin(&pin_cfg); // REED_SWITCH_CLOSED_CONFIG
    GPIO_SetDir(PINSEL_PORT_0, REED_SWITCH_CLOSED_PIN, INPUT);

    pin_cfg.Pinnum = PINSEL_PIN_18;
    pin_cfg.Pinmode = PINSEL_PINMODE_PULLDOWN; // BATTERY_L_CONFIG
    PINSEL_ConfigPin(&pin_cfg);
    GPIO_SetDir(PINSEL_PORT_0, BATTERY_L_PIN, INPUT);

    pin_cfg.Pinnum = PINSEL_PIN_19;
    PINSEL_ConfigPin(&pin_cfg); // BATTERY_H_CONFIG
    GPIO_SetDir(PINSEL_PORT_0, BATTERY_H_PIN, INPUT);

    pin_cfg.Pinnum = PINSEL_PIN_20;
    PINSEL_ConfigPin(&pin_cfg); // BATTERY_VH_CONFIG
    GPIO_SetDir(PINSEL_PORT_0, BATTERY_VH_PIN, INPUT);

    pin_cfg.Pinnum = PINSEL_PIN_16;
    PINSEL_ConfigPin(&pin_cfg); // TEMP_SENSOR_CONFIG
    GPIO_SetDir(PINSEL_PORT_0, TEMP_SENSOR_PIN, INPUT);

    pin_cfg.Pinnum = PINSEL_PIN_12;
    PINSEL_ConfigPin(&pin_cfg); // DOOR_OPEN_CONFIG
    GPIO_SetDir(PINSEL_PORT_0, DOOR_OPEN_PIN, OUTPUT);

    pin_cfg.Pinnum = PINSEL_PIN_13;
    PINSEL_ConfigPin(&pin_cfg); // DOOR_CLOSE_CONFIG
    GPIO_SetDir(PINSEL_PORT_0, DOOR_CLOSE_PIN, OUTPUT);

    pin_cfg.Pinnum = PINSEL_PIN_22;
    pin_cfg.Pinmode = PINSEL_PINMODE_PULLUP;
    PINSEL_ConfigPin(&pin_cfg); // LED_CONFIG
    GPIO_SetDir(PINSEL_PORT_0, LED_PIN, OUTPUT);

    GPIO_IntCmd(PINSEL_PORT_0, TEMP_SENSOR_PIN, EDGE_RISING);
    GPIO_IntCmd(PINSEL_PORT_0, BUTTON_INSIDE_PIN, EDGE_RISING);
    GPIO_IntCmd(PINSEL_PORT_0, BUTTON_OUTSIDE_PIN, EDGE_RISING);
    GPIO_IntCmd(PINSEL_PORT_0, REED_SWITCH_CLOSED_PIN, EDGE_RISING);
    GPIO_IntCmd(PINSEL_PORT_0, REED_SWITCH_CLOSED_PIN, EDGE_RISING);
}

void configure_timer(void)
{
    TIM_TIMERCFG_Type timerConfig;

    timerConfig.PrescaleOption = TIM_PRESCALE_USVAL;
    timerConfig.PrescaleValue = (uint32_t)100; // 100 uS
    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &timerConfig);

    TIM_MATCHCFG_Type matchConfig;
    matchConfig.MatchChannel = 0;
    matchConfig.IntOnMatch = ENABLE;
    matchConfig.ResetOnMatch = ENABLE;
    matchConfig.StopOnMatch = DISABLE;
    matchConfig.ExtMatchOutputType = TIM_EXTMATCH_NOTHING;
    matchConfig.MatchValue = SECOND;

    TIM_ConfigMatch(LPC_TIM0, &matchConfig);
    TIM_Cmd(LPC_TIM0, ENABLE);
}

void configure_systick(void)
{
    SYSTICK_InternalInit(SYSTICK_INITIAL_TIME); /* Initialize the SysTick timer with a time interval of 1 ms */
    SYSTICK_IntCmd(ENABLE);
    SYSTICK_Cmd(ENABLE);
}

void SysTick_Handler(void)
{
    SYSTICK_ClearCounterFlag(); /* Clear interrupt flag */

    uint32_t valorGPIO = GPIO_ReadValue(PINSEL_PORT_0);

    if (valorGPIO & (1 << 18))
    {
        SYSTICK_InternalInit(100); // REVISAR ( numeros magicos)
    }
    else if (valorGPIO & (1 << 19))
    {
        SYSTICK_InternalInit(50);
    }
    else if (valorGPIO & (1 << 20))
    {
        SYSTICK_InternalInit(20);
    }

    if (GPIO_ReadValue(PINSEL_PORT_0) & LED_PIN)
    {
        GPIO_ClearValue(PINSEL_PORT_0, LED_PIN); /* Turn off LED */
    }
    else
    {
        GPIO_SetValue(PINSEL_PORT_0, LED_PIN); /* Turn on LED */
    }
}

// Overwrite the interrupt handle routine for GPIO
void EINT3_IRQHandler(void)
{
    if (door_closing_flag && GPIO_GetIntStatus(PINSEL_PORT_0, REED_SWITCH_CLOSED_PIN, EDGE_RISING))
    {
        GPIO_ClearValue(PINSEL_PORT_0, DOOR_CLOSE_PIN);
        door_closing_flag = 0;
        event_flag = 1;
    }
    if (door_opening_flag && GPIO_GetIntStatus(PINSEL_PORT_0, REED_SWITCH_OPEN_PIN, EDGE_RISING))
    {
        GPIO_ClearValue(PINSEL_PORT_0, DOOR_OPEN_PIN);
        door_opening_flag = 0;
        event_flag = 0;
    }
    if (GPIO_GetIntStatus(PINSEL_PORT_0, TEMP_SENSOR_PIN, EDGE_RISING) &&
        (GPIO_ReadValue(PINSEL_PORT_0) & REED_SWITCH_OPEN_PIN))
    {
        close_door();
    }
    if (GPIO_GetIntStatus(PINSEL_PORT_0, BUTTON_INSIDE_PIN, EDGE_RISING) &&
        (GPIO_ReadValue(PINSEL_PORT_0) & REED_SWITCH_OPEN_PIN))
    {
        close_door();
    }
    if (GPIO_GetIntStatus(PINSEL_PORT_0, BUTTON_INSIDE_PIN, EDGE_RISING) &&
        (GPIO_ReadValue(PINSEL_PORT_0) & REED_SWITCH_CLOSED_PIN))
    {
        open_door();
    }
    if (GPIO_GetIntStatus(PINSEL_PORT_0, BUTTON_OUTSIDE_PIN, EDGE_RISING) &&
        (GPIO_ReadValue(PINSEL_PORT_0) & REED_SWITCH_OPEN_PIN))
    {
        close_door();
    }
    if (GPIO_GetIntStatus(PINSEL_PORT_0, BUTTON_OUTSIDE_PIN, EDGE_RISING) &&
        (GPIO_ReadValue(PINSEL_PORT_0) & REED_SWITCH_CLOSED_PIN))
    {
        open_door();
    }
}

void TIMER0_IRQHandler()
{
    if ((!event_flag) && (GPIO_ReadValue(PINSEL_PORT_0) & REED_SWITCH_OPEN_PIN))
    {
        close_count++;
    }

    if (close_count == COUNTER_LIMIT)
    {
        close_door();
    }
}

void close_door(void)
{
    GPIO_SetValue(PINSEL_PORT_0, DOOR_CLOSE_PIN);
    door_closing_flag = 1;
    event_flag = 1;
    close_count = 0;
}

void open_door(void)
{
    GPIO_SetValue(PINSEL_PORT_0, DOOR_OPEN_PIN);
    door_opening_flag = 1;
}