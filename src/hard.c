//---------------------------------------------
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ## @CPU:    STM32F030
// ##
// #### HARD.C ################################
//---------------------------------------------

/* Includes ------------------------------------------------------------------*/
#include "hard.h"
#include "tim.h"
#include "stm32f0xx.h"
#include "adc.h"
#include "dsp.h"

#include <stdio.h>
#include "uart.h"

/* Externals variables ---------------------------------------------------------*/
extern volatile unsigned short timer_led;
extern volatile unsigned short adc_ch[];




/* Global variables ------------------------------------------------------------*/
//for LED indicator
led_state_t led_state = START_BLINKING;
unsigned char blink = 0;
unsigned char how_many_blinks = 0;
//for voltage and synchro
// unsigned char hard_filter_index = 0;
// unsigned char last_voltage_was_high = 0;
// unsigned short integrate_voltage = 0;
// unsigned short integrate_voltage_in_positive = 0;
// unsigned short last_voltage_cycle = 0;
// unsigned short last_voltage_cycle_in_positive = 0;
// unsigned short last_voltage_peak = 0;
unsigned char current_vline_cnt = 0;
unsigned short current_vline_peak = 0;
unsigned short current_vline_ones = 0;
unsigned short last_vline_peak = 0;
unsigned short last_vline_ones = 0;
// unsigned short voltage_peak = 0;


/* Module Functions ------------------------------------------------------------*/


//cambia configuracion de bips del LED
void ChangeLed (unsigned char how_many)
{
    how_many_blinks = how_many;
    led_state = START_BLINKING;
}

//mueve el LED segun el estado del Pote
void UpdateLed (void)
{
    switch (led_state)
    {
        case START_BLINKING:
            blink = how_many_blinks;
            
            if (blink)
            {
                LED_ON;
                timer_led = 200;
                led_state++;
                blink--;
            }
            break;

        case WAIT_TO_OFF:
            if (!timer_led)
            {
                LED_OFF;
                timer_led = 200;
                led_state++;
            }
            break;

        case WAIT_TO_ON:
            if (!timer_led)
            {
                if (blink)
                {
                    blink--;
                    timer_led = 200;
                    led_state = WAIT_TO_OFF;
                    LED_ON;
                }
                else
                {
                    led_state = WAIT_NEW_CYCLE;
                    timer_led = 2000;
                }
            }
            break;

        case WAIT_NEW_CYCLE:
            if (!timer_led)
                led_state = START_BLINKING;

            break;

        default:
            led_state = START_BLINKING;
            break;
    }
}


//this is called from main on each sample
//24KHz or 23.3KHz
//on 100Hz -> 233 pts or 240 pts
unsigned char Hard_Update_Vline (unsigned short new_sample)
{
    unsigned char cycle_ended = 0;
    
    //search for vline_peak
    if (new_sample > current_vline_peak)
        current_vline_peak = new_sample;

    //search for conduction angle
    if (new_sample > VLINE_ZERO_THRESHOLD)
        current_vline_ones++;
    
    if (current_vline_cnt < 240)
        current_vline_cnt++;
    else
    {
        last_vline_peak = current_vline_peak;
        last_vline_ones = current_vline_ones;
        
        current_vline_peak = 0;
        current_vline_ones = 0;
        current_vline_cnt = 0;
        cycle_ended = 1;
    }

    return cycle_ended;
}

unsigned short Hard_Get_Vline_Peak (void)
{
    return last_vline_peak;
}

//answer between 0 and 180, check if its better 0 to 255
unsigned char Hard_Get_Vline_Conduction_Angle (void)
{
    unsigned int temp = 0;

#ifdef USE_FREQ_48KHZ
    temp = last_vline_ones * 180;
    temp = temp / 240;
#endif

#ifdef USE_FREQ_70KHZ
    temp = last_vline_ones * 180;
    temp = temp / 233;
#endif

    if (temp > 180)
        temp = 180;

    return (unsigned char) temp;
}

// unsigned short Hard_Get_Hidden_Value (void)
// {
//     return last_voltage_cycle;
//     // return last_voltage_cycle_in_positive;
// }

//---- end of file ----//
