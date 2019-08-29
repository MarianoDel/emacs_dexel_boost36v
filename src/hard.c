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

#define STRING2(x) #x
#define STRING(x) STRING2(x)
// #define STRING_CONCAT(str1,str2) #str1 " " #str2
// #pragma message "Corriente para calculos"
// #pragma message(STRING_CONCAT("Corriente para calculos",I_FOR_CALC))
// #pragma message(STRING_CONCAT("Corriente para calculos",STRING(I_FOR_CALC)))
#pragma message(STRING(I_FOR_CALC))

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

//proteccion para no superar el valor Vin . Ton que puede saturar al trafo
//con 6T primario
// unsigned short UpdateDMAX (unsigned short a)
// {
//     if (a > VIN_35V)
//         a = 260;
//     else if (a > VIN_30V)
//         a = 297;
//     else if (a > VIN_25V)
//         a = 347;
//     else if (a > VIN_20V)
//         a = 417;
//     else
//         a = 450;
    
//     return a;
// }

// unsigned short UpdateDMAXSF (unsigned short a)
// {
//     //por saturacion en arranque cambio max D
//     if (a > VIN_35V)
//         a = 50;
//     else if (a > VIN_30V)
//         a = 70;
//     else if (a > VIN_25V)
//         a = 90;
//     else if (a > VIN_20V)
//         a = 120;
//     else
//         a = 150;

//     return a;
// }

//Calcula en funcion de la tension aplicada a la bobina Lout
//el maxido d en ticks posible. Utiliza Imax (entrada o salida), Lout, tick_pwm
// unsigned short UpdateDmaxLout (unsigned short delta_voltage)
// {
//     unsigned int num, den;

//     if (delta_voltage > 0)
//     {
//         // num = I_FOR_CALC * LOUT_UHY * 1000;    //cambio para no tener decimales en el preprocesador
//         num =  (ILOUT * 1000) * LOUT_UHY;
//         // num = I_FOR_CALC_MILLIS * LOUT_UHY;    
//         den = delta_voltage * TICK_PWM_NS;
//         num = num / den;

//         if (num > DMAX_HARDWARE)
//             num = DMAX_HARDWARE;
//     }
//     else
//         num = DMAX_HARDWARE;

//     return (unsigned short) num;
// }

//Convierte el valor de ticks ADC Vout a tension
// unsigned short VoutTicksToVoltage (unsigned short sample_adc)
// {
//     unsigned int num;

//     if (sample_adc > VOUT_300V)
//     {
//         num = sample_adc * 350;
//         num = num / VOUT_350V;
//     }
//     else if (sample_adc > VOUT_200V)
//     {
//         num = sample_adc * 300;
//         num = num / VOUT_300V;
//     }
//     else if (sample_adc > VOUT_110V)
//     {
//         num = sample_adc * 200;
//         num = num / VOUT_200V;
//     }
//     else
//     {
//         num = sample_adc * 110;
//         num = num / VOUT_110V;
//     }
    
//     return (unsigned short) num;
// }

//Convierte el valor de ticks ADC Vin a tension
// unsigned short VinTicksToVoltage (unsigned short sample_adc)
// {
//     unsigned int num;

//     if (sample_adc > VIN_30V)
//     {
//         num = sample_adc * 35;
//         num = num / VIN_35V;
//     }
//     else if (sample_adc > VIN_25V)
//     {
//         num = sample_adc * 30;
//         num = num / VIN_30V;
//     }
//     else if (sample_adc > VIN_20V)
//     {
//         num = sample_adc * 25;
//         num = num / VIN_25V;
//     }    
//     else
//     {
//         num = sample_adc * 20;
//         num = num / VIN_20V;
//     }
    
//     return (unsigned short) num;
// }


//Con la tension de entrada y salida calcula el maximo periodo permitido
// unsigned short Hard_GetDmaxLout (unsigned short vin, unsigned short vout)
// {
//     unsigned int delta_vout = 0;
//     unsigned short normalized_vout = 0;

//     delta_vout = VinTicksToVoltage(vin);
//     delta_vout = (delta_vout * N_TRAFO) / 1000;

//     normalized_vout = VoutTicksToVoltage(vout);
    
//     if (delta_vout > normalized_vout)
//         delta_vout = delta_vout - normalized_vout;
//     else
//         delta_vout = 0;
    
//     return UpdateDmaxLout((unsigned short)delta_vout);
// }

void WelcomeCodeFeatures (char * str)
{
    // Main Program Type
#if (defined INVERTER_MODE_PURE_SINUSOIDAL)
    sprintf(str,"[%s] %s\n", __FILE__, str_macro(INVERTER_MODE_PURE_SINUSOIDAL));
    Usart1Send(str);
    Wait_ms(30);        
#elif defined INVERTER_MODE
    sprintf(str,"[%s] %s\n", __FILE__, str_macro(INVERTER_MODE));    
    Usart1Send(str);
    Wait_ms(30);    
#endif

#ifdef INVERTER_MODE_CURRENT_FDBK
    sprintf(str,"[%s] %s\n", __FILE__, str_macro(INVERTER_MODE_CURRENT_FDBK));
    Usart1Send(str);
    Wait_ms(30);    
#endif

#ifdef INVERTER_MODE_GRID_TIE
    sprintf(str,"[%s] %s\n", __FILE__, str_macro(INVERTER_MODE_GRID_TIE));
    Usart1Send(str);
    Wait_ms(30);    
#endif

#ifdef INVERTER_ONLY_SYNC_AND_POLARITY
    sprintf(str,"[%s] %s\n", __FILE__, str_macro(INVERTER_ONLY_SYNC_AND_POLARITY));
    Usart1Send(str);
    Wait_ms(30);    
#endif
    
    // Features mostly on hardware
#ifdef USE_FREQ_48KHZ
    sprintf(str,"[%s] %s\n", __FILE__, str_macro(USE_FREQ_48KHZ));
    Usart1Send(str);
    Wait_ms(30);    
#endif
#ifdef USE_FREQ_24KHZ
    sprintf(str,"[%s] %s\n", __FILE__, str_macro(USE_FREQ_24KHZ));
    Usart1Send(str);
    Wait_ms(30);    
#endif
#ifdef USE_FREQ_16KHZ
    sprintf(str,"[%s] %s\n", __FILE__, str_macro(USE_FREQ_16KHZ));
    Usart1Send(str);
    Wait_ms(30);    
#endif
#ifdef USE_FREQ_12KHZ
    sprintf(str,"[%s] %s\n", __FILE__, str_macro(USE_FREQ_12KHZ));
    Usart1Send(str);
    Wait_ms(30);    
#endif
#ifdef USE_FREQ_9_6KHZ
    sprintf(str,"[%s] %s\n", __FILE__, str_macro(USE_FREQ_9_6KHZ));
    Usart1Send(str);
    Wait_ms(30);    
#endif

#ifdef WITH_AC_SYNC_INT
    sprintf(str,"[%s] %s\n", __FILE__, str_macro(WITH_AC_SYNC_INT));
    Usart1Send(str);
    Wait_ms(30);    
#endif
#ifdef WITH_OVERCURRENT_SHUTDOWN
    sprintf(str,"[%s] %s\n", __FILE__, str_macro(WITH_OVERCURRENT_SHUTDOWN));
    Usart1Send(str);
    Wait_ms(30);    
#endif
    
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
