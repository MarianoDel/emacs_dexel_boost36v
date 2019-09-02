//--------------------------------------------------
// #### BOOST 100W PROJECT  F030 - Custom Board ####
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ## @CPU:    STM32F030
// ##
// #### MAIN.C #####################################
//--------------------------------------------------

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "gpio.h"
#include "tim.h"
#include "uart.h"
#include "hard.h"

#include "core_cm0.h"
#include "adc.h"
#include "dma.h"
#include "flash_program.h"
#include "dsp.h"
#include "it.h"


// Externals -----------------------------------------------

// -- Externals from or for the ADC ------------------------
volatile unsigned short adc_ch [ADC_CHANNEL_QUANTITY];
volatile unsigned char seq_ready = 0;

// -- Externals for the timers -----------------------------
volatile unsigned short timer_led = 0;

// -- Externals used for analog or digital filters ---------
// volatile unsigned short take_temp_sample = 0;




// Globals -------------------------------------------------
volatile unsigned char overcurrent_shutdown = 0;
//  for the filters
ma8_data_obj_t vline_data_filter;
ma8_data_obj_t vout_data_filter;
ma8_data_obj_t vline_peak_data_filter;
//  for the pid controllers
pid_data_obj_t current_pid;
pid_data_obj_t voltage_pid;

// ------- de los timers -------
volatile unsigned short wait_ms_var = 0;
volatile unsigned short timer_standby;
//volatile unsigned char display_timer;
// volatile unsigned short timer_meas;
volatile unsigned char timer_filters = 0;
// volatile unsigned short dmax_permited = 0;


// Module Functions ----------------------------------------
void TimingDelay_Decrement (void);
// extern void EXTI4_15_IRQHandler(void);


//-------------------------------------------//
// @brief  Main program.
// @param  None
// @retval None
//------------------------------------------//
int main(void)
{
    unsigned char i;
    unsigned short ii;

    board_states_t board_state = AUTO_RESTART;
    unsigned char soft_start_cnt = 0;
    unsigned char undersampling = 0;

    unsigned short Vout_Sense_Filtered = 0;
    unsigned short Vline_Sense_Filtered = 0;
    // unsigned short I_Sense_Filtered = 0;

    short d = 0;
    // short ez1 = 0;
    // short ez2 = 0;


    //GPIO Configuration.
    GPIO_Config();

    //ACTIVAR SYSTICK TIMER
    if (SysTick_Config(48000))
    {
        while (1)	/* Capture error */
        {
            if (LED)
                LED_OFF;
            else
                LED_ON;

            for (i = 0; i < 255; i++)
            {
                asm (	"nop \n\t"
                        "nop \n\t"
                        "nop \n\t" );
            }
        }
    }

//---------- Pruebas de Hardware --------//    
    // EXTIOff ();

    TIM_1_Init ();	   //lo utilizo para mosfet Q2 y para el LED eventualmente
    TIM_3_Init ();	   //lo utilizo para mosfet Q1 y para synchro ADC

    EnablePreload_Mosfet_Q1;
    EnablePreload_Mosfet_Q2;
    
    MA32Circular_Reset();
    
    UpdateTIMSync(DUTY_NONE);
    
    //ADC and DMA configuration
    AdcConfig();
    DMAConfig();
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    ADC1->CR |= ADC_CR_ADSTART;
    //end of ADC & DMA

#ifdef HARD_TEST_MODE_STATIC_PWM
    UpdateTIMSync(DUTY_10_PERCENT);
    CTRL_LED(DUTY_50_PERCENT);
    while (1);
#endif

#ifdef HARD_TEST_MODE_DYNAMIC_PWM
    CTRL_LED(DUTY_50_PERCENT);

    while (1)
    {
        for (unsigned short i = 0; i < 400; i += 100)
        {
            UpdateTIMSync(i);
            Wait_ms (5000);
        }

        for (unsigned short i = 400; i > 0; i -= 100)
        {
            UpdateTIMSync(i);
            Wait_ms (5000);
        }
    }
#endif
    
#ifdef HARD_TEST_MODE_RECT_SINUSOIDAL
    p_signal = mem_signal;

    while (1)
    {
        if (sequence_ready)
        {
            sequence_ready_reset;
            //aca la senial (el ultimo punto) termina en 0
            if (p_signal < &mem_signal[(SIZEOF_SIGNAL - 1)])
            {
                p_signal++;
            }
            else
            {
                p_signal = mem_signal;
#ifdef USE_LED_FOR_SIGNAL
                if (LED)
                    LED_OFF;
                else
                    LED_ON;
#endif
            }
            CTRL_MOSFET(*p_signal);
        }
    }
#endif    // HARD_TEST_MODE_RECT_SINUSOIDAL

#ifdef HARD_TEST_MODE_ADC_SENSE
    //disable pwm
    CTRL_MOSFET(DUTY_NONE);
    
    while (1)
    {
        if (sequence_ready)
        {
            sequence_ready_reset;
            // CTRL_LED(Vout_Sense);
            // CTRL_LED(Vline_Sense);
            CTRL_LED(I_Sense);
        }
    }
#endif
    
#ifdef HARD_TEST_MODE_CONDUCTION_ANGLE
    Hard_Reset_Voltage_Filter();
    
    while (1)
    {
        if (sequence_ready)
        {
            sequence_ready_reset;

            ii = Hard_Update_Voltage_Sense();

            if (ii)
            {
                ii = 0;
                CTRL_LED(Hard_Get_Conduction_Angle());
                // CTRL_LED(Hard_Get_Hidden_Value());
            }
        }
    }
#endif    // HARD_TEST_MODE_CONDUCTION_ANGLE

#ifdef HARD_TEST_MODE_LINE_SYNC
    Hard_Reset_Voltage_Filter();

    // CTRL_MOSFET(DUTY_10_PERCENT);
    // CTRL_LED(DUTY_50_PERCENT);
    while (1)
    {
        if (sequence_ready)
        {
            sequence_ready_reset;
            Hard_Update_Voltage_Filter(Vline_Sense);
        }

        Hard_Update_Voltage_Sense();
    }
#endif
    
#ifdef HARD_TEST_MODE
    ChangeLed(LED_STANDBY);
    while (1)
    {
        if (sequence_ready)
        {
            sequence_ready_reset;
            // if (LED)
            //     LED_OFF;
            // else
            //     LED_ON;
            // CTRL_MOSFET(Vbias_Sense);
            // CTRL_MOSFET(Vup);
            // CTRL_MOSFET(I_Sense);
            // CTRL_MOSFET(Iup);
            CTRL_MOSFET(V220_Sense);
        }
        UpdateLed();
    }
#endif
    
    
    //--- Production Program ----------
#ifdef DRIVER_MODE
    //start the circular filters
    MA8Circular_Reset(&vline_data_filter);
    MA8Circular_Reset(&vline_peak_data_filter);
    MA8Circular_Reset(&vout_data_filter);

    //start the pid data for controllers
    PID_Flush_Errors(&current_pid);
    current_pid.kp = 128;
    current_pid.ki = 0;
    current_pid.kd = 0;

    PID_Flush_Errors(&voltage_pid);
    voltage_pid.kp = 0;
    voltage_pid.ki = 128;
    voltage_pid.kd = 0;
    

    CTRL_MOSFET(DUTY_NONE);

    while (1)
    {
        //the most work involved is sample by sample
        if (sequence_ready)
        {
            sequence_ready_reset;

            //filters
            Vline_Sense_Filtered = MA8Circular(&vline_data_filter, Vline_Sense);
            Vout_Sense_Filtered = MA8Circular(&vout_data_filter, Vout_Sense);
            // I_Sense_Filtered = MA8Circular_I(I_Sense);
            
            switch (driver_state)
            {
            case POWER_UP:
                if (Vline_Sense_Filtered > VLINE_START_THRESHOLD)
                {
                    driver_state = SOFT_START;
                    timer_standby = 10;
                }

                break;

            case SOFT_START:
                // soft_start_cnt++;
                
                // //check to not go overvoltage
                // if (Vout_Sense_Filtered < VOUT_SETPOINT)
                // {
                //     //hago un soft start respecto de la corriente y/o tension de salida
                //     if (soft_start_cnt > SOFT_START_CNT_ROOF)    //update cada 2ms aprox.
                //     {
                //         soft_start_cnt = 0;
                    
                //         if (d < DUTY_FOR_DMAX)
                //         {
                //             d++;
                //             CTRL_MOSFET(d);
                //         }
                //         else
                //         {
                //             ChangeLed(LED_VOLTAGE_MODE);
                //             driver_state = VOLTAGE_MODE;
                //         }
                //     }
                // }
                // else
                // {
                //     ChangeLed(LED_VOLTAGE_MODE);
                //     driver_state = VOLTAGE_MODE;
                // }
                if (!timer_standby)    //doy tiempo de medio ciclo
                {
                    ChangeLed(LED_VOLTAGE_MODE);
                    driver_state = VOLTAGE_MODE;
                }                
                break;

            case AUTO_RESTART:
                CTRL_MOSFET(DUTY_NONE);
                d = 0;
                PID_Flush_Errors(&current_pid);
                PID_Flush_Errors(&voltage_pid);
                ChangeLed(LED_STANDBY);
                driver_state = POWER_UP;
                break;
        
            case VOLTAGE_MODE:
                //reviso no pasarme de corriente
                //no quiero mas de 1V en la corriente
                //1V / 3.3V * 1023 = 310
                //esto por el ciclo de trabajo me da el promedio de corriente que mido
                // current_calc = I_Sense * 1000;
                // current_calc = current_calc / d;

                // if (current_calc > 610)
                //if current is extreamly high, just stop
                if (I_Sense > CURRENT_EXTREAMLY_HIGH)
                {
                    d = DUTY_NONE;
                    CTRL_MOSFET(d);                    
                    LEDR_ON;
                    timer_standby = 10;
                    driver_state = PEAK_OVERCURRENT;
                }
                else
                {
                    //fast current loop
                    // unsigned int current_setpoint = 0;
                    
                    // current_setpoint = Vline_Sense * pfc_multiplier;
                    // current_setpoint >>= 10;

                    // current_pid.setpoint = current_setpoint;
                    // if (undersampling > UNDERSAMPLING_TICKS)
                    // {
#define MAX_CURRENT    450
                    if (Vline_Sense_Filtered < MAX_CURRENT)
                        current_pid.setpoint = Vline_Sense_Filtered;
                    else
                        current_pid.setpoint = MAX_CURRENT;
                    
                        current_pid.sample = I_Sense;
                        d = PID (&current_pid);

                        if (d > 0)    //d can be negative
                        {
                            if (d > DUTY_FOR_DMAX)
                                d = DUTY_FOR_DMAX;
                        }
                        else
                            d = DUTY_NONE;

                        CTRL_MOSFET(d);
                    // }
                    // else
                    //     undersampling++;

                }

                // if (undersampling > UNDERSAMPLING_TICKS)
                // {
                //     unsigned short boost_setpoint = 0;

                //     //40% boosted
                //     boost_setpoint = MA8Circular_Only_Calc(&vline_data_filter);
                //     boost_setpoint = boost_setpoint * 14;
                //     boost_setpoint = boost_setpoint / 10;

                //     if (boost_setpoint > Vout_Sense)
                //     {
                //         voltage_pid.setpoint = boost_setpoint;
                //         voltage_pid.sample = Vout_Sense;
                //         pfc_multiplier = PID(&voltage_pid);
                //     }
                // }
                // else
                //     undersampling++;

                break;
            
            case OUTPUT_OVERVOLTAGE:
                if (!timer_standby)
                {
                    LEDG_OFF;
                    if (Vout_Sense_Filtered < VOUT_MIN_THRESHOLD)
                        driver_state = AUTO_RESTART;
                }
                break;

            case INPUT_OVERVOLTAGE:
                if (!timer_standby)
                    driver_state = AUTO_RESTART;                
                break;

            case INPUT_BROWNOUT:
                if (!timer_standby)
                {
                    LEDG_OFF;
                    if (Vline_Sense_Filtered > VLINE_START_THRESHOLD)
                        driver_state = AUTO_RESTART;
                }
                break;
            
            case PEAK_OVERCURRENT:
                if (!timer_standby)
                {
                    LEDR_OFF;
                    driver_state = AUTO_RESTART;
                }
                break;

            case BIAS_OVERVOLTAGE:
                if (!timer_standby)
                    driver_state = AUTO_RESTART;                
                break;            

            case POWER_DOWN:
                if (!timer_standby)
                    driver_state = AUTO_RESTART;                
                break;

            }

            //
            //The things that are directly attached to the samples period
            //
            if (Hard_Update_Vline(Vline_Sense_Filtered))
            {
                //cycle_ended
                MA8Circular(&vline_data_filter, Hard_Get_Vline_Peak());
            }
        }    //end if sequence

        //
        //The things that are not directly attached to the samples period
        //
        if (Vout_Sense_Filtered > VOUT_MAX_THRESHOLD)
        {
            CTRL_MOSFET(DUTY_NONE);
            driver_state = OUTPUT_OVERVOLTAGE;
            timer_standby = 10;
            LEDG_ON;
        }

        // if ((Vline_Sense_Filtered < VLINE_STOP_THRESHOLD) &&
        //     (driver_state > POWER_UP))
        // {
        //     CTRL_MOSFET(DUTY_NONE);
        //     driver_state = INPUT_BROWNOUT;
        //     timer_standby = 20;
        //     LEDG_ON;
        // }

#ifdef USE_LED_FOR_MAIN_STATES
        UpdateLed();
#endif
        
    }    //end while 1
    
#endif    // DRIVER_MODE
    
    return 0;
}

//--- End of Main ---//


void TimingDelay_Decrement(void)
{
    if (wait_ms_var)
        wait_ms_var--;

    if (timer_standby)
        timer_standby--;

    // if (take_temp_sample)
    //     take_temp_sample--;

    // if (timer_meas)
    //     timer_meas--;

    if (timer_led)
        timer_led--;

    if (timer_filters)
        timer_filters--;

#ifdef INVERTER_ONLY_SYNC_AND_POLARITY
    if (timer_no_sync)
        timer_no_sync--;
#endif
    
    // //cuenta de a 1 minuto
    // if (secs > 59999)	//pasaron 1 min
    // {
    // 	minutes++;
    // 	secs = 0;
    // }
    // else
    // 	secs++;
    //
    // if (minutes > 60)
    // {
    // 	hours++;
    // 	minutes = 0;
    // }


}

#define AC_SYNC_Int        (EXTI->PR & 0x00000100)
#define AC_SYNC_Set        (EXTI->IMR |= 0x00000100)
#define AC_SYNC_Reset      (EXTI->IMR &= ~0x00000100)
#define AC_SYNC_Ack        (EXTI->PR |= 0x00000100)

#define AC_SYNC_Int_Rising          (EXTI->RTSR & 0x00000100)
#define AC_SYNC_Int_Rising_Set      (EXTI->RTSR |= 0x00000100)
#define AC_SYNC_Int_Rising_Reset    (EXTI->RTSR &= ~0x00000100)

#define AC_SYNC_Int_Falling          (EXTI->FTSR & 0x00000100)
#define AC_SYNC_Int_Falling_Set      (EXTI->FTSR |= 0x00000100)
#define AC_SYNC_Int_Falling_Reset    (EXTI->FTSR &= ~0x00000100)

#define OVERCURRENT_POS_Int        (EXTI->PR & 0x00000010)
#define OVERCURRENT_POS_Ack        (EXTI->PR |= 0x00000010)
#define OVERCURRENT_NEG_Int        (EXTI->PR & 0x00000020)
#define OVERCURRENT_NEG_Ack        (EXTI->PR |= 0x00000020)

void EXTI4_15_IRQHandler(void)
{
#ifdef WITH_OVERCURRENT_SHUTDOWN
    if (OVERCURRENT_POS_Int)
    {
        HIGH_LEFT(DUTY_NONE);
        //TODO: trabar el TIM3 aca!!!
        overcurrent_shutdown = 1;
        OVERCURRENT_POS_Ack;
    }

    if (OVERCURRENT_NEG_Int)
    {
        HIGH_RIGHT(DUTY_NONE);
        //TODO: trabar el TIM3 aca!!!
        overcurrent_shutdown = 2;
        OVERCURRENT_NEG_Ack;
    }
#endif
}

//--- end of file ---//
