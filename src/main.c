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
#ifdef ADC_WITH_INT
volatile unsigned char seq_ready = 0;
#endif

// -- Externals for the timers -----------------------------
volatile unsigned short timer_led = 0;

// -- Externals used for analog or digital filters ---------
// volatile unsigned short take_temp_sample = 0;




// Globals -------------------------------------------------
volatile unsigned char overcurrent_shutdown = 0;
//  for the filters
ma16_u16_data_obj_t sense_bat_data_filter;
ma16_u16_data_obj_t sense_boost_data_filter;
ma16_u16_data_obj_t sense_pwr_36_data_filter;
//  for the pid controllers
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
void Shutdown_Mosfets (void);
#ifdef WITH_OVERCURRENT_SHUTDOWN
extern void EXTI4_15_IRQHandler(void);
#endif


//-------------------------------------------//
// @brief  Main program.
// @param  None
// @retval None
//------------------------------------------//
int main(void)
{
    board_states_t board_state = POWER_UP;
    unsigned char soft_start_cnt = 0;
    unsigned char undersampling = 0;

    unsigned short sense_bat_filtered = 0; 
    unsigned short sense_boost_filtered = 0;
    unsigned short sense_pwr_36_filtered = 0;

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

            for (unsigned char i = 0; i < 255; i++)
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
    Shutdown_Mosfets();
    
    //ADC and DMA configuration
    AdcConfig();
    DMAConfig();
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    ADC1->CR |= ADC_CR_ADSTART;
    //end of ADC & DMA

#ifdef HARD_TEST_MODE_STATIC_PWM
    UpdateTIMSync(50);
    CTRL_LED(DUTY_50_PERCENT);
    while (1);
#endif


#ifdef HARD_TEST_MODE_DYNAMIC_PWM
    CTRL_LED(DUTY_50_PERCENT);

    while (1)
    {
        for (unsigned short i = 0; i < 450; i++)
        {
            UpdateTIMSync(i);
            Wait_ms (11);
        }

        for (unsigned short i = 450; i > 0; i--)
        {
            UpdateTIMSync(i);
            Wait_ms (11);
        }
    }
#endif
    
#ifdef HARD_TEST_MODE_ADC_SENSE
#ifndef USE_LED_AS_TIM1_CH3
#error "we need USE_LED_AS_TIM1_CH3 in hard.h"
#endif
    //disable pwm
    UpdateTIMSync(DUTY_NONE);

    //to use Sense_BOOST use the CTRL_SW
    CTRL_SW_ON;    //for Sense_BOOST
    while (1)
    {
        if (sequence_ready)
        {
            sequence_ready_reset;
            // CTRL_LED(Sense_BAT);
            CTRL_LED(Sense_BOOST);
            // CTRL_LED(Sense_PWR_36V);
        }
    }
#endif

#ifdef HARD_TEST_MODE_INT_WITH_PWM
#ifdef USE_LED_AS_TIM1_CH3
#error "we dont need USE_LED_AS_TIM1_CH3 in hard.h"
#endif
#ifndef WITH_OVERCURRENT_SHUTDOWN
#error "we need WITH_OVERCURRENT_SHUTDOWN in hard.h for this test"
#endif
    //enable pwm at 10%
    UpdateTIMSync(DUTY_10_PERCENT);

    //enable ints
    EXTIOn();
    
    while (1)
    {
        if (sequence_ready)
        {
            sequence_ready_reset;
        }
    }
#endif

#ifdef SOFT_TEST_MODE_PID
    //start the pid data for controllers
    PID_Flush_Errors(&voltage_pid);
    voltage_pid.kp = 0;
    voltage_pid.ki = 1;
    voltage_pid.kd = 0;
    voltage_pid.setpoint = 100;
    
    short output [256] = { 0 };
    unsigned short input [256] = {[0 ... 127] = 100, [128 ... 255] = 101 };
    for (unsigned short i = 0; i < 256; i++)
    {
        voltage_pid.sample = input[i];
        output[i] = PID(&voltage_pid);
        LED_TOGGLE;
    }

    //dummy assigment
    for (unsigned short i = 0; i < 256; i++)
    {
        if (output[i] != input[i])
            i = 256;
    }

    while (1);
#endif
    
    
    
    //--- Production Program ----------
#ifdef BOOST_MODE
    //start the circular filters
    MA16_U16Circular_Reset(&sense_bat_data_filter); 
    MA16_U16Circular_Reset(&sense_boost_data_filter);
    MA16_U16Circular_Reset(&sense_pwr_36_data_filter);

    //start the pid data for controllers
    PID_Small_Ki_Flush_Errors(&voltage_pid);

#ifdef USE_PWM_WITH_DITHER
    voltage_pid.kp = 1 << 3;
    voltage_pid.ki = 3 << 3;    //necesito error mayor a 3 por definicion en el pwm    
    voltage_pid.kd = 0;
#endif
#ifdef USE_PWM_NO_DITHER
    voltage_pid.kp = 1;
    voltage_pid.ki = 3;    //necesito error mayor a 3 por definicion en el pwm    
    voltage_pid.kd = 0;
#endif

    unsigned short battery_min = 0;
    unsigned short battery_to_reconnect = 0;

#if defined VER_1_1
    if (PIN_BATTERY_SELECT)
    {
        //car battery
        battery_min = BATTERY_MIN_CAR;
        battery_to_reconnect = BATTERY_TO_RECONNECT_CAR;
    }
    else
    {
        //bi mount battery
        battery_min = BATTERY_MIN_BI_MOUNT;
        battery_to_reconnect = BATTERY_TO_RECONNECT_BI_MOUNT;
    }
#elif defined VER_1_0
#if defined CAR_BATTERY
    //car battery
    battery_min = BATTERY_MIN_CAR;
    battery_to_reconnect = BATTERY_TO_RECONNECT_CAR;
#elif defined BI_MOUNT_BATTERY
    //bi mount battery
    battery_min = BATTERY_MIN_BI_MOUNT;
    battery_to_reconnect = BATTERY_TO_RECONNECT_BI_MOUNT;
#else
#error "Select Type of Battery for VER_1_0 on hard.h"
#endif
#else
#error "Select the Board Version on hard.h"
#endif

    //timer to power up
    ChangeLed(LED_POWER_UP);
    timer_standby = 10;
    while (1)
    {
        //the most work involved is sample by sample
        if (sequence_ready)
        {
            sequence_ready_reset;

            //filters
            sense_bat_filtered = MA16_U16Circular(&sense_bat_data_filter, Sense_BAT);
            sense_boost_filtered = MA16_U16Circular(&sense_boost_data_filter, Sense_BOOST);
            sense_pwr_36_filtered = MA16_U16Circular(&sense_pwr_36_data_filter, Sense_PWR_36V);
            
            switch (board_state)
            {
            case POWER_UP:
                //the filters completes their action on 16 * 1/24KHz = 666us
                if (!timer_standby)
                {
                    if ((sense_pwr_36_filtered > MIN_PWR_36V) &&
                        (sense_pwr_36_filtered < MAX_PWR_36V))
                    {
                        board_state = TO_SUPPLY_BY_MAINS;
                    }
                    else if (sense_bat_filtered > battery_to_reconnect)
                    {
                        board_state = TO_SUPPLY_BY_BATTERY;
                    }
                }
                break;

            case TO_SUPPLY_BY_MAINS:
                //reseteo del PID y control del mosfet
                d = 0;
                PID_Small_Ki_Flush_Errors(&voltage_pid);
                Shutdown_Mosfets();

                //paso a alimentar desde la entrada de 36V
                CTRL_SW_OFF;
                board_state = SUPPLY_BY_MAINS;
                ChangeLed(LED_SUPPLY_BY_MAINS);
                timer_standby = 1000;    //doy algo de tiempo al relay
                break;
                
            case SUPPLY_BY_MAINS:
                if (!timer_standby)
                {
                    //input undervoltage or overvoltage
                    if ((sense_pwr_36_filtered < MIN_PWR_36V) ||
                        (sense_pwr_36_filtered > MAX_PWR_36V)) 
                    {
                        timer_standby = 1000;
                        board_state = POWER_UP;
                        ChangeLed(LED_POWER_UP);
                        soft_start_cnt = 0;
                    }
                }
                break;

            case TO_SUPPLY_BY_BATTERY:
                CTRL_SW_ON;
                board_state = SUPPLY_BY_BATTERY;
                ChangeLed(LED_SUPPLY_BY_BATTERY);
#ifdef USE_PWM_WITH_DITHER
                EnableDitherInterrupt;
#endif
                break;

            case SUPPLY_BY_BATTERY:
                soft_start_cnt++;
                
                //check to not go overvoltage
                if (sense_boost_filtered < VOUT_FOR_SOFT_START)
                {
                    //do a soft start cheking the voltage
                    if (soft_start_cnt > SOFT_START_CNT_ROOF)    //update 200us aprox.
                    {
                        soft_start_cnt = 0;

#ifdef USE_PWM_WITH_DITHER                        
                        if (d < DUTY_FOR_DMAX)    //uso d sin multiplicar
                        {
                            d++;
                            TIM_LoadDitherSequences(d << 3);                            
                        }
                        else
                        {
                            //update PID
                            voltage_pid.last_d = d << 3;
                            ChangeLed(LED_VOLTAGE_MODE);
                            board_state = VOLTAGE_MODE;
                        }
#endif
#ifdef USE_PWM_NO_DITHER                        
                        if (d < DUTY_FOR_DMAX)
                        {
                            d++;
                            CTRL_MOSFET(d);
                        }
                        else
                        {
                            //update PID
                            voltage_pid.last_d = d;
                            ChangeLed(LED_VOLTAGE_MODE);
                            board_state = VOLTAGE_MODE;
                        }
#endif                        
                    }
                }
                else
                {
#ifdef USE_PWM_WITH_DITHER                    
                    //update PID
                    voltage_pid.last_d = d << 3;
                    ChangeLed(LED_VOLTAGE_MODE);
                    board_state = VOLTAGE_MODE;
#endif
#ifdef USE_PWM_NO_DITHER
                    //update PID
                    voltage_pid.last_d = d;
                    ChangeLed(LED_VOLTAGE_MODE);
                    board_state = VOLTAGE_MODE;
#endif
                }
                
                break;
                
            case VOLTAGE_MODE:
                if (undersampling > (UNDERSAMPLING_TICKS - 1))
                {
                    undersampling = 0;
                    voltage_pid.setpoint = VOUT_SETPOINT;
                    voltage_pid.sample = sense_boost_filtered;    //only if undersampling > 16
#ifdef USE_PWM_WITH_DITHER
                    d = PID_Small_Ki(&voltage_pid);

                    if (d > 0)
                    {
                        if (d > DUTY_FOR_DMAX_WITH_DITHER)
                        {
                            d = DUTY_FOR_DMAX_WITH_DITHER;
                            voltage_pid.last_d = DUTY_FOR_DMAX_WITH_DITHER;
                        }
                    }
                    else
                    {
                        d = 0;
                        voltage_pid.last_d = 0;
                    }
                    
                    TIM_LoadDitherSequences(d);
#endif
#ifdef USE_PWM_NO_DITHER
                    d = PID_Small_Ki(&voltage_pid);

                    if (d > 0)
                    {
                        if (d > DUTY_FOR_DMAX)
                        {
                            d = DUTY_FOR_DMAX;
                            voltage_pid.last_d = DUTY_FOR_DMAX;
                        }
                    }
                    else
                    {
                        d = 0;
                        voltage_pid.last_d = 0;
                    }
                    
                    CTRL_MOSFET(d);
#endif
                }
                else
                    undersampling++;

                //si vuelve la alimentacion principal
                if ((sense_pwr_36_filtered > MIN_PWR_36V) &&
                    (sense_pwr_36_filtered < MAX_PWR_36V))
                {
                    board_state = TO_SUPPLY_BY_MAINS;
                }

                //si baja demasiado la bateria
                if (sense_bat_filtered < battery_min)
                {
                    //reseteo del PID y control del mosfet
                    d = 0;
                    PID_Small_Ki_Flush_Errors(&voltage_pid);
                    Shutdown_Mosfets();

                    CTRL_SW_OFF;
                    board_state = INPUT_BROWNOUT;
                    ChangeLed(LED_INPUT_BROWNOUT);
                    timer_standby = 1000;
                }
                break;
            
            case INPUT_OVERVOLTAGE:
                break;

            case OUTPUT_OVERVOLTAGE:
                if (sense_boost_filtered < VOUT_SETPOINT)
                {
                    board_state = VOLTAGE_MODE;
                    ChangeLed(LED_VOLTAGE_MODE);
#ifdef USE_PWM_WITH_DITHER
                    EnableDitherInterrupt;
#endif
                    
                }
                break;
                
            case INPUT_BROWNOUT:
                if (!timer_standby)
                {
                    //si vuelve la alimentacion principal
                    if ((sense_pwr_36_filtered > MIN_PWR_36V) &&
                        (sense_pwr_36_filtered < MAX_PWR_36V))
                    {
                        board_state = TO_SUPPLY_BY_MAINS;
                    }

                    //si se recupera la bateria
                    if (sense_bat_filtered > battery_to_reconnect)
                    {
                        board_state = TO_SUPPLY_BY_BATTERY;
                    }
                }
                break;
            
            case PEAK_OVERCURRENT:
                if (!timer_standby)
                {
                    board_state = POWER_UP;
                }
                break;

            case BIAS_OVERVOLTAGE:
                break;            

            case POWER_DOWN:
                break;

            }

            //
            //The things that are directly attached to the samples period
            //
        }    //end if sequence

        //
        //The things that are not directly attached to the samples period
        //
        if ((board_state != OUTPUT_OVERVOLTAGE) &&
            (sense_boost_filtered > VOUT_MAX_THRESHOLD))
        {
            Shutdown_Mosfets();
            board_state = OUTPUT_OVERVOLTAGE;
            ChangeLed(LED_OUTPUT_OVERVOLTAGE);
        }

        //for overcurrent
        if (overcurrent_shutdown > 40)
        {
            Shutdown_Mosfets();
            board_state = PEAK_OVERCURRENT;
            ChangeLed(LED_PEAK_OVERCURRENT);
            overcurrent_shutdown = 0;
            timer_standby = 2000;
        }

#ifdef USE_LED_FOR_MAIN_STATES
        UpdateLed();
#endif
        
    }    //end while 1
    
#endif    // BOOST_MODE
    
    return 0;
}

//--- End of Main ---//

void Shutdown_Mosfets (void)
{
#ifdef USE_PWM_WITH_DITHER
    DisableDitherInterrupt;
    CTRL_MOSFET(DUTY_NONE);
    TIM_LoadDitherSequences(0);
#endif
#ifdef USE_PWM_NO_DITHER
    CTRL_MOSFET(DUTY_NONE);
#endif
}

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

}


#ifdef WITH_OVERCURRENT_SHUTDOWN

#define PROT_Q1_Int        (EXTI->PR & 0x00000040)
#define PROT_Q1_Ack        (EXTI->PR |= 0x00000040)
#define PROT_Q2_Int        (EXTI->PR & 0x00000020)
#define PROT_Q2_Ack        (EXTI->PR |= 0x00000020)

void EXTI4_15_IRQHandler(void)
{
// #ifdef HARD_TEST_MODE_INT_WITH_PWM
//     if (PROT_Q1_Int)
//     {
//         LED_ON;
//         PROT_Q1_Ack;    //before this 800ns from the event
//                         //after this 1.04us from the event
//     }

//     if (PROT_Q2_Int)
//     {
//         LED_OFF;
//         PROT_Q2_Ack;
//     }
    
//     // if (LED)
//     //     LED_OFF;
//     // else
//     //     LED_ON;
// #endif

    if (PROT_Q1_Int)
    {
        DisablePreload_Mosfet_Q1;
        Mosfet_Q1_Shutdown;
        EnablePreload_Mosfet_Q1;
        overcurrent_shutdown++;
        
        PROT_Q1_Ack;    //before this 800ns from the event
                        //after this 1.04us from the event
    }

    if (PROT_Q2_Int)
    {
        DisablePreload_Mosfet_Q2;
        Mosfet_Q2_Shutdown;
        EnablePreload_Mosfet_Q2;
        overcurrent_shutdown++;

        PROT_Q2_Ack;
    }
    
}

#endif    //WITH_OVERCURRENT_SHUTDOWN

//--- end of file ---//
