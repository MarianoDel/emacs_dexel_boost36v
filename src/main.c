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

    // EnablePreload_Mosfet_Q1;
    // EnablePreload_Mosfet_Q2;
    
    // MA32Circular_Reset();
    
    UpdateTIMSync(DUTY_NONE);
    
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


    
    
    //--- Production Program ----------
#ifdef BOOST_MODE
    //start the circular filters
    MA16_U16Circular_Reset(&sense_bat_data_filter); 
    MA16_U16Circular_Reset(&sense_boost_data_filter);
    MA16_U16Circular_Reset(&sense_pwr_36_data_filter);

    //start the pid data for controllers
    PID_Flush_Errors(&voltage_pid);
    voltage_pid.kp = 1;
    // voltage_pid.ki = 43;    //necesito error mayor a 3 por definicion en el pwm
    voltage_pid.ki = 42;    //necesito error mayor a 3 por definicion en el pwm    
    voltage_pid.kd = 0;

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
                    else if (sense_bat_filtered > BATTERY_TO_RECONNECT)
                    {
                        board_state = TO_SUPPLY_BY_BATTERY;
                    }
                }
                break;

            case TO_SUPPLY_BY_MAINS:
                //reseteo del PID y control del mosfet
                d = 0;
                PID_Flush_Errors(&voltage_pid);
                CTRL_MOSFET(0);                

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
                break;

            case SUPPLY_BY_BATTERY:
                soft_start_cnt++;
                
                //check to not go overvoltage
                if (sense_boost_filtered < VOUT_SETPOINT)
                {
                    //do a soft start cheking the voltage
                    if (soft_start_cnt > SOFT_START_CNT_ROOF)    //update 200us aprox.
                    {
                        soft_start_cnt = 0;
                    
                        if (d < DUTY_FOR_DMAX)
                        {
                            d++;
                            CTRL_MOSFET(d);
                        }
                        else
                        {
                            ChangeLed(LED_VOLTAGE_MODE);
                            board_state = VOLTAGE_MODE;
                        }
                    }
                }
                else
                {
                    ChangeLed(LED_VOLTAGE_MODE);
                    board_state = VOLTAGE_MODE;
                }
                
                break;
                
            case VOLTAGE_MODE:
                if (undersampling > (UNDERSAMPLING_TICKS - 1))
                {
                    undersampling = 0;
                    voltage_pid.setpoint = VOUT_SETPOINT;
                    voltage_pid.sample = sense_boost_filtered;    //only if undersampling > 16
                    d = PID(&voltage_pid);

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
                if (sense_bat_filtered < BATTERY_MIN)
                {
                    //reseteo del PID y control del mosfet
                    d = 0;
                    PID_Flush_Errors(&voltage_pid);
                    CTRL_MOSFET(0);                

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
                    if (sense_bat_filtered > BATTERY_TO_RECONNECT)
                    {
                        board_state = TO_SUPPLY_BY_BATTERY;
                    }
                }
                break;
            
            case PEAK_OVERCURRENT:
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
            CTRL_MOSFET(DUTY_NONE);
            board_state = OUTPUT_OVERVOLTAGE;
            ChangeLed(LED_OUTPUT_OVERVOLTAGE);
        }

#ifdef USE_LED_FOR_MAIN_STATES
        UpdateLed();
#endif
        
    }    //end while 1
    
#endif    // BOOST_MODE
    
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

}


#ifdef WITH_OVERCURRENT_SHUTDOWN

#define PROT_Q1_Int        (EXTI->PR & 0x00000040)
#define PROT_Q1_Ack        (EXTI->PR |= 0x00000040)
#define PROT_Q2_Int        (EXTI->PR & 0x00000020)
#define PROT_Q2_Ack        (EXTI->PR |= 0x00000020)

void EXTI4_15_IRQHandler(void)
{
#ifdef HARD_TEST_MODE_INT_WITH_PWM
    if (PROT_Q1_Int)
    {
        LED_ON;
        PROT_Q1_Ack;    //before this 800ns from the event
                        //after this 1.04us from the event
    }

    if (PROT_Q2_Int)
    {
        LED_OFF;
        PROT_Q2_Ack;
    }
    
    // if (LED)
    //     LED_OFF;
    // else
    //     LED_ON;
#endif
    
}

#endif    //WITH_OVERCURRENT_SHUTDOWN

//--- end of file ---//
