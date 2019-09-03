//---------------------------------------------
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ## @CPU:    STM32F030
// ##
// #### HARD.H ################################
//---------------------------------------------
#ifndef _HARD_H_
#define _HARD_H_

//--- Defines For Configuration ----------------------------
//--- Hardware Board Version -------------------------------
#define VER_1_0    //version original

#define VOUT_SETPOINT    V_200V
#define I_SETPOINT    CURRENT_ABOVE_EXPECTED
#define CURRENT_ABOVE_EXPECTED    900
#define CURRENT_EXTREAMLY_HIGH    1000

#define VBIAS_HIGH    VBIAS_25V
#define VBIAS_LOW     VBIAS_08V
#define VBIAS_START   VBIAS_10V

#define VLINE_START_THRESHOLD    V_120V
#define VLINE_STOP_THRESHOLD    V_80V
#define VLINE_ZERO_THRESHOLD    V_10V
#define VOUT_MAX_THRESHOLD    V_450V
#define VOUT_MIN_THRESHOLD    V_300V    //debiera ser la maxima tension que permito en vline


//--- Configuration for Hardware Versions ------------------
#ifdef VER_2_0
#define HARDWARE_VERSION_2_0
#define SOFTWARE_VERSION_2_0
#endif

#ifdef VER_1_0
#define HARDWARE_VERSION_1_0
#define SOFTWARE_VERSION_1_0
#endif

// SOFTWARE Features -------------------------
//-- Types of programs ----------
// #define DRIVER_MODE_VOUT_FIXED
// #define DRIVER_MODE_VOUT_BOOSTED
// #define HARD_TEST_MODE_STATIC_PWM
// #define HARD_TEST_MODE_DYNAMIC_PWM1
// #define HARD_TEST_MODE_DYNAMIC_PWM2
// #define HARD_TEST_MODE_ADC_SENSE
// #define HARD_TEST_MODE_INT_WITH_PWM
#define HARD_TEST_MODE_DSP_FILTERS
// #define HARD_TEST_MODE_RECT_SINUSOIDAL


//-- Types of led indications ----------
// #define USE_LED_FOR_MAIN_STATES
#define USE_LED_AS_TIM1_CH3
// #define USE_TIM_OUTPUTS_OPEN_DRAIN

//-- Frequency selection ----------
// #define USE_FREQ_70KHZ    //max pwm: 686
#define USE_FREQ_48KHZ    //max pwm: 1000

//-- Types of Interrupts ----------
#define WITH_OVERCURRENT_SHUTDOWN


//---- End of Features Configuration ----------


//--- Hardware Welcome Code ------------------//

//--- Software Welcome Code ------------------//

//-------- Others Configurations depending on the formers ------------

//-------- Hysteresis Conf ------------------------

//-------- PWM Conf ------------------------

//-------- End Of Defines For Configuration ------

#define VBIAS_25V    698
#define VBIAS_12V    346
#define VBIAS_10V    279
#define VBIAS_08V    223
//bias @12V 1.14V -> 346  ;;medido 5-7-19

//input voltage and otput voltage witth the same divider; modif date 27-7-19
//resist divider 33k//27k and 1M + 1M
//divider: 135.7
#define V_10V    23
#define V_80V    182
#define V_100V    228
#define V_120V    274
#define V_160V    365
#define V_200V    457
#define V_240V    548
#define V_300V    685
#define V_350V    799
#define V_400V    914
#define V_450V    1004

    

#if (defined USE_FREQ_70KHZ)
#define SOFT_START_CNT_ROOF    140
#elif (defined USE_FREQ_48KHZ)
#define SOFT_START_CNT_ROOF    96
#else
#error "select FREQ on hard.h"
#endif

//------- PIN CONFIG ----------------------
#ifdef VER_1_0
//GPIOA pin0	Sense_BOOST
//GPIOA pin1	Sense_BAT
//GPIOA pin2	Sense_PWR_36V

//GPIOA pin3	
//GPIOA pin4	
//GPIOA pin5    NC

//GPIOA pin6    TIM3_CH1 (CTRL_Q1)

//GPIOA pin7    
//GPIOB pin0    
//GPIOB pin1	NC

//GPIOA pin8    TIM3_CH1 (CTRL_Q2)

//GPIOA pin9    NC

//GPIOA pin10	LED
#define LED    ((GPIOA->ODR & 0x0400) != 0)
#define LED_ON    (GPIOA->BSRR = 0x00000400)
#define LED_OFF    (GPIOA->BSRR = 0x04000000)

//GPIOA pin11    NC

//GPIOA pin12	CTROL_SW
#define CTRL_SW    ((GPIOA->ODR & 0x1000) != 0)
#define CTRL_SW_ON    GPIOA->BSRR = 0x00001000)
#define CTRL_SW_OFF    (GPIOA->BSRR = 0x10000000)

//GPIOA pin13	
//GPIOA pin14	
//GPIOA pin15    NC

//GPIOB pin3	
//GPIOB pin4	NC

//GPIOB pin5
#define PROT_Q2 ((GPIOB->IDR & 0x0020) != 0)

//GPIOB pin6
#define PROT_Q1 ((GPIOB->IDR & 0x0040) != 0)

//GPIOB pin7	NC
#endif

//------- END OF PIN CONFIG -------------------


//BOARD States
typedef enum
{
    POWER_UP = 0,
    SOFT_START,
    VOLTAGE_MODE,
    CURRENT_MODE,
    OUTPUT_OVERVOLTAGE,
    INPUT_OVERVOLTAGE,
    INPUT_BROWNOUT,
    PEAK_OVERCURRENT,
    BIAS_OVERVOLTAGE,
    AUTO_RESTART,
    POWER_DOWN
    
} board_states_t;


//ESTADOS DEL LED
typedef enum
{    
    START_BLINKING = 0,
    WAIT_TO_OFF,
    WAIT_TO_ON,
    WAIT_NEW_CYCLE
} led_state_t;


//Estados Externos de LED BLINKING
#define LED_NO_BLINKING               0
#define LED_STANDBY                   1
#define LED_VOLTAGE_MODE              2
#define LED_CURRENT_MODE              3
#define LED_JUMPER_PROTECTED          4
#define LED_VIN_ERROR                 5
#define LED_OVERCURRENT_POS           6
#define LED_OVERCURRENT_NEG           7



/* Module Functions ------------------------------------------------------------*/
void ChangeLed (unsigned char);
void UpdateLed (void);

unsigned char Hard_Update_Vline (unsigned short);
unsigned short Hard_Get_Vline_Peak (void);
unsigned char Hard_Get_Vline_Conduction_Angle (void);

#endif /* _HARD_H_ */
