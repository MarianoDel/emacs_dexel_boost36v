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
// from output sensors
#define MIN_PWR_36V    VOLTS_32
#define MAX_PWR_36V    VOLTS_40

#define VOLTS_40    966
#define VOLTS_36    869
#define VOLTS_32    773

// from battery sensor
#define MIN_BAT_16V    BATT_10
#define MAX_BAT_16V    BATT_18

#define BATT_10    347
#define BATT_12    416
#define BATT_18    624

// where to go?
#define VOUT_SETPOINT    VOLTS_36
#define UNDERSAMPLING_TICKS    16

//--- Hardware Board Version -------------------------------
#define VER_1_0    //version original



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
#define HARD_TEST_MODE_STATIC_PWM
// #define HARD_TEST_MODE_DYNAMIC_PWM
// #define HARD_TEST_MODE_ADC_SENSE
// #define HARD_TEST_MODE_INT_WITH_PWM
// #define DRIVER_MODE_VOUT_FIXED
// #define DRIVER_MODE_VOUT_BOOSTED
// #define BOOST_MODE



//-- Types of led indications ----------
#define USE_LED_FOR_MAIN_STATES
// #define USE_LED_AS_TIM1_CH3
#define USE_TIM_OUTPUTS_OPEN_DRAIN

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


    

#if (defined USE_FREQ_70KHZ)
#define SOFT_START_CNT_ROOF    140
#elif (defined USE_FREQ_48KHZ)
#define SOFT_START_CNT_ROOF    96
#else
#error "select FREQ on hard.h"
#endif

//------- PIN CONFIG ----------------------
#ifdef VER_1_0
//GPIOA pin0	Sense_BAT
//GPIOA pin1	Sense_BOOST
//GPIOA pin2	Sense_PWR_36V

//GPIOA pin3	CTROL_SW
#define CTRL_SW    ((GPIOA->ODR & 0x0008) != 0)
#define CTRL_SW_ON    (GPIOA->BSRR = 0x00000008)
#define CTRL_SW_OFF    (GPIOA->BSRR = 0x00080000)

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

//GPIOA pin12	

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
    SUPPLY_BY_MAINS,
    SUPPLY_BY_BATTERY,
    VOLTAGE_MODE,
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
#define LED_SUPPLY_BY_MAINS              2
#define LED_SUPPLY_BY_BATTERY              3
#define LED_VOLTAGE_MODE          4
#define LED_VIN_ERROR                 5
#define LED_OVERCURRENT_POS           6
#define LED_OVERCURRENT_NEG           7
#define LED_POWER_UP    LED_STANDBY


/* Module Functions ------------------------------------------------------------*/
void ChangeLed (unsigned char);
void UpdateLed (void);

#endif /* _HARD_H_ */
