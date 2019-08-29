//---------------------------------------------
// ##
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


//---- Features Configuration ----------------
//features are activeted here and annouced in hard.c
#define FEATURES

// SOFTWARE Features -------------------------
//-- Types of programs ----------
// #define DRIVER_MODE_VOUT_FIXED
#define DRIVER_MODE_VOUT_BOOSTED
// #define HARD_TEST_MODE
// #define HARD_TEST_MODE_LINE_SYNC
// #define HARD_TEST_MODE_CONDUCTION_ANGLE
// #define HARD_TEST_MODE_RECT_SINUSOIDAL
// #define HARD_TEST_MODE_DISABLE_PWM
// #define HARD_TEST_MODE_ADC_SENSE

//-- Types of led indications ----------
#define USE_LED_FOR_MAIN_STATES
// #define USE_LED_FOR_MAINS_SYNC
// #define USE_LED_FOR_POSITIVE_VOLTAGE
// #define USE_LED_AS_TIM1_CH3
// #define USE_LED_FOR_SIGNAL

//-- Frequency selection ----------
// #define USE_FREQ_70KHZ    //max pwm: 686
#define USE_FREQ_48KHZ    //max pwm: 1000

//-- Types of Interrupts ----------
// #define WITH_AC_SYNC_INT
// #define WITH_OVERCURRENT_SHUTDOWN

//-- Types of Optoisolatar used ----------
#define OPTO_KB817
// #define OPTO_FOD8801


//---- End of Features Configuration ----------

//--- Stringtify Utils -----------------------
#define STRING_CONCAT(str1,str2) #str1 " " #str2
#define STRING_CONCAT_NEW_LINE(str1,str2) xstr(str1) #str2 "\n"
#define xstr_macro(s) str_macro(s)
#define str_macro(s) #s

//--- Hardware Welcome Code ------------------//
#ifdef HARDWARE_VERSION_1_0
#define HARD "Hardware V: 1.0\n"
#endif

//--- Software Welcome Code ------------------//
#ifdef SOFTWARE_VERSION_1_0
#define SOFT "Software V: 1.0\n"
#endif


//-------- Others Configurations depending on the formers ------------
#ifdef INVERTER_MODE_PURE_SINUSOIDAL
#ifndef INVERTER_MODE
#define INVERTER_MODE
#endif
#endif

#if (defined DRIVER_MODE_VOUT_FIXED) || (defined DRIVER_MODE_VOUT_BOOSTED)
#ifndef DRIVER_MODE
#define DRIVER_MODE
#endif
#endif
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

    
#if defined OPTO_KB817
#define IOUT_3A    610
#define IOUT_2A    406    //esto da 2.24A en frio 8-7-19
#define IOUT_1A    203
//Iup @2.38A 1.56V -> 484  ;;medido 5-7-2019
#elif defined OPTO_FOD8801
#define IOUT_3A    915
#define IOUT_2A    610    //esto da 2.07A en frio 10-7-19
#define IOUT_1A    305
#else
#error "define the opto in hard.h"
#endif

#if (defined USE_FREQ_70KHZ)
#define SOFT_START_CNT_ROOF    140
#elif (defined USE_FREQ_48KHZ)
#define SOFT_START_CNT_ROOF    96
#else
#error "select FREQ on hard.h"
#endif

//------- PIN CONFIG ----------------------
#ifdef VER_1_0
//GPIOA pin0	Vbias_Sense
//GPIOA pin1	Vup
//GPIOA pin2	I_Sense
//GPIOA pin3	Iup
//GPIOA pin4	V220_Sense, Vline_Sense

//GPIOA pin5    NC

//GPIOA pin6    TIM3_CH1 (CTRL_MOSFET)

//GPIOA pin7    
//GPIOB pin0    
//GPIOB pin1	

//GPIOA pin8
#define LEDR ((GPIOA->ODR & 0x0100) != 0)
#define LEDR_ON	GPIOA->BSRR = 0x00000100
#define LEDR_OFF GPIOA->BSRR = 0x01000000

//GPIOA pin9
#define LEDG ((GPIOA->ODR & 0x0200) != 0)
#define LEDG_ON	GPIOA->BSRR = 0x00000200
#define LEDG_OFF GPIOA->BSRR = 0x02000000

//GPIOA pin10	LED
#define LED ((GPIOA->ODR & 0x0400) != 0)
#define LED_ON	GPIOA->BSRR = 0x00000400
#define LED_OFF GPIOA->BSRR = 0x04000000

//GPIOA pin11	
//GPIOA pin12	
//GPIOA pin13	
//GPIOA pin14	
//GPIOA pin15    NC

//GPIOB pin3	
//GPIOB pin4	
//GPIOB pin5	
//GPIOB pin6
//GPIOB pin7	NC
#endif

//------- END OF PIN CONFIG -------------------


//DRIVER States
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
    
} driver_states_t;


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
unsigned short GetHysteresis (unsigned char);
unsigned char GetNew1to10 (unsigned short);
void UpdateVGrid (void);
void UpdateIGrid (void);
unsigned short GetVGrid (void);
unsigned short GetIGrid (void);
unsigned short PowerCalc (unsigned short, unsigned short);
unsigned short PowerCalcMean8 (unsigned short * p);
void ShowPower (char *, unsigned short, unsigned int, unsigned int);
void ChangeLed (unsigned char);
void UpdateLed (void);
unsigned short UpdateDMAX (unsigned short);
unsigned short UpdateDMAXSF (unsigned short);
unsigned short UpdateDmaxLout (unsigned short);
unsigned short VoutTicksToVoltage (unsigned short);
unsigned short VinTicksToVoltage (unsigned short);
unsigned short Hard_GetDmaxLout (unsigned short, unsigned short);
void WelcomeCodeFeatures (char *);

unsigned char Hard_Update_Vline (unsigned short);
unsigned short Hard_Get_Vline_Peak (void);
unsigned char Hard_Get_Vline_Conduction_Angle (void);

#endif /* _HARD_H_ */
