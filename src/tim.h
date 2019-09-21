//---------------------------------------------
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ## @CPU:    STM32F103
// ##
// #### TIM.H ################################
//---------------------------------------------
#ifndef _TIM_H_
#define _TIM_H_

//--- Incuded for help defines ---//
#include "hard.h"
#include "stm32f0xx.h"

//--- Exported types ---//
//--- Exported constants ---//
#define DUTY_NONE		(DUTY_00_PERCENT)
#define DUTY_FOR_DMAX           (DUTY_85_PERCENT)
#define DUTY_50_PERCENT_PLUS_ONE    (DUTY_50_PERCENT + 1)
#define DUTY_ALWAYS        (DUTY_100_PERCENT + 1)
#if (defined USE_FREQ_48KHZ)
#define DUTY_00_PERCENT        0
#define DUTY_5_PERCENT		50
#define DUTY_10_PERCENT		100
#define DUTY_20_PERCENT		200
#define DUTY_50_PERCENT		500
#define DUTY_80_PERCENT		800
#define DUTY_85_PERCENT		850
#define DUTY_100_PERCENT        1000
#elif (defined USE_FREQ_70KHZ)
#define DUTY_5_PERCENT		34
#define DUTY_10_PERCENT		68
#define DUTY_50_PERCENT		343
#define DUTY_100_PERCENT        686
#else
#error "No FREQ selected for TIM3 on hard.h"
#endif

#define DUTY_FB_25A    395    //esto es 1.17V que equivale a 25Apico en el primario

#define ENABLE_TIM1			TIM1->CR1 |= TIM_CR1_CEN;
#define DISABLE_TIM1			TIM1->CR1 &= ~TIM_CR1_CEN;

#define ENABLE_TIM3			TIM3->CR1 |= TIM_CR1_CEN;
#define DISABLE_TIM3			TIM3->CR1 &= ~TIM_CR1_CEN;



//--- Exported macro ---//
#define RCC_TIM1_CLK 		(RCC->APB2ENR & 0x00000800)
#define RCC_TIM1_CLK_ON 	RCC->APB2ENR |= 0x00000800
#define RCC_TIM1_CLK_OFF 	RCC->APB2ENR &= ~0x00000800

#define RCC_TIM3_CLK 		(RCC->APB1ENR & 0x00000002)
#define RCC_TIM3_CLK_ON 	RCC->APB1ENR |= 0x00000002
#define RCC_TIM3_CLK_OFF 	RCC->APB1ENR &= ~0x00000002

#define RCC_TIM6_CLK 		(RCC->APB1ENR & 0x00000010)
#define RCC_TIM6_CLK_ON 	RCC->APB1ENR |= 0x00000010
#define RCC_TIM6_CLK_OFF 	RCC->APB1ENR &= ~0x00000010

#define RCC_TIM14_CLK 		(RCC->APB1ENR & 0x00000100)
#define RCC_TIM14_CLK_ON 	RCC->APB1ENR |= 0x00000100
#define RCC_TIM14_CLK_OFF 	RCC->APB1ENR &= ~0x00000100

#define RCC_TIM15_CLK 		(RCC->APB2ENR & 0x00010000)
#define RCC_TIM15_CLK_ON 	RCC->APB2ENR |= 0x00010000
#define RCC_TIM15_CLK_OFF 	RCC->APB2ENR &= ~0x00010000

#define RCC_TIM16_CLK 		(RCC->APB2ENR & 0x00020000)
#define RCC_TIM16_CLK_ON 	RCC->APB2ENR |= 0x00020000
#define RCC_TIM16_CLK_OFF 	RCC->APB2ENR &= ~0x00020000

#define RCC_TIM17_CLK 		(RCC->APB2ENR & 0x00040000)
#define RCC_TIM17_CLK_ON 	RCC->APB2ENR |= 0x00040000
#define RCC_TIM17_CLK_OFF 	RCC->APB2ENR &= ~0x00040000


//--- Exported wrapped functions ---//
#define CTRL_LED(X)    Update_TIM1_CH3(X)

#define EnablePreload_Mosfet_Q1    (TIM3->CR1 |= TIM_CR1_ARPE)
#define DisablePreload_Mosfet_Q1    (TIM3->CR1 &= ~TIM_CR1_ARPE)
#define UpdateTIM_Mosfet_Q1(X)    (TIM3->ARR = DUTY_50_PERCENT + (X))
#define EnablePreload_Mosfet_Q2    (TIM1->CCMR1 |= TIM_CCMR1_OC1PE)
#define DisablePreload_Mosfet_Q2    (TIM1->CCMR1 &= ~TIM_CCMR1_OC1PE)
#define UpdateTIM_Mosfet_Q2(X)    (TIM1->CCR1 = (X))

#define UpdateTIMSync(X)    do { \
    TIM1->CCR1 = (X); \
    TIM3->CCR1 = DUTY_100_PERCENT - 2 - (X); \
    } while(0)

#define CTRL_MOSFET(X)    UpdateTIMSync(X)

//--- Exported functions ---//


void TIM_1_Init(void);
void Update_TIM1_CH1 (unsigned short);
void Update_TIM1_CH3 (unsigned short);

void TIM_3_Init(void);
void TIM3_IRQHandler (void);
void Update_TIM3_CH1 (unsigned short);
void Update_TIM3_CH2 (unsigned short);
void Update_TIM3_CH3 (unsigned short);
void Update_TIM3_CH4 (unsigned short);

//Only for C8 Devices
void TIM_6_Init (void);
void TIM6Enable (void);
void TIM6Disable (void);

void TIM_14_Init(void);
void TIM14_IRQHandler (void);

void TIM_16_Init(void);
void TIM16_IRQHandler (void);
void OneShootTIM16 (unsigned short);
void TIM16Enable (void);
void TIM16Disable (void);

void TIM_17_Init(void);
void TIM17_IRQHandler (void);
void TIM17Enable (void);
void TIM17Disable (void);


void Wait_ms (unsigned short wait);
#endif
//--- End ---//


//--- END OF FILE ---//
