//---------------------------------------------
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ## @CPU:    STM32F030
// ##
// #### GPIO.C ################################
//---------------------------------------------

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "gpio.h"
#include "hard.h"



//--- Private typedef ---//
//--- Private define ---//
//--- Private macro ---//
//--- Private variables ---//
//--- Private function prototypes ---//
//--- Private functions ---//

//-------------------------------------------//
// @brief  GPIO configure.
// @param  None
// @retval None
//------------------------------------------//
void GPIO_Config (void)
{
    unsigned long temp;

    //--- MODER ---//
    //00: Input mode (reset state)
    //01: General purpose output mode
    //10: Alternate function mode
    //11: Analog mode

    //--- OTYPER ---//
    //These bits are written by software to configure the I/O output type.
    //0: Output push-pull (reset state)
    //1: Output open-drain

    //--- ORSPEEDR ---//
    //These bits are written by software to configure the I/O output speed.
    //x0: Low speed.
    //01: Medium speed.
    //11: High speed.
    //Note: Refer to the device datasheet for the frequency.

    //--- PUPDR ---//
    //These bits are written by software to configure the I/O pull-up or pull-down
    //00: No pull-up, pull-down
    //01: Pull-up
    //10: Pull-down
    //11: Reserved


    //--- GPIO A ---//
    if (!GPIOA_CLK)
        GPIOA_CLK_ON;

#ifdef USE_LED_AS_TIM1_CH3    
    temp = GPIOA->MODER;	//2 bits por pin
    temp &= 0xFCCCCFC0;		//PA0 - PA2 analog input; PA6 alternative (TIM3 CH1)
    temp |= 0x0122203F;		//PA8 alternative (TIM1 CH1); PA10 alternative (TIM CH3) PA12 output
    GPIOA->MODER = temp;
#else
    temp = GPIOA->MODER;	//2 bits por pin
    temp &= 0xFCCCCFC0;		//PA0 - PA2 analog input; PA6 alternative (TIM3 CH1)
    temp |= 0x0112203F;		//PA8 alternative (TIM1 CH1); PA10 PA12 output
    GPIOA->MODER = temp;
#endif

    temp = GPIOA->OTYPER;	//1 bit por pin
    temp &= 0xFFFFFEBF;         //PA6 PA8 open drain
    temp |= 0x00000140;
    GPIOA->OTYPER = temp;

    temp = GPIOA->OSPEEDR;	//2 bits por pin
    temp &= 0xFCCCCFFF;
    temp |= 0x00000000;		//PA6 PA8 PA10 PA12 low speed
    GPIOA->OSPEEDR = temp;

    temp = GPIOA->PUPDR;	//2 bits por pin
    temp &= 0xFFFFFFFF;
    temp |= 0x00000000;	
    GPIOA->PUPDR = temp;


    //--- GPIO B ---//
#ifdef GPIOB_ENABLE
    if (!GPIOB_CLK)
        GPIOB_CLK_ON;

    temp = GPIOB->MODER;	//2 bits por pin
    temp &= 0xFFFC3FFF;		//PB5 PB6 input mode
    temp |= 0x00000000;
    GPIOB->MODER = temp;

    temp = GPIOB->OTYPER;	//1 bit por pin
    temp &= 0xFFFFFFFF;
    temp |= 0x00000000;
    GPIOB->OTYPER = temp;

    temp = GPIOB->OSPEEDR;	//2 bits por pin
    temp &= 0xFFFFFFFF;
    temp |= 0x00000000;		//low speed
    GPIOB->OSPEEDR = temp;

    temp = GPIOB->PUPDR;	//2 bits por pin
    temp &= 0xFFFFFFFF;		//pull up
    temp |= 0x00000000;
    GPIOB->PUPDR = temp;

#endif

#ifdef GPIOF_ENABLE

    //--- GPIO F ---//
    if (!GPIOF_CLK)
        GPIOF_CLK_ON;

    temp = GPIOF->MODER;
    temp &= 0xFFFFFFFF;
    temp |= 0x00000000;
    GPIOF->MODER = temp;

    temp = GPIOF->OTYPER;
    temp &= 0xFFFFFFFF;
    temp |= 0x00000000;
    GPIOF->OTYPER = temp;

    temp = GPIOF->OSPEEDR;
    temp &= 0xFFFFFFFF;
    temp |= 0x00000000;
    GPIOF->OSPEEDR = temp;

    temp = GPIOF->PUPDR;
    temp &= 0xFFFFFFFF;
    temp |= 0x00000000;
    GPIOF->PUPDR = temp;

#endif

#ifdef WITH_OVERCURRENT_SHUTDOWN
    //Interrupt in PB5 and PB6
    if (!SYSCFG_CLK)
        SYSCFG_CLK_ON;

    SYSCFG->EXTICR[1] = 0x00000000; //Select Port A & Pin4 Pin5  external interrupt
    // EXTI->IMR |= 0x00000030; 			//Corresponding mask bit for interrupts EXTI4 EXTI5
    EXTI->EMR |= 0x00000000; 			//Corresponding mask bit for events
    EXTI->RTSR |= 0x00000030; 			//pin4 pin5 Interrupt line on rising edge
    EXTI->FTSR |= 0x00000000; 			//Interrupt line on falling edge

    NVIC_EnableIRQ(EXTI4_15_IRQn);
    NVIC_SetPriority(EXTI4_15_IRQn, 2);    
#endif    

}

#ifdef WITH_OVERCURRENT_SHUTDOWN
inline void EXTIOff (void)
{
    EXTI->IMR &= ~0x00000030;
}

inline void EXTIOn (void)
{
    EXTI->IMR |= 0x00000030;
}
#endif




//--- end of file ---//
