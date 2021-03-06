//---------------------------------------------
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ## @CPU:    STM32F030
// ##
// #### TIM.C ################################
//---------------------------------------------

/* Includes ------------------------------------------------------------------*/
#include "tim.h"
#include "stm32f0xx.h"
#include "hard.h"


// Externals -------------------------------------------------------------------
extern volatile unsigned short wait_ms_var;


// Globals ---------------------------------------------------------------------
#ifdef USE_PWM_WITH_DITHER
#define SIZEOF_DITHER_VECT    8
volatile unsigned short v_dither_tim1_ch1[SIZEOF_DITHER_VECT] = { 0 };
volatile unsigned short v_dither_tim3_ch1[SIZEOF_DITHER_VECT] = { 0 };
volatile unsigned short* p_dither_tim1_ch1;
volatile unsigned short* p_dither_tim3_ch1;
#endif



//--- FUNCIONES DEL MODULO ---//
// inline void UpdateTIMSync (unsigned short a)
// {
//     //primero cargo TIM1
//     TIM1->CCR1 = a;
//     TIM3->ARR = DUTY_50_PERCENT + a;    //TIM3->CCR1 es el delay entre timers
//                                         //lo cargo en el timer init
// }

// inline void UpdateTIM_MosfetA (unsigned short a)
// {
//     TIM3->ARR = DUTY_50_PERCENT + a;    
// }

// inline void UpdateTIM_MosfetB (unsigned short a)
// {
//     TIM1->CCR1 = a;
// }

// inline void EnablePreload_MosfetA (void)
// {
//     // TIM3->CCMR1 |= TIM_CCMR1_OC1PE;
//     TIM3->CR1 |= TIM_CR1_ARPE;
// }

// inline void DisablePreload_MosfetA (void)
// {
//     // TIM3->CCMR1 &= ~TIM_CCMR1_OC1PE;
//     TIM3->CR1 &= ~TIM_CR1_ARPE;    
// }

// inline void EnablePreload_MosfetB (void)
// {
//     TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
// }

// inline void DisablePreload_MosfetB (void)
// {
//     TIM1->CCMR1 &= ~TIM_CCMR1_OC1PE;
// }

void Update_TIM1_CH3 (unsigned short a)
{
    TIM1->CCR3 = a;
}

void Update_TIM3_CH1 (unsigned short a)
{
    TIM3->CCR1 = a;
}

void Update_TIM3_CH2 (unsigned short a)
{
    TIM3->CCR2 = a;
}

void Update_TIM3_CH3 (unsigned short a)
{
    TIM3->CCR3 = a;
}

void Update_TIM3_CH4 (unsigned short a)
{
    TIM3->CCR4 = a;
}

void Update_TIM14_CH1 (unsigned short a)
{
    TIM14->CCR1 = a;
}

void Wait_ms (unsigned short wait)
{
    wait_ms_var = wait;

    while (wait_ms_var);
}

//-------------------------------------------//
// @brief  TIM configure.
// @param  None
// @retval None
//------------------------------------------//
void TIM_1_Init (void)
{
    unsigned int temp = 0;

    if (!RCC_TIM1_CLK)
        RCC_TIM1_CLK_ON;

    //Configuracion del timer.
    //TIM1->CR1 |= TIM_CR1_OPM;        //clk int / 1; upcounting; one pulse
    TIM1->CR1 = 0x00;        //clk int / 1;
    TIM1->CR2 |= TIM_CR2_MMS_1;        //UEV -> TRG0
    // TIM1->CR2 = 0x00;
    // TIM1->SMCR |= TIM_SMCR_MSM | TIM_SMCR_SMS_2 |
    //     TIM_SMCR_SMS_1 | TIM_SMCR_TS_1;    //link timer3; resync <- TRG2
    // TIM1->SMCR |= TIM_SMCR_MSM | TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1;    //link timer3; resync <- TRG0
    // TIM1->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1;    //link timer3 <- TRG0    
    TIM1->SMCR = 0x0000;

#ifdef USE_LED_AS_TIM1_CH3
    TIM1->CCMR1 = 0x0060;    //CH1 output PWM mode 1 (channel active TIM1->CNT < TIM1->CCR1)
    TIM1->CCMR2 = 0x0060;    //CH3 output PWM mode 1 (channel active TIM1->CNT < TIM1->CCR1)
    TIM1->CCER |= TIM_CCER_CC1E  | TIM_CCER_CC1P | TIM_CCER_CC3E;    //CH1 inverted polarity
#else
    TIM1->CCMR1 = 0x0060;    //CH1 output PWM mode 1 (channel active TIM1->CNT < TIM1->CCR1)
    TIM1->CCMR2 = 0x0000;    //
    TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P;    //CH1 inverted polarity
#endif

    TIM1->BDTR |= TIM_BDTR_MOE;
    TIM1->ARR = DUTY_100_PERCENT;   

    TIM1->CNT = 0;
    TIM1->PSC = 0;

#ifdef USE_LED_AS_TIM1_CH3    
    temp = GPIOA->AFR[1];
    temp &= 0xFFFFF0F0;
    temp |= 0x00000202;    //PA8 -> AF2; PA10 -> AF2
    GPIOA->AFR[1] = temp;
#else
    temp = GPIOA->AFR[1];
    temp &= 0xFFFFFFF0;
    temp |= 0x00000002;    //PA8 -> AF2
    GPIOA->AFR[1] = temp;    
#endif

#ifdef USE_PWM_WITH_DITHER
    p_dither_tim1_ch1 = &v_dither_tim1_ch1[0];
    p_dither_tim3_ch1 = &v_dither_tim3_ch1[0];

    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 8);        
#endif
    
    TIM1->CR1 |= TIM_CR1_CEN;
}


#ifdef USE_PWM_WITH_DITHER
//                                              0     1     2     3     4     5     6     7
unsigned char v_sequence[SIZEOF_DITHER_VECT] = {0x00, 0x80, 0x88, 0xA8, 0xAA, 0xBA, 0xBB, 0xFB };

void TIM_LoadDitherSequences (unsigned short new_duty)
{
    unsigned short * p1;
    unsigned short * p3;

    p1 = (unsigned short *) v_dither_tim1_ch1;
    p3 = (unsigned short *) v_dither_tim3_ch1;

    unsigned char seq_index = (unsigned char) (new_duty & 0x0007);
    unsigned char seq = v_sequence[seq_index];    

    unsigned short adj_duty = new_duty >> 3;
    unsigned short adj_duty_plus_one = adj_duty + 1;

    unsigned short adj_duty_inv = DUTY_100_PERCENT - 2 - adj_duty;
    unsigned short adj_duty_plus_one_inv = DUTY_100_PERCENT - 2 - adj_duty_plus_one;
    
    for (unsigned char i = 0; i < SIZEOF_DITHER_VECT; i++)
    {
        if (seq & 0x01)
        {
            *(p1 + i) = adj_duty_plus_one;
            *(p3 + i) = adj_duty_plus_one_inv;
        }
        else
        {
            *(p1 + i) = adj_duty;
            *(p3 + i) = adj_duty_inv;
        }

        seq >>= 1;
    }
}


void TIM1_BRK_UP_TRG_COM_IRQHandler (void)	
{
    //actualizo los valores de los canales mosfet con sequencias pre-calculadas
    TIM1->CCR1 = *p_dither_tim1_ch1;
    TIM3->CCR1 = *p_dither_tim3_ch1;

    //ajusto los punteros para la proxima vuelta
    if (p_dither_tim1_ch1 < &v_dither_tim1_ch1[SIZEOF_DITHER_VECT - 1])
    {
        p_dither_tim1_ch1++;
        p_dither_tim3_ch1++;
    }
    else
    {
        p_dither_tim1_ch1 = &v_dither_tim1_ch1[0];
        p_dither_tim3_ch1 = &v_dither_tim3_ch1[0];
    }

#ifdef USE_LED_FOR_TIM1_INT
    if (LED)
        LED_OFF;
    else
        LED_ON;
#endif

    //bajar flag
    if (TIM1->SR & TIM_SR_UIF)
        TIM1->SR = 0x00;
}
#endif


void TIM_3_Init (void)
{
    unsigned int temp = 0;

    if (!RCC_TIM3_CLK)
        RCC_TIM3_CLK_ON;

    //Configuracion del timer.
    TIM3->CR1 |= TIM_CR1_OPM;        //clk int / 1; upcounting; one pulse mode
    // TIM3->CR2 |= TIM_CR2_MMS_1;        //UEV -> TRG0 (for the ADC sync)
    TIM3->CR2 = 0x0000;
    //TIM3->SMCR |= TIM_SMCR_SMS_2 |TIM_SMCR_SMS_1 | TIM_SMCR_TS_1 | TIM_SMCR_TS_0;    //reset mode
    //TIM3->SMCR |= TIM_SMCR_SMS_2;    //reset mode link timer1    OJO no anda
    TIM3->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1;    //trigger mode link timer1
    // TIM3->SMCR = 0x0000;    //

    // TIM3->CCMR1 = 0x0060;    //CH1 output PWM mode 1 (channel active TIM3->CNT < TIM3->CCR1)
    TIM3->CCMR1 = 0x0070;    //CH1 output PWM mode 2 (channel inactive TIM3->CNT < TIM3->CCR1)
    TIM3->CCMR2 = 0x0000;
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1P;    //CH1 inverted polarity

    TIM3->ARR = DUTY_100_PERCENT - 2;    //hago que termine siempre antes de tim1
    TIM3->CNT = 0;
    TIM3->PSC = 0;
    TIM3->CCR1 = 0;
    
    //Alternative Function Pins
    temp = GPIOA->AFR[0];
    temp &= 0xF0FFFFFF;
    temp |= 0x01000000;    //PA6 -> AF1;
    GPIOA->AFR[0] = temp;

    // Enable timer ver UDIS
    //TIM3->DIER |= TIM_DIER_UIE;
    TIM3->CR1 |= TIM_CR1_CEN;
}


void TIM3_IRQHandler (void)	
{
    //bajar flag
    if (TIM3->SR & 0x01)	//bajo el flag
        TIM3->SR = 0x00;
}


void TIM_6_Init (void)
{
    if (!RCC_TIM6_CLK)
        RCC_TIM6_CLK_ON;

    //Configuracion del timer.
    TIM6->CR1 = 0x00;		//clk int / 1; upcounting
    TIM6->PSC = 47;			//tick cada 1us
    TIM6->ARR = 0xFFFF;			//para que arranque
    //TIM6->CR1 |= TIM_CR1_CEN;
}

void TIM6Enable (void)
{
    TIM6->CR1 |= TIM_CR1_CEN;
}

void TIM6Disable (void)
{
    TIM6->CR1 &= ~TIM_CR1_CEN;
}

void TIM14_IRQHandler (void)	//100uS
{

    if (TIM14->SR & 0x01)
        //bajar flag
        TIM14->SR = 0x00;
}

void TIM_14_Init (void)
{
    unsigned int temp;

    if (!RCC_TIM14_CLK)
        RCC_TIM14_CLK_ON;

    TIM14->CCMR1 = 0x0060;            //CH1 output PWM mode 1
    TIM14->CCER |= TIM_CCER_CC1E;    //CH1 enable on pin active high
    //TIM3->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2P;    //CH2 enable on pin active high
    TIM14->PSC = 3;			//tick cada 83.33n
    TIM14->ARR = 1023;    //freq 11.73KHz

    //Configuracion del timer.
    TIM14->EGR |= 0x0001;

    //Configuracion Pin PB1
    temp = GPIOB->AFR[0];
    temp &= 0xFFFFFF0F;
    temp |= 0x00000000;	//PB1 -> AF0
    GPIOB->AFR[0] = temp;

    TIM14->CR1 |= TIM_CR1_CEN;

}

void TIM16_IRQHandler (void)	//es one shoot
{
    //SendDMXPacket(PCKT_UPDATE);

    if (TIM16->SR & 0x01)
        //bajar flag
        TIM16->SR = 0x00;
}

void TIM_16_Init (void)
{
    if (!RCC_TIM16_CLK)
        RCC_TIM16_CLK_ON;

    //Configuracion del timer.
    TIM16->CR1 = 0x00;		//clk int / 1; upcounting; uev
    TIM16->ARR = 0xFFFF;
    TIM16->CNT = 0;
    TIM16->PSC = 47;			//tick 1us
    TIM16->EGR = TIM_EGR_UG;

}


void OneShootTIM16 (unsigned short a)
{
    TIM16->ARR = a;
    TIM16->CR1 |= TIM_CR1_CEN;
}


void TIM16Enable (void)
{
    TIM16->CR1 |= TIM_CR1_CEN;
}


void TIM16Disable (void)
{
    TIM16->CR1 &= ~TIM_CR1_CEN;
}


void TIM_17_Init (void)
{
    if (!RCC_TIM17_CLK)
        RCC_TIM17_CLK_ON;

    //Configuracion del timer.
    TIM17->ARR = 0xFFFF;
    TIM17->CNT = 0;
    TIM17->PSC = 47;

    // Enable timer interrupt ver UDIS
    TIM17->DIER |= TIM_DIER_UIE;
    TIM17->CR1 |= TIM_CR1_URS;	//solo int cuando hay overflow y one shot

    NVIC_EnableIRQ(TIM17_IRQn);
    NVIC_SetPriority(TIM17_IRQn, 8);
}


void TIM17_IRQHandler (void)
{
    if (TIM17->SR & 0x01)
    {
        // SYNC_Zero_Crossing_Handler();
        TIM17->SR = 0x00;    //flag down
    }    
}


void TIM17Enable (void)
{
    TIM17->CR1 |= TIM_CR1_CEN;
}


void TIM17Disable (void)
{
    TIM17->CR1 &= ~TIM_CR1_CEN;
}


//--- end of file ---//
