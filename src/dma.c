//---------------------------------------------
// ##
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### DMA.C #################################
//---------------------------------------------

#include "dma.h"
#include "stm32f0xx.h"

#include "adc.h"

/* Externals variables ---------------------------------------------------------*/
extern volatile unsigned short adc_ch [];

/* Global variables ---------------------------------------------------------*/


/* Module Definitions ---------------------------------------------------------*/


/* Module functions ---------------------------------------------------------*/
void DMAConfig(void)
{
    /* DMA1 clock enable */
    if (!RCC_DMA_CLK)
        RCC_DMA_CLK_ON;

    //////////////////////////////////
    // DMA1 Channel 1 Configuration //
    //////////////////////////////////
    //Configuro el control del DMA CH1 y lo uso para el ADC
    DMA1_Channel1->CCR = 0;
    //priority very high
    //memory halfword
    //peripheral halfword
    //increment memory
    DMA1_Channel1->CCR |= DMA_CCR_PL | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC;
    //DMA1_Channel1->CCR |= DMA_Mode_Circular | DMA_CCR_TCIE;
    //cicular mode
    DMA1_Channel1->CCR |= DMA_CCR_CIRC;

    //Tamaño del buffer a transmitir
    DMA1_Channel1->CNDTR = ADC_CHANNEL_QUANTITY;

    //Address del periferico
    DMA1_Channel1->CPAR = (uint32_t) &ADC1->DR;

    //Address en memoria
    DMA1_Channel1->CMAR = (uint32_t) &adc_ch[0];

    //Enable
    //DMA1_Channel1->CCR |= DMA_CCR_EN;

    //////////////////////////////////
    // DMA1 Channel 2 Configuration //
    //////////////////////////////////
    // //Configuro el control del DMA CH2 para el PWM dither en channel
    // DMA1_Channel2->CCR = 0;
    // //priority very high
    // //memory halfword
    // //peripheral halfword
    // //increment memory
    // DMA1_Channel2->CCR |= DMA_CCR_PL | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC;
    // //DMA1_Channel2->CCR |= DMA_Mode_Circular | DMA_CCR_TCIE;
    // //cicular mode
    // DMA1_Channel2->CCR |= DMA_CCR_CIRC;

    // //Tamaño del buffer a transmitir
    // DMA1_Channel2->CNDTR = ADC_CHANNEL_QUANTITY;

    // //Address del periferico
    // DMA1_Channel2->CPAR = (uint32_t) &ADC1->DR;

    // //Address en memoria
    // DMA1_Channel2->CMAR = (uint32_t) &adc_ch[0];

    // //Enable
    // //DMA1_Channel2->CCR |= DMA_CCR_EN;

#ifdef DMA_CH1_WITH_INTERRUPT
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_SetPriority(DMA1_Channel1_IRQn, 5);
#endif
}

#ifdef DMA_CH1_WITH_INTERRUPT
void DMA_Channel1_EnableInterrupt (void)
{
    DMA1_Channel1->CCR |= DMA_CCR_TCIE;
}

void DMA_Channel1_DisableInterrupt (void)
{
    DMA1_Channel1->CCR &= ~DMA_CCR_TCIE;
}

void DMA1_Channel1_IRQHandler (void)
{
    if (sequence_ready)
    {
        // Clear DMA TC flag
        sequence_ready_reset;
        
    }
}
#endif

//---- end of file ----//
