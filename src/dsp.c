//---------------------------------------------
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### DSP.C #################################
//---------------------------------------------

#include "dsp.h"


#include <stdlib.h>
#include <math.h>


/* Externals variables ---------------------------------------------------------*/
#ifdef USE_PID_UPDATED_CONSTANTS
unsigned short pid_param_p;
unsigned short pid_param_i;
unsigned short pid_param_d;
#endif

/* Global variables ---------------------------------------------------------*/
//------- de los PID ---------
#ifdef USE_PID_CONTROLLERS
int acc = 0;
short error_z1 = 0;
short error_z2 = 0;
short d_last = 0;
#endif

#ifdef USE_MA8_CIRCULAR
//vout filter
unsigned short v_ma8_vout [8];
unsigned short * p_ma8_vout;
unsigned int total_ma8_vout = 0;
//vline filter
unsigned short v_ma8_vline [8];
unsigned short * p_ma8_vline;
unsigned int total_ma8_vline = 0;
//general
unsigned short v_ma8 [8];
unsigned short * p_ma8;
unsigned int total_ma8 = 0;
#endif

/* Module Definitions ---------------------------------------------------------*/
// #define PID_CONSTANT_DIVIDER    10    //todos se dividen por 1024
// #define PID_CONSTANT_DIVIDER    8    //todos se dividen por 256
#define PID_CONSTANT_DIVIDER    7    //todos se dividen por 128
// #define PID_CONSTANT_DIVIDER    6    //todos se dividen por 64

//from microinverter01.py
// #define KPV	14    //0.108
// #define KIV	11    //0.08333
// #define KDV	0

//estos funcionan bastante bien para tension en vacio, prende apaga alrededor de 100V
//usan divisor por 128, ajusta en 35.6V, este con carga ajusta mejor 34.3 a 35.6
#define KPV    5    //kp_dig = 0.039
// #define KIV    3    //ki_dig = 0.023, ajusta pero tiene bastante error 42pts
#define KIV    128    //ki_dig = 0.023, ajusta pero tiene bastante error 42pts
#define KDV    0

//estos funcionan bastante bien para tension en vacio, prende apaga alrededor de 80V
//usan divisor por 128, ajustan en 34.3V, ajusta muy rapido diferentes cambios de tension
//con carga no ajusta tan bien 32.3 a 34.6
// #define KPV	19    //kp_dig = 0.15
// #define KIV	1    //ki_dig = 0.0078
// #define KDV	182    //kd_dig = 1.42

// #define KPV	2    //kp_dig = 0.01
// #define KIV	1    //ki_dig = 0.0000416
// #define KDV	24    //kd_dig = 0.024


#define K1V (KPV + KIV + KDV)
#define K2V (KPV + KDV + KDV)
#define K3V (KDV)

/* Module functions ---------------------------------------------------------*/

unsigned short RandomGen (unsigned int seed)
{
	unsigned int random;

	//Random Generator
	srand (seed);
	random = rand();

	return (unsigned short) random;

}

#ifdef USE_MA16_U16_CIRCULAR
//set de punteros y vaciado del filtro
//recibe:
// puntero a estructura de datos del filtro "ma16_u16_data_obj_t *"
void MA16_U16Circular_Reset (ma16_u16_data_obj_t * p_data)
{
    unsigned char i;
    
    for (i = 0; i < 16; i++)
        p_data->v_ma[i] = 0;

    p_data->p_ma = p_data->v_ma;
    p_data->total_ma = 0;
}

//Filtro circular, necesito activar previamente con MA16_U16Circular_Reset()
//recibe:
// puntero a estructura de datos del filtro "ma16_u16_data_obj_t *"
// nueva mustra "new_sample"
//contesta:
// resultado del filtro
unsigned short MA16_U16Circular (ma16_u16_data_obj_t *p_data, unsigned short new_sample)
{
    p_data->total_ma -= *(p_data->p_ma);
    p_data->total_ma += new_sample;
    *(p_data->p_ma) = new_sample;

    if (p_data->p_ma < ((p_data->v_ma) + 15))
        p_data->p_ma += 1;
    else
        p_data->p_ma = p_data->v_ma;

    return (unsigned short) (p_data->total_ma >> 4);    
}

unsigned short MA16_U16Circular_Only_Calc (ma16_u16_data_obj_t *p_data)
{
    return (unsigned short) (p_data->total_ma >> 4);
}

#endif    //USE_MA16_U16_CIRCULAR


#ifdef USE_MA32_U8_CIRCULAR
//set de punteros y vaciado del filtro
//recibe:
// puntero a estructura de datos del filtro "ma32_u8_data_obj_t *"
void MA32_U8Circular_Reset (ma32_u8_data_obj_t * p_data)
{
    unsigned char i;
    
    for (i = 0; i < 32; i++)
        p_data->v_ma[i] = 0;

    p_data->p_ma = p_data->v_ma;
    p_data->total_ma = 0;
}

//Filtro circular, necesito activar previamente con MA32_U8Circular_Reset()
//recibe:
// puntero a estructura de datos del filtro "ma32_u8_data_obj_t *"
// nueva mustra "new_sample"
//contesta:
// resultado del filtro
unsigned char MA32_U8Circular (ma32_u8_data_obj_t *p_data, unsigned char new_sample)
{
    p_data->total_ma -= *(p_data->p_ma);
    p_data->total_ma += new_sample;
    *(p_data->p_ma) = new_sample;

    if (p_data->p_ma < ((p_data->v_ma) + 31))
        p_data->p_ma += 1;
    else
        p_data->p_ma = p_data->v_ma;

    return (unsigned char) (p_data->total_ma >> 5);    
}

unsigned char MA32_U8Circular_Only_Calc (ma32_u8_data_obj_t *p_data)
{
    return (unsigned char) (p_data->total_ma >> 5);
}

#endif    //USE_MA32_U8_CIRCULAR

#ifdef USE_PID_CONTROLLERS
short PID (pid_data_obj_t * p)
{
    short error = 0;
    short d = 0;

    unsigned short k1 = 0;
    unsigned short k2 = 0;
    unsigned short k3 = 0;
    
    short val_k1 = 0;
    short val_k2 = 0;
    short val_k3 = 0;

    k1 = p->kp + p->ki + p->kd;
    k2 = p->kp + p->kd + p->kd;
    k3 = p->kd;
    
    error = p->setpoint - p->sample;

    //K1
    acc = k1 * error;
    val_k1 = acc >> PID_CONSTANT_DIVIDER;

    //K2
    acc = k2 * p->error_z1;
    val_k2 = acc >> PID_CONSTANT_DIVIDER;

    //K3
    acc = k3 * p->error_z2;
    val_k3 = acc >> PID_CONSTANT_DIVIDER;

    d = p->last_d + val_k1 - val_k2 + val_k3;

    //Update PID variables
    p->error_z2 = p->error_z1;
    p->error_z1 = error;
    p->last_d = d;

    return d;
}

void PID_Flush_Errors (pid_data_obj_t * p)
{
    p->last_d = 0;
    p->error_z1 = 0;
    p->error_z2 = 0;
}

#if (defined USE_PID_UPDATED_CONSTANTS)
#define K1    (pid_param_p + pid_param_i + pid_param_d)
#define K2    (pid_param_p + pid_param_d + pid_param_d)
#define K3    (pid_param_d)

void PID_update_constants (unsigned short kp, unsigned short ki, unsigned short kd)
{
    pid_param_p = kp;
    pid_param_i = ki;
    pid_param_d = kd;
}
#elif (defined USE_PID_FIXED_CONSTANTS)
#define K1    K1V
#define K2    K2V
#define K3    K3V
#else
#error "Select the PID constants mode on dsp.h"
#endif

short PID_roof (short setpoint, short sample, short local_last_d, short * e_z1, short * e_z2)
{
    short error = 0;
    short d = 0;

    short val_k1 = 0;
    short val_k2 = 0;
    short val_k3 = 0;

    error = setpoint - sample;

    //K1
    acc = K1 * error;
    val_k1 = acc >> PID_CONSTANT_DIVIDER;

    //K2
    acc = K2 * *e_z1;
    val_k2 = acc >> PID_CONSTANT_DIVIDER;    //si es mas grande que K1 + K3 no lo deja arrancar

    //K3
    acc = K3 * *e_z2;
    val_k3 = acc >> PID_CONSTANT_DIVIDER;

    d = local_last_d + val_k1 - val_k2 + val_k3;

    //Update variables PID
    *e_z2 = *e_z1;
    *e_z1 = error;

    return d;
}

#endif    //USE_PID_CONTROLLERS

//--- end of file ---//
