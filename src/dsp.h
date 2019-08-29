//---------------------------------------------
// ## @Author: Med
// ## @Editor: Emacs - ggtags
// ## @TAGS:   Global
// ##
// #### DSP.H #################################
//---------------------------------------------

#ifndef _DSP_H_
#define _DSP_H_

//--- Defines for configuration -----------------
#define USE_PID_CONTROLLERS
#define USE_PID_FIXED_CONSTANTS
// #define USE_PID_UPDATED_CONSTANTS

#define USE_MA8_CIRCULAR
#define USE_MA32_CIRCULAR


//--- Exported constants ------------------------

//--- Exported types ----------------------------
typedef struct {
    unsigned short v_ma8[8];
    unsigned short * p_ma8;
    unsigned int total_ma8;
} ma8_data_obj_t;

typedef struct {
    short setpoint;
    short sample;
    short last_d;
    short error_z1;
    short error_z2;
    unsigned short kp;
    unsigned short ki;
    unsigned short kd;
} pid_data_obj_t;

//--- Module Functions --------------------------
unsigned short RandomGen (unsigned int);
unsigned char MAFilter (unsigned char, unsigned char *);
unsigned short MAFilterFast (unsigned short ,unsigned short *);
unsigned short MAFilter8 (unsigned short *);
unsigned short MAFilter32 (unsigned short, unsigned short *);

unsigned short MAFilter32Fast (unsigned short *);
unsigned short MAFilter32Circular (unsigned short, unsigned short *, unsigned char *, unsigned int *);

#ifdef USE_PID_CONTROLLERS
short PID (pid_data_obj_t *);
void PID_Flush_Errors (pid_data_obj_t *);
short PID_roof (short, short, short, short *, short *);
void PID_update_constants (unsigned short, unsigned short, unsigned short);
#endif

#ifdef USE_MA8_CIRCULAR
void MA8Circular_Reset (ma8_data_obj_t *);
unsigned short MA8Circular (ma8_data_obj_t *, unsigned short);
unsigned short MA8Circular_Only_Calc (ma8_data_obj_t *);
#endif

#ifdef USE_MA32_CIRCULAR
void MA32Circular_Start (void);
void MA32Circular_Reset (void);
unsigned short MA32Circular_Calc (void);
void MA32Circular_Load (unsigned short);
#endif

#endif /* _DSP_H_ */
