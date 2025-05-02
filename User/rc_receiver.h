/**
  ******************************************************************************
  * @file    mpu6050.h
  * @author  NTUT Fryan Liu
  * @brief   Header file of rc_receiver handler  
  ******************************************************************************
  */

#ifndef rc_receiver_h
#define rc_receiver_h

/* This is for STM32 HAL library */
#include "stm32f1xx_hal.h"
#include "main.h"
/* ----------------------------- */

typedef enum {
    THROTTLE,
    ROLL,
    PITCH,
    YAW
} Angle_TypeDefEnum;

typedef struct {
    short int throttle_rc_value;
    short int roll_rc_value;
    short int pitch_rc_value;
    short int yaw_rc_value;
} RC_Receiver_TypeDefStruct;

extern RC_Receiver_TypeDefStruct rc_receiver_handler;

void RC_Receiver_Init(void);
void RC_Receiver_SetValue(RC_Receiver_TypeDefStruct *rc_receiver_handler, short int value, Angle_TypeDefEnum angle_type);
short int RC_Receiver_GetValue(RC_Receiver_TypeDefStruct *rc_receiver_handler, Angle_TypeDefEnum angle_type);

#endif