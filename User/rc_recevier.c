/**
  ******************************************************************************
  * @file    mpu6050.h
  * @author  NTUT Fryan Liu
  * @brief   Source file of rc_receiver handler  
  ******************************************************************************
  */

#include "rc_receiver.h"

RC_Receiver_TypeDefStruct rc_receiver_handler;


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)
    {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
        {
            rc_receiver_handler.channel_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
            __HAL_TIM_SET_COUNTER(htim, 0);
        }
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
        {
            rc_receiver_handler.channel_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
            __HAL_TIM_SET_COUNTER(htim, 0);
        }
    }

    if(htim->Instance == TIM4)
    {
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {
            rc_receiver_handler.channel_3 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            __HAL_TIM_SET_COUNTER(htim, 0);
        }
        else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
        {
            rc_receiver_handler.channel_4 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            __HAL_TIM_SET_COUNTER(htim, 0);
        }
    }
}