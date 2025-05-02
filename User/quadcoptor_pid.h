/**
  ******************************************************************************
  * @file    mpu6050.h
  * @author  NTUT Fryan Liu
  * @brief   Header file of PID controller for quadcoptor
  ******************************************************************************
  */

#ifndef quadcoptor_pid_h
#define quadcoptor_pid_h

/* This is for STM32 HAL library */
#include "stm32f1xx_hal.h"
/* ----------------------------- */

#include "mpu6050.h"
#include <math.h>

#define MAX_ANGLE_ROLL (float)45
#define MAX_ANGLE_PITCH (float)45

#define MIN_ANGLE_ROLL (float)-45
#define MIN_ANGLE_PITCH (float)-45

#define MAX_PID_OUTPUT 200
#define MIN_PID_OUTPUT -200

#define MID_RECEIVER_VALUE 1500

#define MAX_THROTTLE 1900

#define MOTOR_MAX_OUTPUT 2000 //us
#define MOTOR_MIN_OUTPUT 1100 //us
#define MOTOR_ARMING 1000 //us
#define MOTOR_FRONT_LEFT_REG (TIM2->CCR1) // motor 4 -> motor[3] CW
#define MOTOR_FRONT_RIGHT_REG (TIM4->CCR2) //motor 1 -> motor[0] CCW
#define MOTOR_REAR_LEFT_REG (TIM2->CCR2) // motor 3 -> motor[2]  CCW
#define MOTOR_REAR_RIGHT_REG (TIM4->CCR1) // motor 2 -> motor[1] CW

typedef enum {
    ROLL,
    PITCH,
    YAW
} Angle_TypeDefEnum;

typedef struct {
    float outer_kp;
    float inner_kp;
    float inner_ki;
    float inner_kd;
    float error_angle_rate_list[3];  // error_list[0] => t error, error_list[1] => t-1 error, error_list[2] => t-2 error
    float target_angle_rate;
    float target_angle;
    short int pid_prev_output;
    short int pid_output;
} PID_TypeDefStruct;

typedef struct {
    short int throttle;
    short int roll_output;
    short int pitch_output;
    short int yaw_output;
    short int motor[4];  // motor[0] => front right, motor[1] => rear right, motor[2] => rear left, motor[3] => front left
} Motor_TypeDefStruct;

extern PID_TypeDefStruct PID_handler_roll;
extern PID_TypeDefStruct PID_handler_pitch;
extern PID_TypeDefStruct PID_handler_yaw;

extern Motor_TypeDefStruct motor_handler;

void Motor_calibrate(void);
void Motor_init(Motor_TypeDefStruct *motor_handler);
void Motor_control(Motor_TypeDefStruct *motor_handler, PID_TypeDefStruct *PID_handler_roll, PID_TypeDefStruct *PID_handler_pitch, PID_TypeDefStruct *PID_handler_yaw);
void Motor_setPWM(short int pwm_pulse);
void Motor_setThrottle(Motor_TypeDefStruct *motor_handler, float throttle);

void PID_init(PID_TypeDefStruct *PID_handler, float inner_kp, float inner_ki, float inner_kd, float outer_kp);
void PID_angle(PID_TypeDefStruct *PID_handler, float received_value, IMU_Data_Frame *imu_data_frame, int angle_type);
void PID_angleRate(PID_TypeDefStruct *PID_handler, IMU_Data_Frame *imu_data_frame, int angle_type);
void PID_setting(PID_TypeDefStruct *PID_handler, float inner_kp, float inner_ki, float inner_kd, float outer_kp);
void PID_ResetOutputAndError(PID_TypeDefStruct *PID_handler);


#endif