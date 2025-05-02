/**
  ******************************************************************************
  * @file    mpu6050.h
  * @author  NTUT Fryan Liu
  * @brief   Source file of PID controller for quadcoptor
  ******************************************************************************
  */

#include "quadcoptor_pid.h"

PID_TypeDefStruct PID_handler_roll;
PID_TypeDefStruct PID_handler_pitch;
PID_TypeDefStruct PID_handler_yaw;

Motor_TypeDefStruct motor_handler;

void Motor_calibrate(void)
{
    MOTOR_FRONT_LEFT_REG = MOTOR_MAX_OUTPUT;
    MOTOR_FRONT_RIGHT_REG = MOTOR_MAX_OUTPUT;
    MOTOR_REAR_LEFT_REG = MOTOR_MAX_OUTPUT;
    MOTOR_REAR_RIGHT_REG = MOTOR_MAX_OUTPUT;
    delay_ms(10);
    MOTOR_FRONT_LEFT_REG = MOTOR_MIN_OUTPUT;
    MOTOR_FRONT_RIGHT_REG = MOTOR_MIN_OUTPUT;
    MOTOR_REAR_LEFT_REG = MOTOR_MIN_OUTPUT;
    MOTOR_REAR_RIGHT_REG = MOTOR_MIN_OUTPUT;
}

void Motor_init(Motor_TypeDefStruct *motor_handler)
{
    motor_handler->throttle = 0;
    motor_handler->roll_output = 0;
    motor_handler->pitch_output = 0;
    motor_handler->yaw_output = 0;
    motor_handler->motor[0] = 0;
    motor_handler->motor[1] = 0;
    motor_handler->motor[2] = 0;
    motor_handler->motor[3] = 0;

    MOTOR_FRONT_LEFT_REG = 0;
    MOTOR_FRONT_RIGHT_REG = 0;
    MOTOR_REAR_LEFT_REG = 0;
    MOTOR_REAR_RIGHT_REG = 0;
}

void Motor_control(Motor_TypeDefStruct *motor_handler, PID_TypeDefStruct *PID_handler_roll, PID_TypeDefStruct *PID_handler_pitch, PID_TypeDefStruct *PID_handler_yaw)
{
    int i;

    motor_handler->roll_output = PID_handler_roll->pid_output;
    motor_handler->pitch_output = PID_handler_pitch->pid_output;
    motor_handler->yaw_output = PID_handler_yaw->pid_output;

    motor_handler->motor[0] = motor_handler->throttle - motor_handler->roll_output - motor_handler->pitch_output - motor_handler->yaw_output;
    motor_handler->motor[1] = motor_handler->throttle - motor_handler->roll_output + motor_handler->pitch_output + motor_handler->yaw_output;
    motor_handler->motor[2] = motor_handler->throttle + motor_handler->roll_output + motor_handler->pitch_output - motor_handler->yaw_output;
    motor_handler->motor[3] = motor_handler->throttle + motor_handler->roll_output - motor_handler->pitch_output + motor_handler->yaw_output;

    for(i = 0; i < 4; i++)
    {
        if(motor_handler->motor[i] > MOTOR_MAX_OUTPUT)
        {
            motor_handler->motor[i] = MOTOR_MAX_OUTPUT;
        }
        else if(motor_handler->motor[i] < MOTOR_MIN_OUTPUT)
        {
            motor_handler->motor[i] = MOTOR_MIN_OUTPUT;
        }
    }
    
    MOTOR_FRONT_RIGHT_REG = motor_handler->motor[0];
    MOTOR_REAR_RIGHT_REG = motor_handler->motor[1];
    MOTOR_REAR_LEFT_REG = motor_handler->motor[2];
    MOTOR_FRONT_LEFT_REG = motor_handler->motor[3];
}

void Motor_setPWM(short int pwm_pulse)
{
    MOTOR_FRONT_LEFT_REG = pwm_pulse;
    MOTOR_FRONT_RIGHT_REG = pwm_pulse;
    MOTOR_REAR_LEFT_REG = pwm_pulse;
    MOTOR_REAR_RIGHT_REG = pwm_pulse;
}

void Motor_setThrottle(Motor_TypeDefStruct *motor_handler, float throttle)
{
    if(throttle > MAX_THROTTLE)
    {
        throttle = MAX_THROTTLE;
    }
    else
    {
        motor_handler->throttle = throttle;
    }
}

void PID_init(PID_TypeDefStruct *PID_handler, float inner_kp, float inner_ki, float inner_kd, float outer_kp)
{
    PID_handler->inner_kp = inner_kp;
    PID_handler->inner_ki = inner_ki;
    PID_handler->inner_kd = inner_kd;
    PID_handler->error_angle_rate_list[0] = 0;
    PID_handler->error_angle_rate_list[1] = 0;
    PID_handler->error_angle_rate_list[2] = 0;
    PID_handler->pid_prev_output = 0;
    PID_handler->pid_output = 0;
    PID_handler->target_angle_rate = 0;
    PID_handler->target_angle = 0;
    PID_handler->outer_kp = outer_kp;
}

void PID_angleRate(PID_TypeDefStruct *PID_handler, IMU_Data_Frame *imu_data_frame, int angle_type)
{
    if(PID_handler->pid_prev_output > MAX_PID_OUTPUT || PID_handler->pid_prev_output < MIN_PID_OUTPUT)
    {
        PID_ResetOutputAndError(PID_handler);
    }

    PID_handler->error_angle_rate_list[2] = PID_handler->error_angle_rate_list[1];
    PID_handler->error_angle_rate_list[1] = PID_handler->error_angle_rate_list[0];
    PID_handler->error_angle_rate_list[0] = PID_handler->target_angle_rate - imu_data_frame->gyro_data[angle_type];

    float delta_p = PID_handler->inner_kp * (PID_handler->error_angle_rate_list[0] - PID_handler->error_angle_rate_list[1]);
    float delta_i = PID_handler->inner_ki * PID_handler->error_angle_rate_list[0];
    float delta_d = PID_handler->inner_kd * (PID_handler->error_angle_rate_list[0] - 2 * PID_handler->error_angle_rate_list[1] + PID_handler->error_angle_rate_list[2]);

    PID_handler->pid_output = delta_p + delta_i + delta_d + PID_handler->pid_prev_output;
    PID_handler->pid_prev_output = PID_handler->pid_output;

    if(PID_handler->pid_output > MAX_PID_OUTPUT)
    {
        PID_handler->pid_output = MAX_PID_OUTPUT;
    }
    else if(PID_handler->pid_output < MIN_PID_OUTPUT)
    {
        PID_handler->pid_output = MIN_PID_OUTPUT;
    }

    //printf("pid output %f\n", PID_handler->pid_output);
}

void PID_angle(PID_TypeDefStruct *PID_handler, float received_value, IMU_Data_Frame *imu_data_frame, int angle_type)
{
    float current_angle = 0;
    received_value = received_value - MID_RECEIVER_VALUE;
    if(angle_type == ROLL)
    {
        if(received_value >= 0)
        {
            PID_handler->target_angle = (MAX_ANGLE_ROLL / 500.0) * received_value;
        }
        else
        {
            received_value = -received_value;  
            PID_handler->target_angle = (MIN_ANGLE_ROLL / 500.0) * received_value;
        }
        current_angle = imu_data_frame->angle[0];
    }
    else if(angle_type == PITCH)
    {
        if(received_value >= 0)
        {
            PID_handler->target_angle = (MAX_ANGLE_PITCH / 500.0) * received_value;
        }
        else
        {
            received_value = -received_value;
            PID_handler->target_angle = (MIN_ANGLE_PITCH / 500.0) * received_value;
        }
        current_angle = imu_data_frame->angle[1];
    }
    else if(angle_type == YAW)
    {
        current_angle = imu_data_frame->angle[2];
    } 
    else
    {
        PID_handler->target_angle = 0;
        return;
    }

    PID_handler->target_angle_rate = PID_handler->outer_kp * (PID_handler->target_angle - current_angle);
    //printf("target angle rate %f\n", PID_handler->target_angle_rate);
    PID_angleRate(PID_handler, imu_data_frame, angle_type);
}

void PID_ResetOutputAndError(PID_TypeDefStruct *PID_handler)
{
    PID_handler->error_angle_rate_list[0] = 0;
    PID_handler->error_angle_rate_list[1] = 0;
    PID_handler->error_angle_rate_list[2] = 0;
    PID_handler->pid_prev_output = 0;
    PID_handler->pid_output = 0;
}