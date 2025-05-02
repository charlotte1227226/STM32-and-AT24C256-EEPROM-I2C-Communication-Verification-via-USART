/**
  ******************************************************************************
  * @file    bsp_i2c.h
  * @author  NTUT MCA Lab Fryan Liu
  * @brief   Header file of custom I2C driver (HAL-based)
  ******************************************************************************
  */

#ifndef __BSP_I2C_H__
#define __BSP_I2C_H__
  
#ifdef __cplusplus
extern "C" {
#endif
  
/* STM32 HAL headers */
#include "stm32f1xx_hal.h"
#include "i2c.h"
  
/* Return type for BSP I2C operations */
typedef enum
{
    I2C_OK = 0,
    I2C_ERROR,
    I2C_TIMEOUT
} I2C_StatusTypeDef;  
/* I2C Controller IDs */
#define I2C1_ID 1
#define I2C2_ID 2
#define I2C3_ID 3

/* Timeout setting */
#define I2C_TIMEOUT_MS 1000
  
/* Function Prototypes */
I2C_StatusTypeDef I2C_Init(uint8_t i2c_controller_num);
I2C_StatusTypeDef I2C_Write(uint8_t i2c_controller_num, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t size);
I2C_StatusTypeDef I2C_Read(uint8_t i2c_controller_num, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t size);
#ifdef __cplusplus
}
#endif
  
#endif /* __BSP_I2C_H__ */
  