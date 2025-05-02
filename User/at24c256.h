/**
******************************************************************************
* @file    at24c256.h
* @author  NTUT Chung Po An
* @brief   Header file of at24c256 driver  
******************************************************************************
*/

#ifndef AT24C256_H
#define AT24C256_H
  
/* This is for STM32 HAL library */
#include "stm32f1xx_hal.h"
/* ----------------------------- */
#include "bsp_i2c.h"
#include "bsp_delay.h"
#include <math.h>

/* AT24C256 EEPROM device address */
#define AT24C256_BASE  0x50
#define AT24C256_WRITE ((AT24C256_BASE << 1) | 0)
#define AT24C256_READ  ((AT24C256_BASE << 1) | 1)
//---------------------------------------------------------------------
I2C_StatusTypeDef AT24C256_WriteByte(uint8_t i2c_controller_num, uint8_t dev_addr, uint16_t reg_addr, uint8_t *data);
I2C_StatusTypeDef AT24C256_WriteBytes(uint8_t i2c_controller_num, uint8_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t size);
//---------------------------------------------------------------------
I2C_StatusTypeDef AT24C256_ReadByte(uint8_t i2c_controller_num, uint8_t dev_addr, uint16_t reg_addr, uint8_t *data);
I2C_StatusTypeDef AT24C256_ReadBytes(uint8_t i2c_controller_num, uint8_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t size);
//---------------------------------------------------------------------

#endif