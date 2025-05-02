/**
******************************************************************************
* @file    at24c256.h
* @author  NTUT Chung Po An
* @brief   at24c256 driver  
******************************************************************************
*/

#include "at24c256.h"

I2C_StatusTypeDef AT24C256_WriteByte(uint8_t i2c_controller_num, uint8_t dev_addr, uint16_t reg_addr, uint8_t *data)
{
    if (dev_addr == AT24C256_BASE) {
        printf("dev_address correct\n");
        HAL_Delay(10);
        dev_addr = AT24C256_WRITE;
    }

    uint8_t tx_data[3];
    tx_data[0] = (reg_addr >> 8) & 0xFF;
    tx_data[1] = reg_addr & 0xFF;
    tx_data[2] = *data;

    I2C_StatusTypeDef status;

    switch (i2c_controller_num)
    {
        case I2C1_ID:
            status = I2C_Write(i2c_controller_num, dev_addr, 0, tx_data, 3, 2);
            if (status != I2C_OK) return status;
            printf("Byte transmit is HAL OK!! now continue\n");
            break;

        case I2C2_ID:
            // status = I2C_Write(i2c_controller_num, dev_addr, 0, tx_data, 3);
            // if (status != I2C_OK) return status;
            // printf("Byte transmit is HAL OK!! now continue\n");
            break;

        default:
            return I2C_ERROR;
    }

    return I2C_OK;
}

I2C_StatusTypeDef AT24C256_WriteBytes(uint8_t i2c_controller_num, uint8_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t size)
{
    if (dev_addr == AT24C256_BASE){
        printf("dev_address correct\n");
        HAL_Delay(10);
        dev_addr = AT24C256_WRITE;
    }

    uint8_t tx_data[size+2];
    tx_data[0] = (reg_addr >> 8) & 0xFF;
    tx_data[1] = (reg_addr) & 0xFF;

    for (int i = 0; i < size; i++)
    {
        tx_data[i+2] = data[i];
    }
    
    I2C_StatusTypeDef status;

    switch (i2c_controller_num)
    {
        case I2C1_ID :
            status = I2C_Write(i2c_controller_num, dev_addr, 0, tx_data, size+2, 2);
            if (status != I2C_OK) return status;
            printf("Byte transmit is HAL OK!! now continue\n");
            break;
        case I2C2_ID :
            // status = I2C_Write(i2c_controller_num, dev_addr, 0, tx_data, 3);
            // if (status != I2C_OK) return status;
            // printf("Byte transmit is HAL OK!! now continue\n");
            break;
        default:
            return I2C_ERROR;
    }
  
    return I2C_OK;
}

I2C_StatusTypeDef AT24C256_ReadByte(uint8_t i2c_controller_num, uint8_t dev_addr, uint16_t reg_addr, uint8_t *data)
{
    if (dev_addr == AT24C256_BASE){
        printf("dev_address correct\n");
        HAL_Delay(10);
        dev_addr = AT24C256_WRITE;
    }

    uint8_t rx_data_addr[2];
    rx_data_addr[0] = (reg_addr >> 8) & 0xFF;
    rx_data_addr[1] = reg_addr & 0xFF;

    I2C_StatusTypeDef status;

    switch (i2c_controller_num)
    {
        case I2C1_ID :
            // status = I2C_Write(i2c_controller_num, dev_addr, 0, rx_data_addr, 2);
            // if (status != I2C_OK) return status;
            // printf("Byte transmit is HAL OK!! now continue\n");

            dev_addr = AT24C256_READ;
            HAL_Delay(5);

            status = I2C_Read(i2c_controller_num, dev_addr, rx_data_addr, data, 1, 2);
            if (status != I2C_OK) return status;
            printf("read AT24C256 memory is HAL OK!! now continue\n");
            printf("Byte receive is HAL OK!! now continue\n");

            break;
        case I2C2_ID :
            // status = I2C_Write(i2c_controller_num, dev_addr, 0, rx_data_addr, 2);
            // if (status != I2C_OK) return status;
            // printf("Byte transmit is HAL OK!! now continue\n");

            // dev_addr = AT24C256_READ;
            // HAL_Delay(5);

            // status = I2C_Read(i2c_controller_num, dev_addr, 0, data, 1);
            // if (status != I2C_OK) return status;
            // printf("read AT24C256 memory is HAL OK!! now continue\n");
            // printf("Byte receive is HAL OK!! now continue\n");
            
            break;
        default:
            return I2C_ERROR;
    }
  
    return I2C_OK;
}


I2C_StatusTypeDef AT24C256_ReadBytes(uint8_t i2c_controller_num, uint8_t dev_addr, uint16_t reg_addr, uint8_t *data, uint16_t size)
{
    if (dev_addr == AT24C256_BASE){
        printf("dev_address correct\n");
        HAL_Delay(10);
        dev_addr = AT24C256_WRITE;
    }

    uint8_t rx_data_addr[2];
    rx_data_addr[0] = (reg_addr >> 8) & 0xFF;
    rx_data_addr[1] = reg_addr & 0xFF;

    I2C_StatusTypeDef status;

    switch (i2c_controller_num)
    {
        case I2C1_ID :
            // status = I2C_Write(i2c_controller_num, dev_addr, 0, rx_data_addr, 2);
            // if (status != I2C_OK) return status;
            // printf("Byte transmit is HAL OK!! now continue\n");

            dev_addr = AT24C256_READ;
            HAL_Delay(5);

            status = I2C_Read(i2c_controller_num, dev_addr, rx_data_addr, data, size, 2);
            if (status != I2C_OK) return status;
            printf("read AT24C256 memory is HAL OK!! now continue\n");
            printf("Bytes receive is HAL OK!! now continue\n");

            break;
        case I2C2_ID :
            // if (status != I2C_OK) return status;
            // printf("Byte transmit is HAL OK!! now continue\n");

            // dev_addr = AT24C256_READ;
            // HAL_Delay(5);

            // status = I2C_Read(i2c_controller_num, dev_addr, 0, data, size);
            // if (status != I2C_OK) return status;
            // printf("read AT24C256 memory is HAL OK!! now continue\n");
            // printf("Bytes receive is HAL OK!! now continue\n");

            break;
        default:
            return I2C_ERROR;
    }
  
    return I2C_OK;
}
  