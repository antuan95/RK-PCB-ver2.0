/******************************************************************************
 * @file    rk_mt6701.c
 * @brief   Implementation file for RK MT6701 sensor interface
 *
 * @details This file provides the implementation of functions to interact with the RK MT6701
 *          sensor using I2C communication.
 ******************************************************************************/

/************ INCLUDES ********************************************************/
#include "rk_mt6701.h"

/************ LOCAL DEFINES ***************************************************/

/************ TYPE DEFS *******************************************************/

/************ LOCAL DATA ******************************************************/
MT_instance_TypeDef MT_instance[MT_ID_COUNT] = { 0 };

/************ LOCAL FUNCTION DECLARATION **************************************/

/************ LOCAL FUNCTION DEFINITION ***************************************/

/************ PUBLIC FUNCTION DEFINITION **************************************/
void MT_Init(MT_identifier_TypeDef Id, I2C_HandleTypeDef *i2c_handle)
{
  if((i2c_handle != NULL) && (Id < MT_ID_COUNT))
  {
    MT_instance[Id].i2c_h = i2c_handle;
    MT_instance[Id].initialized = MT_INITIALIZED;
  }
}

void MT_Write_Register(MT_identifier_TypeDef Id, uint8_t reg, uint8_t *data)
{
  if(MT_instance[Id].initialized == MT_INITIALIZED)
  {
    HAL_I2C_Mem_Write(MT_instance[Id].i2c_h, MT_CHIP_ADDRESS, reg, SIZE_OF_MEM_ADDR, data, 1, TIMEOUT_I2C);
  }
}

void MT_Read_Register(MT_identifier_TypeDef Id, uint8_t reg, uint8_t *data)
{
  if(MT_instance[Id].initialized == MT_INITIALIZED)
  {
    HAL_I2C_Mem_Read(MT_instance[Id].i2c_h, MT_CHIP_ADDRESS, reg, SIZE_OF_MEM_ADDR, data, 1, TIMEOUT_I2C);
  }
}

void MT_Read_Multiple_Registers(MT_identifier_TypeDef Id, uint8_t start_reg, uint8_t number_of_regs, uint8_t *data)
{
  if(MT_instance[Id].initialized == MT_INITIALIZED)
  {
    HAL_I2C_Mem_Read(MT_instance[Id].i2c_h, MT_CHIP_ADDRESS, start_reg, SIZE_OF_MEM_ADDR, data, number_of_regs, TIMEOUT_I2C);
  }
}
