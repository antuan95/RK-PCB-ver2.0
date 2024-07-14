/******************************************************************************
 * @file    rk_mt.h
 * @brief   Header file for Sensor Controller Interface
 *
 * @details This file contains definitions, macros, structures, and function
 *          prototypes for interfacing with sensors via I2C communication on
 *          different STM32 microcontroller families.
 ******************************************************************************/

#ifndef _RK_MT_H
#define _RK_MT_H

/************ INCLUDES ********************************************************/

// Include the appropriate STM32 header based on the defined MCU type
#ifdef STM32G031xx
    #include "stm32g031xx.h"
#endif
#ifdef STM32F407xx
    #include "stm32f4xx_hal.h"
#endif
#ifdef STM32L431xx
    #include "stm32l4xx_hal.h"
#endif

/************ DEFINES **********************************************************/

#define TIMEOUT_I2C         10u   ///< Timeout value for I2C operations in milliseconds
#define SIZE_OF_MEM_ADDR    1u    ///< Size of memory address (typically 1 byte)

#define MT_CHIP_ADDRESS     0x0Cu  ///< I2C address of the chip for STM32L431xx

#define MT_SENSOR_NUMBER    2u     ///< Number of sensors

/************ TYPEDEFS ********************************************************/

/**
 * @brief Enumerate the identifiers for different sensors.
 */
typedef enum
{
  MT_MOTOR,   ///< Identifier for the motor
  MT_ARM,     ///< Identifier for the arm
  MT_ID_COUNT ///< Total count of identifiers
} MT_identifier_TypeDef;

/**
 * @brief Enumerate the initialization states.
 */
typedef enum
{
  MT_NOT_INITIALIZED, ///< Not initialized state
  MT_INITIALIZED      ///< Initialized state
} MT_initialized_TypeDef;

/**
 * @brief Structure defining a I2C instance.
 */
typedef struct
{
    I2C_HandleTypeDef *i2c_h;        ///< Pointer to the I2C handle
    MT_initialized_TypeDef initialized; ///< Initialization status
} MT_instance_TypeDef;

/************ FUNCTION PROTOTYPES *********************************************/

/**
 * @brief Initialize the sensor specified by Id with the provided I2C handle.
 *
 * @param Id Identifier of the sensor (MT_MOTOR or MT_ARM)
 * @param i2c_handle Pointer to the I2C handle to be used for communication
 */
void MT_Init(MT_identifier_TypeDef Id, I2C_HandleTypeDef *i2c_handle);

/**
 * @brief Get the position information from the specified sensor.
 *
 * @param Id Identifier of the sensor (MT_MOTOR or MT_ARM)
 */
void MT_Get_Position(MT_identifier_TypeDef Id);

/**
 * @brief Enable the specified sensor.
 *
 * @param Id Identifier of the sensor (MT_MOTOR or MT_ARM)
 */
void MT_Enable(MT_identifier_TypeDef Id);

/**
 * @brief Write data to a register of the specified sensor.
 *
 * @param Id Identifier of the sensor (MT_MOTOR or MT_ARM)
 * @param reg Register address to write to
 * @param data Pointer to the data buffer containing data to write
 */
void MT_Write_Register(MT_identifier_TypeDef Id, uint8_t reg, uint8_t *data);


/**
 * @brief Read data from a register of the specified sensor.
 *
 * @param Id Identifier of the sensor (MT_MOTOR or MT_ARM)
 * @param reg Register address to read from
 * @param data Pointer to the buffer where read data will be stored
 */
void MT_Read_Register(MT_identifier_TypeDef Id, uint8_t reg, uint8_t *data);

#endif /* _RK_MT_H */
